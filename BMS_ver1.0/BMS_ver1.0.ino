#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <esp_system.h>

/*
  ELEC3117 2S BMS supervisory controller

  IMPORTANT:
  - This firmware is a supervisory layer. MM3220 and the charger hardware remain
    the independent protection/charging layers.
  - Every manual and automatic request passes through arbitrateOutputs().
  - All power outputs start LOW after every reset.
*/

// -----------------------------------------------------------------------------
// 1. Hardware mapping verified from the previous board-test firmware
// -----------------------------------------------------------------------------

constexpr int PIN_ADC_SDA      = 32;
constexpr int PIN_ADC_SCL      = 33;
constexpr int PIN_CURRENT_SDA  = 16;
constexpr int PIN_CURRENT_SCL  = 17;
constexpr int PIN_SOLAR_EN     = 25;
constexpr int PIN_LOAD_EN      = 26;
constexpr int PIN_HEATER1_EN   = 27;  // R26 / original HB_EN
constexpr int PIN_HEATER2_EN   = 13;  // R32 / original LB_EN
constexpr int PIN_LED          = 18;

// Change these only if an enable signal is electrically active LOW.
constexpr bool SOLAR_ACTIVE_HIGH  = true;
constexpr bool LOAD_ACTIVE_HIGH   = true;
constexpr bool HEATER_ACTIVE_HIGH = true;

// MCP3424 channels: CH1=B1/4, CH2=B2/4, CH3=NTC1, CH4=NTC2.
constexpr float CELL1_DIVIDER = 4.00f; // use 4.28 if 82k/25k is fitted
constexpr float CELL2_DIVIDER = 4.00f;
constexpr float CELL1_CAL     = 1.000f;
constexpr float CELL2_CAL     = 1.000f;

// INA219 is at the input. Its current is total input current, not BQ charge current.
constexpr float INA_SHUNT_OHM = 0.050f;
constexpr float INA_CURRENT_SIGN = 1.0f; // set -1.0 if positive current reads negative
constexpr float NOMINAL_CHARGE_CURRENT_A = 0.60f; // set by BQ2057 hardware, not by this firmware

// B57540G1103F000: 3V3 -> NTC -> ADC node -> 3.3k -> GND.
constexpr float NTC_SUPPLY_V       = 3.300f; // replace with measured 3V3 rail
constexpr float NTC_FIXED_OHM      = 3300.0f;
constexpr float NTC_R25_OHM        = 10000.0f;
constexpr float NTC_BETA_K         = 3492.0f;
constexpr float NTC_T25_K          = 298.15f;
constexpr float NTC_OPEN_V         = 0.020f; // open upper NTC pulls node toward 0 V
constexpr float NTC_SHORT_V        = 2.000f; // MCP3424 clips near 2.048 V
constexpr float NTC_MIN_PLAUS_C    = -30.0f;
constexpr float NTC_MAX_PLAUS_C    = 80.0f;

constexpr char WIFI_SSID[] = "ELEC3117-BMS";
constexpr char WIFI_PASSWORD[] = "elec3117";
constexpr bool WIFI_START_ON_BOOT = true;

TwoWire ADCBus(0);
TwoWire CurrentBus(1);
WebServer server(80);
Preferences preferences;

// -----------------------------------------------------------------------------
// 2. Fixed safety thresholds (not editable from the web page)
// -----------------------------------------------------------------------------

constexpr float CELL_UV_V             = 3.00f;
constexpr float CELL_CRITICAL_UV_V    = 2.70f;
constexpr float CELL_OV_V             = 4.25f;
constexpr float CELL_CRITICAL_OV_V    = 4.35f;
constexpr float CELL_PLAUS_MIN_V      = 2.30f;
constexpr float CELL_PLAUS_MAX_V      = 4.50f;
constexpr float HEATER_CELL_MIN_V     = 3.45f;
constexpr float BALANCE_CELL_MIN_V    = 3.80f;
constexpr float TEMP_REDUCE_C         = 15.0f;
constexpr float TEMP_HEATER_STOP_C    = 45.0f;
constexpr float TEMP_CRITICAL_C       = 60.0f;
constexpr float TEMP_DIFF_WARN_C      = 8.0f;
constexpr float TEMP_DIFF_FAULT_C     = 15.0f;
constexpr float CELL_DIFF_WARN_V      = 0.100f;
constexpr float CELL_DIFF_FAULT_V     = 0.200f;
constexpr uint32_t SENSOR_STALE_MS    = 2500;
constexpr uint32_t HEAT_MAX_ROUND_MS  = 30000;
constexpr uint32_t HEAT_COOLDOWN_MS   = 10000;
constexpr uint32_t MANUAL_HEATER_MS   = 10000;
constexpr uint32_t MANUAL_SOLAR_MS    = 3000;
constexpr uint32_t PWM_WINDOW_MS      = 1000;

// -----------------------------------------------------------------------------
// 3. User settings stored in ESP32 NVS
// -----------------------------------------------------------------------------

enum class ProbeMode : uint8_t { RECOVERY_VOLTAGE = 0, FIXED_INTERVAL = 1 };
enum class BalanceMode : uint8_t { OFF = 0, HEATING_ONLY = 1, INDEPENDENT = 2, BOTH = 3 };

struct UserSettings {
  float heaterStartC = 5.0f;
  float heaterStopC = 10.0f;
  float mpptVref = 10.0f;
  float balanceStartV = 0.050f;
  float balanceStopV = 0.020f;
  uint16_t probeIntervalS = 10;
  ProbeMode probeMode = ProbeMode::RECOVERY_VOLTAGE;
  BalanceMode balanceMode = BalanceMode::BOTH;
};

UserSettings settings;

float clampFloat(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

void validateSettings() {
  settings.heaterStartC = clampFloat(settings.heaterStartC, -10.0f, 15.0f);
  settings.heaterStopC = clampFloat(settings.heaterStopC, settings.heaterStartC + 2.0f, 25.0f);
  settings.mpptVref = clampFloat(settings.mpptVref, 6.0f, 14.0f);
  settings.balanceStartV = clampFloat(settings.balanceStartV, 0.030f, 0.150f);
  settings.balanceStopV = clampFloat(settings.balanceStopV, 0.010f, settings.balanceStartV - 0.010f);
  settings.probeIntervalS = constrain(settings.probeIntervalS, 2, 60);
  if ((uint8_t)settings.probeMode > 1) settings.probeMode = ProbeMode::RECOVERY_VOLTAGE;
  if ((uint8_t)settings.balanceMode > 3) settings.balanceMode = BalanceMode::BOTH;
}

void loadSettings() {
  preferences.begin("elec3117", true);
  settings.heaterStartC = preferences.getFloat("heatStart", 5.0f);
  settings.heaterStopC = preferences.getFloat("heatStop", 10.0f);
  settings.mpptVref = preferences.getFloat("mpptVref", 10.0f);
  settings.balanceStartV = preferences.getFloat("balStart", 0.050f);
  settings.balanceStopV = preferences.getFloat("balStop", 0.020f);
  settings.probeIntervalS = preferences.getUShort("probeS", 10);
  settings.probeMode = (ProbeMode)preferences.getUChar("probeMode", 0);
  settings.balanceMode = (BalanceMode)preferences.getUChar("balMode", 3);
  preferences.end();
  validateSettings();
}

void saveSettings() {
  validateSettings();
  preferences.begin("elec3117", false);
  preferences.putFloat("heatStart", settings.heaterStartC);
  preferences.putFloat("heatStop", settings.heaterStopC);
  preferences.putFloat("mpptVref", settings.mpptVref);
  preferences.putFloat("balStart", settings.balanceStartV);
  preferences.putFloat("balStop", settings.balanceStopV);
  preferences.putUShort("probeS", settings.probeIntervalS);
  preferences.putUChar("probeMode", (uint8_t)settings.probeMode);
  preferences.putUChar("balMode", (uint8_t)settings.balanceMode);
  preferences.end();
}

// -----------------------------------------------------------------------------
// 4. Measurements and non-blocking sensor drivers
// -----------------------------------------------------------------------------

struct FilteredValue {
  bool initialized = false;
  float raw = NAN;
  float fast = NAN;
  float slow = NAN;
  uint32_t updatedMs = 0;
};

struct Measurements {
  FilteredValue mcp[4];
  bool mcpFound = false;
  uint8_t mcpAddress = 0;
  uint32_t mcpLastSeenMs = 0;

  bool inaFound = false;
  bool inaValid = false;
  uint8_t inaAddress = 0;
  float inputV = NAN;
  float inputCurrentmA = NAN;
  float inputPowerW = NAN;
  float shuntmV = NAN;
  uint32_t inaUpdatedMs = 0;

  bool cell1Valid = false;
  bool cell2Valid = false;
  float cell1V = NAN;
  float cell2V = NAN;
  float cell1SlowV = NAN;
  float cell2SlowV = NAN;
  float packV = NAN;
  float cellDiffV = NAN;

  bool ntc1Valid = false;
  bool ntc2Valid = false;
  float ntc1V = NAN;
  float ntc2V = NAN;
  float temp1C = NAN;
  float temp2C = NAN;
  float tempMinC = NAN;
  float tempMaxC = NAN;
  float tempDiffC = NAN;

  float soc1 = NAN;
  float soc2 = NAN;
  float packSoc = NAN;
};

Measurements meas;

// Explicit prototype: prevents Arduino's .ino preprocessor from generating
// this declaration before FilteredValue has been defined.
void updateFilter(FilteredValue &value, float sample, uint32_t now);

void updateFilter(FilteredValue &value, float sample, uint32_t now) {
  value.raw = sample;
  if (!value.initialized) {
    value.fast = sample;
    value.slow = sample;
    value.initialized = true;
  } else {
    value.fast += 0.25f * (sample - value.fast);
    value.slow += 0.025f * (sample - value.slow);
  }
  value.updatedMs = now;
}

bool i2cDevicePresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

uint8_t findDevice(TwoWire &bus, uint8_t first, uint8_t last) {
  for (uint8_t address = first; address <= last; ++address) {
    if (i2cDevicePresent(bus, address)) return address;
  }
  return 0;
}

bool readRegister16(TwoWire &bus, uint8_t address, uint8_t reg, uint16_t &value) {
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(address, (uint8_t)2) != 2) return false;
  value = ((uint16_t)bus.read() << 8) | bus.read();
  return true;
}

uint32_t lastINAServiceMs = 0;
uint32_t lastINAProbeMs = 0;

void serviceINA219(uint32_t now) {
  if (meas.inaAddress == 0) {
    if (now - lastINAProbeMs < 2000) return;
    lastINAProbeMs = now;
    meas.inaAddress = findDevice(CurrentBus, 0x40, 0x4F);
    meas.inaFound = meas.inaAddress != 0;
    return;
  }
  if (now - lastINAServiceMs < 250) return;
  lastINAServiceMs = now;

  uint16_t shuntReg = 0, busReg = 0;
  bool ok = readRegister16(CurrentBus, meas.inaAddress, 0x01, shuntReg) &&
            readRegister16(CurrentBus, meas.inaAddress, 0x02, busReg);
  if (!ok) {
    meas.inaValid = false;
    if (now - meas.inaUpdatedMs > SENSOR_STALE_MS) {
      meas.inaAddress = 0;
      meas.inaFound = false;
    }
    return;
  }

  meas.inaFound = true;
  meas.inaValid = true;
  meas.shuntmV = (int16_t)shuntReg * 0.01f;
  meas.inputV = (busReg >> 3) * 0.004f;
  meas.inputCurrentmA = INA_CURRENT_SIGN * meas.shuntmV / INA_SHUNT_OHM;
  meas.inputPowerW = meas.inputV * meas.inputCurrentmA / 1000.0f;
  meas.inaUpdatedMs = now;
}

uint8_t mcpChannel = 1;
bool mcpWaiting = false;
uint32_t mcpStartedMs = 0;
uint32_t lastMCPProbeMs = 0;

bool startMCPConversion(uint8_t address, uint8_t channel) {
  // One-shot, 16-bit (15 SPS), PGA x1.
  uint8_t config = 0x80 | ((channel - 1) << 5) | 0x08;
  ADCBus.beginTransmission(address);
  ADCBus.write(config);
  return ADCBus.endTransmission() == 0;
}

bool finishMCPConversion(uint8_t address, float &voltage, bool &ready) {
  ready = false;
  if (ADCBus.requestFrom(address, (uint8_t)3) != 3) return false;
  uint8_t msb = ADCBus.read();
  uint8_t lsb = ADCBus.read();
  uint8_t config = ADCBus.read();
  if (config & 0x80) return true;
  int16_t raw = (int16_t)(((uint16_t)msb << 8) | lsb);
  voltage = raw * 0.0000625f;
  ready = true;
  return true;
}

void serviceMCP3424(uint32_t now) {
  if (meas.mcpAddress == 0) {
    if (now - lastMCPProbeMs < 2000) return;
    lastMCPProbeMs = now;
    meas.mcpAddress = findDevice(ADCBus, 0x68, 0x6F);
    meas.mcpFound = meas.mcpAddress != 0;
    mcpWaiting = false;
    return;
  }

  if (!mcpWaiting) {
    if (startMCPConversion(meas.mcpAddress, mcpChannel)) {
      mcpStartedMs = now;
      mcpWaiting = true;
    } else {
      meas.mcpAddress = 0;
      meas.mcpFound = false;
    }
    return;
  }

  if (now - mcpStartedMs < 72) return;
  float voltage = NAN;
  bool ready = false;
  if (!finishMCPConversion(meas.mcpAddress, voltage, ready)) {
    if (now - meas.mcpLastSeenMs > SENSOR_STALE_MS) {
      meas.mcpAddress = 0;
      meas.mcpFound = false;
    }
    mcpWaiting = false;
    return;
  }
  if (!ready) return;

  updateFilter(meas.mcp[mcpChannel - 1], voltage, now);
  meas.mcpFound = true;
  meas.mcpLastSeenMs = now;
  mcpChannel = (mcpChannel % 4) + 1;
  mcpWaiting = false;
}

bool ntcToTemperature(float voltage, float &temperatureC) {
  if (!isfinite(voltage) || voltage <= NTC_OPEN_V || voltage >= NTC_SHORT_V) return false;
  float resistance = NTC_FIXED_OHM * (NTC_SUPPLY_V / voltage - 1.0f);
  if (!isfinite(resistance) || resistance <= 0.0f) return false;
  float invT = 1.0f / NTC_T25_K + logf(resistance / NTC_R25_OHM) / NTC_BETA_K;
  if (!isfinite(invT) || invT <= 0.0f) return false;
  temperatureC = 1.0f / invT - 273.15f;
  return isfinite(temperatureC) && temperatureC >= NTC_MIN_PLAUS_C && temperatureC <= NTC_MAX_PLAUS_C;
}

float voltageToSoc(float voltage) {
  static const float volts[] = {3.00f, 3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float soc[]   = {0.0f,  2.0f,  8.0f, 15.0f, 30.0f, 50.0f, 65.0f, 80.0f, 90.0f, 100.0f};
  if (!isfinite(voltage)) return NAN;
  if (voltage <= volts[0]) return 0.0f;
  if (voltage >= volts[9]) return 100.0f;
  for (int i = 0; i < 9; ++i) {
    if (voltage <= volts[i + 1]) {
      float ratio = (voltage - volts[i]) / (volts[i + 1] - volts[i]);
      return soc[i] + ratio * (soc[i + 1] - soc[i]);
    }
  }
  return NAN;
}

void deriveMeasurements(uint32_t now) {
  bool ch1Fresh = meas.mcp[0].initialized && now - meas.mcp[0].updatedMs <= SENSOR_STALE_MS;
  bool ch2Fresh = meas.mcp[1].initialized && now - meas.mcp[1].updatedMs <= SENSOR_STALE_MS;
  meas.cell1V = ch1Fresh ? meas.mcp[0].fast * CELL1_DIVIDER * CELL1_CAL : NAN;
  meas.cell2V = ch2Fresh ? meas.mcp[1].fast * CELL2_DIVIDER * CELL2_CAL : NAN;
  meas.cell1SlowV = ch1Fresh ? meas.mcp[0].slow * CELL1_DIVIDER * CELL1_CAL : NAN;
  meas.cell2SlowV = ch2Fresh ? meas.mcp[1].slow * CELL2_DIVIDER * CELL2_CAL : NAN;
  meas.cell1Valid = ch1Fresh && meas.cell1V >= CELL_PLAUS_MIN_V && meas.cell1V <= CELL_PLAUS_MAX_V;
  meas.cell2Valid = ch2Fresh && meas.cell2V >= CELL_PLAUS_MIN_V && meas.cell2V <= CELL_PLAUS_MAX_V;
  meas.packV = meas.cell1Valid && meas.cell2Valid ? meas.cell1V + meas.cell2V : NAN;
  meas.cellDiffV = meas.cell1Valid && meas.cell2Valid ? fabsf(meas.cell1V - meas.cell2V) : NAN;

  bool ch3Fresh = meas.mcp[2].initialized && now - meas.mcp[2].updatedMs <= SENSOR_STALE_MS;
  bool ch4Fresh = meas.mcp[3].initialized && now - meas.mcp[3].updatedMs <= SENSOR_STALE_MS;
  meas.ntc1V = ch3Fresh ? meas.mcp[2].fast : NAN;
  meas.ntc2V = ch4Fresh ? meas.mcp[3].fast : NAN;
  meas.ntc1Valid = ch3Fresh && ntcToTemperature(meas.ntc1V, meas.temp1C);
  meas.ntc2Valid = ch4Fresh && ntcToTemperature(meas.ntc2V, meas.temp2C);
  if (meas.ntc1Valid && meas.ntc2Valid) {
    meas.tempMinC = fminf(meas.temp1C, meas.temp2C);
    meas.tempMaxC = fmaxf(meas.temp1C, meas.temp2C);
    meas.tempDiffC = fabsf(meas.temp1C - meas.temp2C);
  } else {
    meas.tempMinC = meas.tempMaxC = meas.tempDiffC = NAN;
  }

  meas.soc1 = meas.cell1Valid ? voltageToSoc(meas.cell1SlowV) : NAN;
  meas.soc2 = meas.cell2Valid ? voltageToSoc(meas.cell2SlowV) : NAN;
  meas.packSoc = meas.cell1Valid && meas.cell2Valid ? fminf(meas.soc1, meas.soc2) : NAN;
  if (meas.inaValid && now - meas.inaUpdatedMs > SENSOR_STALE_MS) meas.inaValid = false;
}

// -----------------------------------------------------------------------------
// 5. Fault manager: location, evidence and automatic response
// -----------------------------------------------------------------------------

enum class FaultSeverity : uint8_t { WARNING, RECOVERABLE, CRITICAL };
enum class FaultCode : uint8_t {
  MCP3424_NOT_FOUND,
  INA219_COMMUNICATION_FAULT,
  CELL1_ADC_FAULT,
  CELL2_ADC_FAULT,
  NTC1_ADC_FAULT,
  NTC2_ADC_FAULT,
  NTC1_OPEN,
  NTC1_SHORT,
  NTC1_OUT_OF_RANGE,
  NTC2_OPEN,
  NTC2_SHORT,
  NTC2_OUT_OF_RANGE,
  TEMP_B1_HIGH,
  TEMP_B2_HIGH,
  TEMP_B1_CRITICAL,
  TEMP_B2_CRITICAL,
  TEMP_DIFFERENCE_WARNING,
  TEMP_DIFFERENCE_FAULT,
  CELL1_UNDERVOLTAGE,
  CELL2_UNDERVOLTAGE,
  CELL1_CRITICAL_UNDERVOLTAGE,
  CELL2_CRITICAL_UNDERVOLTAGE,
  CELL1_OVERVOLTAGE,
  CELL2_OVERVOLTAGE,
  CELL1_CRITICAL_OVERVOLTAGE,
  CELL2_CRITICAL_OVERVOLTAGE,
  CELL_IMBALANCE_WARNING,
  CELL_IMBALANCE_FAULT,
  INPUT_COLLAPSE,
  HEATER1_NOT_EFFECTIVE,
  HEATER2_NOT_EFFECTIVE,
  HEATERS_NOT_EFFECTIVE,
  HEATER_STUCK_ON_SUSPECTED,
  WATCHDOG_RESET,
  COUNT
};

enum ActionMask : uint16_t {
  ACTION_NONE       = 0,
  ACTION_SOLAR_OFF  = 1 << 0,
  ACTION_LOAD_OFF   = 1 << 1,
  ACTION_HEAT_OFF   = 1 << 2,
  ACTION_BALANCE_OFF= 1 << 3,
  ACTION_ALL_OFF    = ACTION_SOLAR_OFF | ACTION_LOAD_OFF | ACTION_HEAT_OFF | ACTION_BALANCE_OFF
};

struct FaultRecord {
  bool condition = false;
  bool active = false;
  bool latched = false;
  uint32_t pendingSinceMs = 0;
  uint32_t firstDetectedMs = 0;
  uint32_t confirmedMs = 0;
  uint32_t clearedMs = 0;
  float measured = NAN;
  float threshold = NAN;
};

constexpr size_t FAULT_COUNT = (size_t)FaultCode::COUNT;
FaultRecord faults[FAULT_COUNT];
bool inputCollapseCondition = false;
bool heater1IneffectiveCondition = false;
bool heater2IneffectiveCondition = false;
bool heatersIneffectiveCondition = false;
bool heaterStuckCondition = false;
bool watchdogResetDetected = false;

// Explicit prototypes for functions whose signatures use custom enum types.
// Without these, some Arduino IDE/ESP32 core combinations auto-generate the
// prototypes above FaultCode/FaultSeverity and report that the types are
// undeclared.
const char *faultCodeName(FaultCode code);
FaultSeverity faultSeverity(FaultCode code);
const char *severityName(FaultSeverity severity);
const char *faultLocation(FaultCode code);
uint16_t faultActions(FaultCode code);
void updateFault(FaultCode code, bool condition, uint32_t confirmMs,
                 float measured, float threshold, uint32_t now);
void forceFault(FaultCode code, float measured, float threshold,
                bool latch, uint32_t now);

const char *faultCodeName(FaultCode code) {
  switch (code) {
    case FaultCode::MCP3424_NOT_FOUND: return "MCP3424_NOT_FOUND";
    case FaultCode::INA219_COMMUNICATION_FAULT: return "INA219_COMMUNICATION_FAULT";
    case FaultCode::CELL1_ADC_FAULT: return "CELL1_ADC_FAULT";
    case FaultCode::CELL2_ADC_FAULT: return "CELL2_ADC_FAULT";
    case FaultCode::NTC1_ADC_FAULT: return "NTC1_ADC_FAULT";
    case FaultCode::NTC2_ADC_FAULT: return "NTC2_ADC_FAULT";
    case FaultCode::NTC1_OPEN: return "NTC1_OPEN";
    case FaultCode::NTC1_SHORT: return "NTC1_SHORT";
    case FaultCode::NTC1_OUT_OF_RANGE: return "NTC1_OUT_OF_RANGE";
    case FaultCode::NTC2_OPEN: return "NTC2_OPEN";
    case FaultCode::NTC2_SHORT: return "NTC2_SHORT";
    case FaultCode::NTC2_OUT_OF_RANGE: return "NTC2_OUT_OF_RANGE";
    case FaultCode::TEMP_B1_HIGH: return "TEMP_B1_HIGH";
    case FaultCode::TEMP_B2_HIGH: return "TEMP_B2_HIGH";
    case FaultCode::TEMP_B1_CRITICAL: return "TEMP_B1_CRITICAL";
    case FaultCode::TEMP_B2_CRITICAL: return "TEMP_B2_CRITICAL";
    case FaultCode::TEMP_DIFFERENCE_WARNING: return "TEMP_DIFFERENCE_WARNING";
    case FaultCode::TEMP_DIFFERENCE_FAULT: return "TEMP_DIFFERENCE_FAULT";
    case FaultCode::CELL1_UNDERVOLTAGE: return "CELL1_UNDERVOLTAGE";
    case FaultCode::CELL2_UNDERVOLTAGE: return "CELL2_UNDERVOLTAGE";
    case FaultCode::CELL1_CRITICAL_UNDERVOLTAGE: return "CELL1_CRITICAL_UNDERVOLTAGE";
    case FaultCode::CELL2_CRITICAL_UNDERVOLTAGE: return "CELL2_CRITICAL_UNDERVOLTAGE";
    case FaultCode::CELL1_OVERVOLTAGE: return "CELL1_OVERVOLTAGE";
    case FaultCode::CELL2_OVERVOLTAGE: return "CELL2_OVERVOLTAGE";
    case FaultCode::CELL1_CRITICAL_OVERVOLTAGE: return "CELL1_CRITICAL_OVERVOLTAGE";
    case FaultCode::CELL2_CRITICAL_OVERVOLTAGE: return "CELL2_CRITICAL_OVERVOLTAGE";
    case FaultCode::CELL_IMBALANCE_WARNING: return "CELL_IMBALANCE_WARNING";
    case FaultCode::CELL_IMBALANCE_FAULT: return "CELL_IMBALANCE_FAULT";
    case FaultCode::INPUT_COLLAPSE: return "INPUT_COLLAPSE";
    case FaultCode::HEATER1_NOT_EFFECTIVE: return "HEATER1_NOT_EFFECTIVE";
    case FaultCode::HEATER2_NOT_EFFECTIVE: return "HEATER2_NOT_EFFECTIVE";
    case FaultCode::HEATERS_NOT_EFFECTIVE: return "HEATERS_NOT_EFFECTIVE";
    case FaultCode::HEATER_STUCK_ON_SUSPECTED: return "HEATER_STUCK_ON_SUSPECTED";
    case FaultCode::WATCHDOG_RESET: return "WATCHDOG_RESET";
    default: return "UNKNOWN";
  }
}

FaultSeverity faultSeverity(FaultCode code) {
  switch (code) {
    case FaultCode::MCP3424_NOT_FOUND:
    case FaultCode::CELL1_ADC_FAULT:
    case FaultCode::CELL2_ADC_FAULT:
    case FaultCode::TEMP_B1_CRITICAL:
    case FaultCode::TEMP_B2_CRITICAL:
    case FaultCode::CELL1_CRITICAL_UNDERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_UNDERVOLTAGE:
    case FaultCode::CELL1_CRITICAL_OVERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_OVERVOLTAGE:
    case FaultCode::HEATER_STUCK_ON_SUSPECTED:
      return FaultSeverity::CRITICAL;
    case FaultCode::TEMP_DIFFERENCE_WARNING:
    case FaultCode::CELL_IMBALANCE_WARNING:
    case FaultCode::HEATER1_NOT_EFFECTIVE:
    case FaultCode::HEATER2_NOT_EFFECTIVE:
    case FaultCode::HEATERS_NOT_EFFECTIVE:
    case FaultCode::WATCHDOG_RESET:
      return FaultSeverity::WARNING;
    default:
      return FaultSeverity::RECOVERABLE;
  }
}

const char *severityName(FaultSeverity severity) {
  switch (severity) {
    case FaultSeverity::WARNING: return "Warning";
    case FaultSeverity::RECOVERABLE: return "Recoverable";
    case FaultSeverity::CRITICAL: return "Critical";
  }
  return "Unknown";
}

const char *faultLocation(FaultCode code) {
  switch (code) {
    case FaultCode::MCP3424_NOT_FOUND: return "MCP3424 / ADC I2C bus";
    case FaultCode::INA219_COMMUNICATION_FAULT: return "INA219 / input I2C bus";
    case FaultCode::CELL1_ADC_FAULT: return "Battery 1 / MCP3424 CH1";
    case FaultCode::CELL2_ADC_FAULT: return "Battery 2 / MCP3424 CH2";
    case FaultCode::NTC1_ADC_FAULT:
    case FaultCode::NTC1_OPEN:
    case FaultCode::NTC1_SHORT:
    case FaultCode::NTC1_OUT_OF_RANGE: return "NTC1 on Battery 1 / MCP3424 CH3";
    case FaultCode::NTC2_ADC_FAULT:
    case FaultCode::NTC2_OPEN:
    case FaultCode::NTC2_SHORT:
    case FaultCode::NTC2_OUT_OF_RANGE: return "NTC2 on Battery 2 / MCP3424 CH4";
    case FaultCode::TEMP_B1_HIGH:
    case FaultCode::TEMP_B1_CRITICAL: return "Battery 1 temperature / NTC1";
    case FaultCode::TEMP_B2_HIGH:
    case FaultCode::TEMP_B2_CRITICAL: return "Battery 2 temperature / NTC2";
    case FaultCode::TEMP_DIFFERENCE_WARNING:
    case FaultCode::TEMP_DIFFERENCE_FAULT: return "Battery pack thermal distribution";
    case FaultCode::CELL1_UNDERVOLTAGE:
    case FaultCode::CELL1_CRITICAL_UNDERVOLTAGE:
    case FaultCode::CELL1_OVERVOLTAGE:
    case FaultCode::CELL1_CRITICAL_OVERVOLTAGE: return "Battery 1 / MCP3424 CH1";
    case FaultCode::CELL2_UNDERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_UNDERVOLTAGE:
    case FaultCode::CELL2_OVERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_OVERVOLTAGE: return "Battery 2 / MCP3424 CH2";
    case FaultCode::CELL_IMBALANCE_WARNING:
    case FaultCode::CELL_IMBALANCE_FAULT: return "Battery 1 versus Battery 2";
    case FaultCode::INPUT_COLLAPSE: return "External input / solar DC-DC path";
    case FaultCode::HEATER1_NOT_EFFECTIVE: return "Heater1 / R26 / GPIO27 path";
    case FaultCode::HEATER2_NOT_EFFECTIVE: return "Heater2 / R32 / GPIO13 path";
    case FaultCode::HEATERS_NOT_EFFECTIVE: return "Both heater paths or pack thermal contact";
    case FaultCode::HEATER_STUCK_ON_SUSPECTED: return "Heater MOSFET/output path";
    case FaultCode::WATCHDOG_RESET: return "ESP32 firmware/system";
    default: return "System";
  }
}

uint16_t faultActions(FaultCode code) {
  switch (code) {
    case FaultCode::MCP3424_NOT_FOUND:
    case FaultCode::CELL1_ADC_FAULT:
    case FaultCode::CELL2_ADC_FAULT:
    case FaultCode::TEMP_B1_CRITICAL:
    case FaultCode::TEMP_B2_CRITICAL:
    case FaultCode::HEATER_STUCK_ON_SUSPECTED:
      return ACTION_ALL_OFF;
    case FaultCode::INA219_COMMUNICATION_FAULT:
    case FaultCode::INPUT_COLLAPSE:
    case FaultCode::CELL1_OVERVOLTAGE:
    case FaultCode::CELL2_OVERVOLTAGE:
    case FaultCode::CELL1_CRITICAL_OVERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_OVERVOLTAGE:
      return ACTION_SOLAR_OFF;
    case FaultCode::NTC1_ADC_FAULT:
    case FaultCode::NTC2_ADC_FAULT:
    case FaultCode::NTC1_OPEN:
    case FaultCode::NTC1_SHORT:
    case FaultCode::NTC1_OUT_OF_RANGE:
    case FaultCode::NTC2_OPEN:
    case FaultCode::NTC2_SHORT:
    case FaultCode::NTC2_OUT_OF_RANGE:
    case FaultCode::TEMP_DIFFERENCE_FAULT:
      return ACTION_HEAT_OFF | ACTION_BALANCE_OFF;
    case FaultCode::TEMP_B1_HIGH:
    case FaultCode::TEMP_B2_HIGH:
      return ACTION_SOLAR_OFF | ACTION_HEAT_OFF | ACTION_BALANCE_OFF;
    case FaultCode::CELL1_UNDERVOLTAGE:
    case FaultCode::CELL2_UNDERVOLTAGE:
    case FaultCode::CELL1_CRITICAL_UNDERVOLTAGE:
    case FaultCode::CELL2_CRITICAL_UNDERVOLTAGE:
      return ACTION_LOAD_OFF | ACTION_HEAT_OFF | ACTION_BALANCE_OFF;
    case FaultCode::CELL_IMBALANCE_FAULT:
      return ACTION_LOAD_OFF | ACTION_HEAT_OFF;
    case FaultCode::HEATER1_NOT_EFFECTIVE:
    case FaultCode::HEATER2_NOT_EFFECTIVE:
    case FaultCode::HEATERS_NOT_EFFECTIVE:
      return ACTION_HEAT_OFF;
    default:
      return ACTION_NONE;
  }
}

const char *actionText(uint16_t mask) {
  if (mask == ACTION_NONE) return "Warning only; continue monitoring";
  if (mask == ACTION_ALL_OFF) return "Solar, load, heating and balancing disabled";
  if (mask == ACTION_SOLAR_OFF) return "Solar EN disabled";
  if (mask == (ACTION_HEAT_OFF | ACTION_BALANCE_OFF)) return "Both heaters and balancing disabled";
  if (mask == (ACTION_LOAD_OFF | ACTION_HEAT_OFF | ACTION_BALANCE_OFF)) return "Load, both heaters and balancing disabled; charging may recover cells";
  if (mask == (ACTION_SOLAR_OFF | ACTION_HEAT_OFF | ACTION_BALANCE_OFF)) return "Solar, both heaters and balancing disabled";
  if (mask == (ACTION_LOAD_OFF | ACTION_HEAT_OFF)) return "Load and normal heating disabled; safe balancing may continue";
  if (mask == ACTION_HEAT_OFF) return "Normal heating disabled";
  return "Related power functions restricted";
}

void updateFault(FaultCode code, bool condition, uint32_t confirmMs, float measured, float threshold, uint32_t now) {
  FaultRecord &record = faults[(size_t)code];
  if (condition) {
    if (!record.condition) {
      record.condition = true;
      record.pendingSinceMs = now;
      if (record.firstDetectedMs == 0) record.firstDetectedMs = now;
    }
    record.measured = measured;
    record.threshold = threshold;
    if (!record.active && now - record.pendingSinceMs >= confirmMs) {
      record.active = true;
      record.confirmedMs = now;
      if (faultSeverity(code) == FaultSeverity::CRITICAL) record.latched = true;
    }
  } else {
    record.condition = false;
    record.pendingSinceMs = 0;
    if (record.active && !record.latched) {
      record.active = false;
      record.clearedMs = now;
    }
  }
}

void forceFault(FaultCode code, float measured, float threshold, bool latch, uint32_t now) {
  FaultRecord &record = faults[(size_t)code];
  record.condition = true;
  record.active = true;
  record.latched = latch;
  record.measured = measured;
  record.threshold = threshold;
  if (record.firstDetectedMs == 0) record.firstDetectedMs = now;
  record.confirmedMs = now;
}

bool anyActiveCriticalFault() {
  for (size_t i = 0; i < FAULT_COUNT; ++i) {
    if (faults[i].active && faultSeverity((FaultCode)i) == FaultSeverity::CRITICAL) return true;
  }
  return false;
}

uint16_t combinedFaultActions() {
  uint16_t actions = ACTION_NONE;
  for (size_t i = 0; i < FAULT_COUNT; ++i) {
    if (faults[i].active) actions |= faultActions((FaultCode)i);
  }
  return actions;
}

void clearInactiveLatchedFaults(uint32_t now) {
  for (size_t i = 0; i < FAULT_COUNT; ++i) {
    FaultRecord &record = faults[i];
    if (record.latched && !record.condition) {
      record.latched = false;
      record.active = false;
      record.clearedMs = now;
    }
  }
}

void updateFaults(uint32_t now) {
  bool bootGraceOver = now > 3000;
  updateFault(FaultCode::MCP3424_NOT_FOUND, bootGraceOver && !meas.mcpFound, 1500, NAN, NAN, now);
  updateFault(FaultCode::INA219_COMMUNICATION_FAULT, bootGraceOver && !meas.inaValid, 1500, NAN, NAN, now);

  bool ch1Fresh = meas.mcp[0].initialized && now - meas.mcp[0].updatedMs <= SENSOR_STALE_MS;
  bool ch2Fresh = meas.mcp[1].initialized && now - meas.mcp[1].updatedMs <= SENSOR_STALE_MS;
  bool ch3Fresh = meas.mcp[2].initialized && now - meas.mcp[2].updatedMs <= SENSOR_STALE_MS;
  bool ch4Fresh = meas.mcp[3].initialized && now - meas.mcp[3].updatedMs <= SENSOR_STALE_MS;
  updateFault(FaultCode::CELL1_ADC_FAULT, bootGraceOver && (!ch1Fresh || !meas.cell1Valid), 1000, meas.cell1V, CELL_PLAUS_MIN_V, now);
  updateFault(FaultCode::CELL2_ADC_FAULT, bootGraceOver && (!ch2Fresh || !meas.cell2Valid), 1000, meas.cell2V, CELL_PLAUS_MIN_V, now);
  updateFault(FaultCode::NTC1_ADC_FAULT, bootGraceOver && !ch3Fresh, 1000, meas.ntc1V, NAN, now);
  updateFault(FaultCode::NTC2_ADC_FAULT, bootGraceOver && !ch4Fresh, 1000, meas.ntc2V, NAN, now);
  updateFault(FaultCode::NTC1_OPEN, ch3Fresh && meas.ntc1V <= NTC_OPEN_V, 800, meas.ntc1V, NTC_OPEN_V, now);
  updateFault(FaultCode::NTC1_SHORT, ch3Fresh && meas.ntc1V >= NTC_SHORT_V, 800, meas.ntc1V, NTC_SHORT_V, now);
  updateFault(FaultCode::NTC1_OUT_OF_RANGE, ch3Fresh && meas.ntc1V > NTC_OPEN_V && meas.ntc1V < NTC_SHORT_V && !meas.ntc1Valid, 1000, meas.ntc1V, NAN, now);
  updateFault(FaultCode::NTC2_OPEN, ch4Fresh && meas.ntc2V <= NTC_OPEN_V, 800, meas.ntc2V, NTC_OPEN_V, now);
  updateFault(FaultCode::NTC2_SHORT, ch4Fresh && meas.ntc2V >= NTC_SHORT_V, 800, meas.ntc2V, NTC_SHORT_V, now);
  updateFault(FaultCode::NTC2_OUT_OF_RANGE, ch4Fresh && meas.ntc2V > NTC_OPEN_V && meas.ntc2V < NTC_SHORT_V && !meas.ntc2Valid, 1000, meas.ntc2V, NAN, now);

  updateFault(FaultCode::TEMP_B1_HIGH, meas.ntc1Valid && meas.temp1C >= TEMP_HEATER_STOP_C, 1000, meas.temp1C, TEMP_HEATER_STOP_C, now);
  updateFault(FaultCode::TEMP_B2_HIGH, meas.ntc2Valid && meas.temp2C >= TEMP_HEATER_STOP_C, 1000, meas.temp2C, TEMP_HEATER_STOP_C, now);
  updateFault(FaultCode::TEMP_B1_CRITICAL, meas.ntc1Valid && meas.temp1C >= TEMP_CRITICAL_C, 500, meas.temp1C, TEMP_CRITICAL_C, now);
  updateFault(FaultCode::TEMP_B2_CRITICAL, meas.ntc2Valid && meas.temp2C >= TEMP_CRITICAL_C, 500, meas.temp2C, TEMP_CRITICAL_C, now);
  updateFault(FaultCode::TEMP_DIFFERENCE_WARNING, isfinite(meas.tempDiffC) && meas.tempDiffC >= TEMP_DIFF_WARN_C, 3000, meas.tempDiffC, TEMP_DIFF_WARN_C, now);
  updateFault(FaultCode::TEMP_DIFFERENCE_FAULT, isfinite(meas.tempDiffC) && meas.tempDiffC >= TEMP_DIFF_FAULT_C, 10000, meas.tempDiffC, TEMP_DIFF_FAULT_C, now);

  updateFault(FaultCode::CELL1_UNDERVOLTAGE, meas.cell1Valid && meas.cell1V <= CELL_UV_V, 1500, meas.cell1V, CELL_UV_V, now);
  updateFault(FaultCode::CELL2_UNDERVOLTAGE, meas.cell2Valid && meas.cell2V <= CELL_UV_V, 1500, meas.cell2V, CELL_UV_V, now);
  updateFault(FaultCode::CELL1_CRITICAL_UNDERVOLTAGE, meas.cell1Valid && meas.cell1V <= CELL_CRITICAL_UV_V, 500, meas.cell1V, CELL_CRITICAL_UV_V, now);
  updateFault(FaultCode::CELL2_CRITICAL_UNDERVOLTAGE, meas.cell2Valid && meas.cell2V <= CELL_CRITICAL_UV_V, 500, meas.cell2V, CELL_CRITICAL_UV_V, now);
  updateFault(FaultCode::CELL1_OVERVOLTAGE, meas.cell1Valid && meas.cell1V >= CELL_OV_V, 1000, meas.cell1V, CELL_OV_V, now);
  updateFault(FaultCode::CELL2_OVERVOLTAGE, meas.cell2Valid && meas.cell2V >= CELL_OV_V, 1000, meas.cell2V, CELL_OV_V, now);
  updateFault(FaultCode::CELL1_CRITICAL_OVERVOLTAGE, meas.cell1Valid && meas.cell1V >= CELL_CRITICAL_OV_V, 500, meas.cell1V, CELL_CRITICAL_OV_V, now);
  updateFault(FaultCode::CELL2_CRITICAL_OVERVOLTAGE, meas.cell2Valid && meas.cell2V >= CELL_CRITICAL_OV_V, 500, meas.cell2V, CELL_CRITICAL_OV_V, now);
  updateFault(FaultCode::CELL_IMBALANCE_WARNING, isfinite(meas.cellDiffV) && meas.cellDiffV >= CELL_DIFF_WARN_V, 3000, meas.cellDiffV, CELL_DIFF_WARN_V, now);
  updateFault(FaultCode::CELL_IMBALANCE_FAULT, isfinite(meas.cellDiffV) && meas.cellDiffV >= CELL_DIFF_FAULT_V, 3000, meas.cellDiffV, CELL_DIFF_FAULT_V, now);
  updateFault(FaultCode::INPUT_COLLAPSE, inputCollapseCondition, 0, meas.inputV, settings.mpptVref - 0.5f, now);
  updateFault(FaultCode::HEATER1_NOT_EFFECTIVE, heater1IneffectiveCondition, 0, NAN, 0.3f, now);
  updateFault(FaultCode::HEATER2_NOT_EFFECTIVE, heater2IneffectiveCondition, 0, NAN, 0.3f, now);
  updateFault(FaultCode::HEATERS_NOT_EFFECTIVE, heatersIneffectiveCondition, 0, NAN, 0.3f, now);
  updateFault(FaultCode::HEATER_STUCK_ON_SUSPECTED, heaterStuckCondition, 0, NAN, NAN, now);
  updateFault(FaultCode::WATCHDOG_RESET, watchdogResetDetected, 0, NAN, NAN, now);
}

// -----------------------------------------------------------------------------
// 6. Parallel controllers: Solar, pack heating, balancing and output arbitration
// -----------------------------------------------------------------------------

enum class SystemState : uint8_t { BOOT, NORMAL, LOW_POWER, SAFE_MODE, CRITICAL_FAULT };
enum class SolarState : uint8_t { OFF, WAIT_RECOVERY, PROBE, RUN, WEAK, FAULT };

SystemState systemState = SystemState::BOOT;
SolarState solarState = SolarState::OFF;
bool autoControlEnabled = false; // deliberately false after every reboot
bool safeMode = false;
bool loadUserRequest = false;
bool packHeatingDemand = false;
bool noSolarHeatEnergyLatched = false;
int8_t balanceTarget = 0; // 0=none, 1=discharge B1/R26, 2=discharge B2/R32

uint32_t solarStateSinceMs = 0;
uint32_t lastSolarOffMs = 0;
uint32_t manualSolarUntilMs = 0;
float solarOpenCircuitV = NAN;
float lastCollapseV = NAN;
uint8_t collapseCount = 0;
uint8_t solarFailedProbes = 0;

uint32_t manualHeater1UntilMs = 0;
uint32_t manualHeater2UntilMs = 0;
float requestedHeatDuty1 = 0.0f;
float requestedHeatDuty2 = 0.0f;
float requestedBalanceDuty1 = 0.0f;
float requestedBalanceDuty2 = 0.0f;
float finalDuty1 = 0.0f;
float finalDuty2 = 0.0f;

bool solarOutput = false;
bool loadOutput = false;
bool heater1Output = false;
bool heater2Output = false;
bool wifiEnabled = false;
uint32_t wifiOffAtMs = 0;
uint32_t dutyWindowStartMs = 0;
bool powerRoundActive = false;
uint32_t powerRoundStartedMs = 0;
uint32_t cooldownUntilMs = 0;

uint32_t heatDiagnosticStartedMs = 0;
uint32_t heatActualOnMs = 0;
uint32_t heatLastAccountingMs = 0;
float heatDiagnosticStartTempC = NAN;
uint8_t heatDiagnosticMask = 0;
uint32_t heatersOffSinceMs = 0;
float heatersOffStartTempC = NAN;

// Explicit prototype for Arduino .ino preprocessing compatibility.
void enterSolarState(SolarState state, uint32_t now);

const char *systemStateName() {
  switch (systemState) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::NORMAL: return "NORMAL";
    case SystemState::LOW_POWER: return "LOW_POWER";
    case SystemState::SAFE_MODE: return "SAFE_MODE";
    case SystemState::CRITICAL_FAULT: return "CRITICAL_FAULT";
  }
  return "UNKNOWN";
}

const char *solarStateName() {
  switch (solarState) {
    case SolarState::OFF: return "OFF";
    case SolarState::WAIT_RECOVERY: return "WAIT_RECOVERY";
    case SolarState::PROBE: return "PROBE";
    case SolarState::RUN: return "RUN";
    case SolarState::WEAK: return "WEAK";
    case SolarState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char *balanceStateName() {
  if (balanceTarget == 1) return "B1_HIGH_R26";
  if (balanceTarget == 2) return "B2_HIGH_R32";
  return "OFF";
}

void enterSolarState(SolarState state, uint32_t now) {
  solarState = state;
  solarStateSinceMs = now;
  if (state == SolarState::WAIT_RECOVERY || state == SolarState::OFF || state == SolarState::WEAK) {
    lastSolarOffMs = now;
  }
}

void updateSystemState(uint32_t now) {
  if (now < 3000) systemState = SystemState::BOOT;
  else if (safeMode) systemState = SystemState::SAFE_MODE;
  else if (anyActiveCriticalFault()) systemState = SystemState::CRITICAL_FAULT;
  else if (isfinite(meas.packSoc) && meas.packSoc <= 20.0f) systemState = SystemState::LOW_POWER;
  else systemState = SystemState::NORMAL;
}

bool manualSolarRequested(uint32_t now) {
  return manualSolarUntilMs != 0 && (int32_t)(manualSolarUntilMs - now) > 0;
}

void updateSolarController(uint32_t now) {
  bool controllerRequested = autoControlEnabled || manualSolarRequested(now);
  bool canMeasureInput = meas.inaValid;
  bool cellsCanAcceptCharge = meas.cell1Valid && meas.cell2Valid &&
    meas.cell1V < CELL_OV_V && meas.cell2V < CELL_OV_V;

  if (!controllerRequested || safeMode) {
    inputCollapseCondition = false;
    enterSolarState(SolarState::OFF, now);
    return;
  }
  if (!canMeasureInput || !cellsCanAcceptCharge) {
    enterSolarState(SolarState::FAULT, now);
    return;
  }

  switch (solarState) {
    case SolarState::OFF:
      solarOpenCircuitV = meas.inputV;
      enterSolarState(SolarState::WAIT_RECOVERY, now);
      break;

    case SolarState::WAIT_RECOVERY: {
      bool minimumOffTime = now - lastSolarOffMs >= 2000;
      bool recovered = false;
      if (settings.probeMode == ProbeMode::FIXED_INTERVAL) {
        recovered = now - lastSolarOffMs >= (uint32_t)settings.probeIntervalS * 1000UL;
      } else if (isfinite(lastCollapseV)) {
        recovered = meas.inputV >= lastCollapseV + 0.50f;
      } else {
        recovered = meas.inputV >= settings.mpptVref;
      }
      if (manualSolarRequested(now)) recovered = true;
      if (minimumOffTime && recovered) {
        solarOpenCircuitV = meas.inputV;
        inputCollapseCondition = false;
        collapseCount = 0;
        enterSolarState(SolarState::PROBE, now);
      }
      break;
    }

    case SolarState::PROBE:
      if (now - solarStateSinceMs >= 500) {
        bool voltageGood = meas.inputV >= settings.mpptVref - 0.50f;
        bool dropAcceptable = !isfinite(solarOpenCircuitV) || solarOpenCircuitV - meas.inputV <= 1.00f;
        if (voltageGood && dropAcceptable) {
          solarFailedProbes = 0;
          inputCollapseCondition = false;
          enterSolarState(SolarState::RUN, now);
        } else {
          lastCollapseV = meas.inputV;
          solarFailedProbes++;
          inputCollapseCondition = true;
          enterSolarState(SolarState::WEAK, now);
        }
      }
      break;

    case SolarState::RUN: {
      bool weak = meas.inputV < settings.mpptVref - 0.50f ||
                  (isfinite(solarOpenCircuitV) && solarOpenCircuitV - meas.inputV > 1.00f);
      collapseCount = weak ? (uint8_t)min(255, collapseCount + 1) : 0;
      if (collapseCount >= 3) {
        lastCollapseV = meas.inputV;
        inputCollapseCondition = true;
        enterSolarState(SolarState::WEAK, now);
      }
      break;
    }

    case SolarState::WEAK:
      if (now - solarStateSinceMs >= 300) enterSolarState(SolarState::WAIT_RECOVERY, now);
      break;

    case SolarState::FAULT:
      if (canMeasureInput && cellsCanAcceptCharge) enterSolarState(SolarState::WAIT_RECOVERY, now);
      break;
  }
}

void updateHeatingDemand() {
  if (!autoControlEnabled || !meas.ntc1Valid || !meas.ntc2Valid) {
    packHeatingDemand = false;
    return;
  }
  if (meas.tempMinC < settings.heaterStartC) packHeatingDemand = true;
  if (meas.tempMinC >= settings.heaterStopC || meas.tempMaxC >= TEMP_HEATER_STOP_C) packHeatingDemand = false;
}

void updateNoSolarHeatEnergyLatch() {
  if (!meas.cell1Valid || !meas.cell2Valid || !isfinite(meas.packSoc)) {
    noSolarHeatEnergyLatched = false;
    return;
  }
  if (!noSolarHeatEnergyLatched && meas.packSoc >= 40.0f && meas.cell1V >= 3.50f && meas.cell2V >= 3.50f) {
    noSolarHeatEnergyLatched = true;
  }
  if (noSolarHeatEnergyLatched && (meas.packSoc <= 35.0f || meas.cell1V <= 3.45f || meas.cell2V <= 3.45f)) {
    noSolarHeatEnergyLatched = false;
  }
}

void updateBalanceTarget() {
  if (!autoControlEnabled || settings.balanceMode == BalanceMode::OFF || !meas.cell1Valid || !meas.cell2Valid) {
    balanceTarget = 0;
    return;
  }
  float signedDifference = meas.cell1V - meas.cell2V;
  if (balanceTarget != 0 && fabsf(signedDifference) <= settings.balanceStopV) balanceTarget = 0;
  if (balanceTarget == 0 && fabsf(signedDifference) >= settings.balanceStartV) {
    balanceTarget = signedDifference > 0.0f ? 1 : 2;
  }
}

float baseHeatingDuty() {
  if (!packHeatingDemand || !isfinite(meas.tempMinC) || !isfinite(meas.tempMaxC)) return 0.0f;
  float duty = meas.tempMinC < 0.0f ? 0.70f : (meas.tempMinC < 5.0f ? 0.60f : 0.45f);
  if (meas.tempMaxC >= TEMP_REDUCE_C) duty = fminf(duty, 0.30f);
  if (isfinite(meas.tempDiffC) && meas.tempDiffC >= TEMP_DIFF_WARN_C) duty *= 0.5f;
  return duty;
}

void calculateHeaterAndBalanceRequests(uint32_t now) {
  requestedHeatDuty1 = requestedHeatDuty2 = 0.0f;
  requestedBalanceDuty1 = requestedBalanceDuty2 = 0.0f;
  updateHeatingDemand();
  updateNoSolarHeatEnergyLatch();
  updateBalanceTarget();

  bool stableInput = solarState == SolarState::RUN;
  bool autoHeatEnergyAllowed = stableInput || noSolarHeatEnergyLatched;
  float baseDuty = autoHeatEnergyAllowed ? baseHeatingDuty() : 0.0f;

  bool combinedMode = settings.balanceMode == BalanceMode::HEATING_ONLY || settings.balanceMode == BalanceMode::BOTH;
  if (baseDuty > 0.0f) {
    requestedHeatDuty1 = baseDuty;
    requestedHeatDuty2 = baseDuty;
    if (combinedMode && balanceTarget != 0) {
      constexpr float adjustment = 0.15f;
      if (balanceTarget == 1) {
        requestedHeatDuty1 = fminf(1.0f, baseDuty + adjustment);
        requestedHeatDuty2 = fmaxf(0.0f, baseDuty - adjustment);
      } else {
        requestedHeatDuty2 = fminf(1.0f, baseDuty + adjustment);
        requestedHeatDuty1 = fmaxf(0.0f, baseDuty - adjustment);
      }
    }
  }

  bool independentMode = settings.balanceMode == BalanceMode::INDEPENDENT || settings.balanceMode == BalanceMode::BOTH;
  bool independentEnergyAllowed = meas.cell1Valid && meas.cell2Valid &&
    meas.cell1V >= BALANCE_CELL_MIN_V && meas.cell2V >= BALANCE_CELL_MIN_V;
  if (independentMode && independentEnergyAllowed && baseDuty == 0.0f) {
    if (balanceTarget == 1) requestedBalanceDuty1 = 0.40f;
    if (balanceTarget == 2) requestedBalanceDuty2 = 0.40f;
  }

  if (manualHeater1UntilMs != 0 && (int32_t)(manualHeater1UntilMs - now) > 0) {
    requestedHeatDuty1 = fmaxf(requestedHeatDuty1, 0.50f);
  }
  if (manualHeater2UntilMs != 0 && (int32_t)(manualHeater2UntilMs - now) > 0) {
    requestedHeatDuty2 = fmaxf(requestedHeatDuty2, 0.50f);
  }
}

void writeEnabled(int pin, bool enabled, bool activeHigh) {
  digitalWrite(pin, (enabled == activeHigh) ? HIGH : LOW);
}

void allPowerOutputsOff() {
  writeEnabled(PIN_SOLAR_EN, false, SOLAR_ACTIVE_HIGH);
  writeEnabled(PIN_LOAD_EN, false, LOAD_ACTIVE_HIGH);
  writeEnabled(PIN_HEATER1_EN, false, HEATER_ACTIVE_HIGH);
  writeEnabled(PIN_HEATER2_EN, false, HEATER_ACTIVE_HIGH);
  solarOutput = loadOutput = heater1Output = heater2Output = false;
}

void arbitrateOutputs(uint32_t now) {
  uint16_t actions = combinedFaultActions();
  if (safeMode) actions |= ACTION_ALL_OFF;

  bool cellHeaterSafe = meas.cell1Valid && meas.cell2Valid &&
    meas.cell1V > HEATER_CELL_MIN_V && meas.cell2V > HEATER_CELL_MIN_V;
  bool thermalSafe = meas.ntc1Valid && meas.ntc2Valid &&
    meas.tempMaxC < TEMP_HEATER_STOP_C && meas.tempDiffC < TEMP_DIFF_FAULT_C;

  float heat1 = (actions & ACTION_HEAT_OFF) ? 0.0f : requestedHeatDuty1;
  float heat2 = (actions & ACTION_HEAT_OFF) ? 0.0f : requestedHeatDuty2;
  float bal1 = (actions & ACTION_BALANCE_OFF) ? 0.0f : requestedBalanceDuty1;
  float bal2 = (actions & ACTION_BALANCE_OFF) ? 0.0f : requestedBalanceDuty2;
  if (!cellHeaterSafe || !thermalSafe) heat1 = heat2 = bal1 = bal2 = 0.0f;

  finalDuty1 = fmaxf(heat1, bal1);
  finalDuty2 = fmaxf(heat2, bal2);
  bool anyPowerRequest = finalDuty1 > 0.0f || finalDuty2 > 0.0f;

  if ((int32_t)(cooldownUntilMs - now) > 0) {
    finalDuty1 = finalDuty2 = 0.0f;
  } else if (anyPowerRequest) {
    if (!powerRoundActive) {
      powerRoundActive = true;
      powerRoundStartedMs = now;
    } else if (now - powerRoundStartedMs >= HEAT_MAX_ROUND_MS) {
      powerRoundActive = false;
      cooldownUntilMs = now + HEAT_COOLDOWN_MS;
      finalDuty1 = finalDuty2 = 0.0f;
    }
  } else {
    powerRoundActive = false;
  }

  if (now - dutyWindowStartMs >= PWM_WINDOW_MS) {
    dutyWindowStartMs = now - ((now - dutyWindowStartMs) % PWM_WINDOW_MS);
  }
  uint32_t inWindowMs = now - dutyWindowStartMs;
  heater1Output = finalDuty1 > 0.0f && inWindowMs < (uint32_t)(finalDuty1 * PWM_WINDOW_MS);
  heater2Output = finalDuty2 > 0.0f && inWindowMs < (uint32_t)(finalDuty2 * PWM_WINDOW_MS);

  bool solarRequested = solarState == SolarState::PROBE || solarState == SolarState::RUN;
  solarOutput = solarRequested && !(actions & ACTION_SOLAR_OFF);
  loadOutput = loadUserRequest && !(actions & ACTION_LOAD_OFF);

  writeEnabled(PIN_SOLAR_EN, solarOutput, SOLAR_ACTIVE_HIGH);
  writeEnabled(PIN_LOAD_EN, loadOutput, LOAD_ACTIVE_HIGH);
  writeEnabled(PIN_HEATER1_EN, heater1Output, HEATER_ACTIVE_HIGH);
  writeEnabled(PIN_HEATER2_EN, heater2Output, HEATER_ACTIVE_HIGH);
}

void updateHeaterDiagnostics(uint32_t now) {
  bool anyRequested = finalDuty1 > 0.0f || finalDuty2 > 0.0f;
  bool anyActuallyOn = heater1Output || heater2Output;
  float averageTemp = meas.ntc1Valid && meas.ntc2Valid ? (meas.temp1C + meas.temp2C) * 0.5f : NAN;

  if (anyRequested && isfinite(averageTemp)) {
    if (heatDiagnosticStartedMs == 0) {
      heatDiagnosticStartedMs = now;
      heatActualOnMs = 0;
      heatLastAccountingMs = now;
      heatDiagnosticStartTempC = averageTemp;
      heatDiagnosticMask = (finalDuty1 > 0.0f ? 1 : 0) | (finalDuty2 > 0.0f ? 2 : 0);
    }
    uint32_t elapsed = now - heatLastAccountingMs;
    heatLastAccountingMs = now;
    if (anyActuallyOn) heatActualOnMs += elapsed;
    if (heatActualOnMs >= 20000 && averageTemp - heatDiagnosticStartTempC < 0.30f) {
      if (heatDiagnosticMask == 1) heater1IneffectiveCondition = true;
      else if (heatDiagnosticMask == 2) heater2IneffectiveCondition = true;
      else heatersIneffectiveCondition = true;
    }
    heatersOffSinceMs = 0;
  } else {
    heatDiagnosticStartedMs = 0;
    heatActualOnMs = 0;
    heatDiagnosticMask = 0;
    heatLastAccountingMs = now;
    if (heatersOffSinceMs == 0 && isfinite(averageTemp)) {
      heatersOffSinceMs = now;
      heatersOffStartTempC = averageTemp;
    }
    // Conservative inference only: both outputs have been commanded off for 20 s,
    // yet average pack temperature rose by more than 3 C.
    if (heatersOffSinceMs != 0 && now - heatersOffSinceMs >= 20000 &&
        isfinite(averageTemp) && averageTemp - heatersOffStartTempC >= 3.0f) {
      heaterStuckCondition = true;
    }
  }
}

// -----------------------------------------------------------------------------
// 7. Statistics and RAM telemetry (downloadable as CSV)
// -----------------------------------------------------------------------------

struct Statistics {
  float minCell1 = NAN, maxCell1 = NAN;
  float minCell2 = NAN, maxCell2 = NAN;
  float minTemp = NAN, maxTemp = NAN;
  float maxInputPowerW = 0.0f;
  double inputEnergyWh = 0.0;
  uint32_t heater1OnMs = 0;
  uint32_t heater2OnMs = 0;
  uint32_t solarProbeSuccess = 0;
  uint32_t solarProbeFailure = 0;
};

struct TelemetryRow {
  uint32_t seconds;
  float b1, b2, t1, t2, inputV, inputmA, soc;
  uint8_t solar, load, h1, h2;
};

Statistics stats;
constexpr size_t TELEMETRY_ROWS = 120;
TelemetryRow telemetry[TELEMETRY_ROWS];
size_t telemetryHead = 0;
size_t telemetryCount = 0;
uint32_t lastStatsMs = 0;
uint32_t lastTelemetryMs = 0;
SolarState previousSolarState = SolarState::OFF;

void updateMinMax(float value, float &minimum, float &maximum) {
  if (!isfinite(value)) return;
  if (!isfinite(minimum) || value < minimum) minimum = value;
  if (!isfinite(maximum) || value > maximum) maximum = value;
}

void updateStatistics(uint32_t now) {
  if (lastStatsMs == 0) lastStatsMs = now;
  uint32_t elapsed = now - lastStatsMs;
  if (elapsed < 1000) return;
  lastStatsMs = now;

  updateMinMax(meas.cell1V, stats.minCell1, stats.maxCell1);
  updateMinMax(meas.cell2V, stats.minCell2, stats.maxCell2);
  updateMinMax(meas.tempMinC, stats.minTemp, stats.maxTemp);
  updateMinMax(meas.tempMaxC, stats.minTemp, stats.maxTemp);
  if (isfinite(meas.inputPowerW) && meas.inputPowerW > 0.0f) {
    stats.maxInputPowerW = fmaxf(stats.maxInputPowerW, meas.inputPowerW);
    stats.inputEnergyWh += meas.inputPowerW * elapsed / 3600000.0;
  }
  if (heater1Output) stats.heater1OnMs += elapsed;
  if (heater2Output) stats.heater2OnMs += elapsed;

  if (previousSolarState == SolarState::PROBE && solarState == SolarState::RUN) stats.solarProbeSuccess++;
  if (previousSolarState == SolarState::PROBE && solarState == SolarState::WEAK) stats.solarProbeFailure++;
  previousSolarState = solarState;
}

void appendTelemetry(uint32_t now) {
  if (now - lastTelemetryMs < 5000) return;
  lastTelemetryMs = now;
  TelemetryRow &row = telemetry[telemetryHead];
  row.seconds = now / 1000;
  row.b1 = meas.cell1V; row.b2 = meas.cell2V;
  row.t1 = meas.temp1C; row.t2 = meas.temp2C;
  row.inputV = meas.inputV; row.inputmA = meas.inputCurrentmA; row.soc = meas.packSoc;
  row.solar = solarOutput; row.load = loadOutput; row.h1 = heater1Output; row.h2 = heater2Output;
  telemetryHead = (telemetryHead + 1) % TELEMETRY_ROWS;
  if (telemetryCount < TELEMETRY_ROWS) telemetryCount++;
}

// -----------------------------------------------------------------------------
// 8. Text/JSON helpers and serial command interface
// -----------------------------------------------------------------------------

String jsonEscape(const String &input) {
  String output;
  output.reserve(input.length() + 12);
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input.charAt(i);
    if (c == '\\') output += "\\\\";
    else if (c == '"') output += "\\\"";
    else if (c == '\n') output += "\\n";
    else if (c == '\t') output += "\\t";
    else if ((uint8_t)c >= 0x20) output += c;
  }
  return output;
}

String jsonFloat(float value, uint8_t decimals = 3) {
  return isfinite(value) ? String(value, static_cast<unsigned int>(decimals)) : "null";
}

const char *socConfidenceName() {
  if (!meas.cell1Valid || !meas.cell2Valid) return "UNAVAILABLE";
  if (solarOutput || heater1Output || heater2Output) return "LOW";
  if (loadOutput || wifiEnabled) return "MEDIUM";
  return "HIGH";
}

const char *limitingCellName() {
  if (!meas.cell1Valid || !meas.cell2Valid) return "UNKNOWN";
  return meas.soc1 <= meas.soc2 ? "B1" : "B2";
}

const char *chargingEstimateName() {
  if (!meas.inaValid || !solarOutput) return "INPUT_INACTIVE";
  if (meas.cell1V >= 4.15f && meas.cell2V >= 4.15f && meas.inputCurrentmA < 80.0f) return "NEAR_FULL_ESTIMATE";
  if (meas.inputCurrentmA > 50.0f) return "CHARGING_LIKELY";
  return "CHARGING_PATH_UNCERTAIN";
}

String activeFaultsText() {
  String text = "Active faults:\n";
  int count = 0;
  for (size_t i = 0; i < FAULT_COUNT; ++i) {
    if (!faults[i].active) continue;
    FaultCode code = (FaultCode)i;
    text += "- "; text += faultCodeName(code);
    text += " ["; text += severityName(faultSeverity(code)); text += "]\n  Location: ";
    text += faultLocation(code);
    text += "\n  Measured: "; text += jsonFloat(faults[i].measured, 3);
    text += "  Threshold: "; text += jsonFloat(faults[i].threshold, 3);
    text += "\n  Action: "; text += actionText(faultActions(code));
    text += faults[i].latched ? "\n  Status: active, latched\n" : "\n  Status: active\n";
    count++;
  }
  if (count == 0) text += "None";
  return text;
}

String statusText() {
  String text;
  text.reserve(1800);
  text += "System: "; text += systemStateName();
  text += " | Auto: "; text += autoControlEnabled ? "ON" : "OFF";
  text += " | Safe mode: "; text += safeMode ? "ON" : "OFF";
  text += "\nB1: "; text += jsonFloat(meas.cell1V, 3); text += " V (SOC "; text += jsonFloat(meas.soc1, 1); text += "%)";
  text += "\nB2: "; text += jsonFloat(meas.cell2V, 3); text += " V (SOC "; text += jsonFloat(meas.soc2, 1); text += "%)";
  text += "\nPack: "; text += jsonFloat(meas.packV, 3); text += " V | difference "; text += jsonFloat(meas.cellDiffV, 3); text += " V";
  text += "\nPack SOC: "; text += jsonFloat(meas.packSoc, 1); text += "% (voltage estimate, confidence ";
  text += socConfidenceName(); text += ", limiting cell "; text += limitingCellName(); text += ")";
  text += "\nNTC1/B1: "; text += jsonFloat(meas.temp1C, 2); text += " C | NTC2/B2: "; text += jsonFloat(meas.temp2C, 2); text += " C";
  text += "\nTemperature min/max/difference: "; text += jsonFloat(meas.tempMinC, 2); text += " / "; text += jsonFloat(meas.tempMaxC, 2); text += " / "; text += jsonFloat(meas.tempDiffC, 2); text += " C";
  text += "\nInput: "; text += jsonFloat(meas.inputV, 3); text += " V, "; text += jsonFloat(meas.inputCurrentmA, 1); text += " mA, "; text += jsonFloat(meas.inputPowerW, 3); text += " W";
  text += " | charge estimate: "; text += chargingEstimateName(); text += " (BQ hardware nominal 0.60 A)";
  text += "\nSolar: "; text += solarStateName(); text += solarOutput ? " (EN ON)" : " (EN OFF)";
  text += " | Balance: "; text += balanceStateName();
  text += "\nHeat demand: "; text += packHeatingDemand ? "YES" : "NO";
  text += " | H1 duty: "; text += String(finalDuty1 * 100.0f, 0); text += "%";
  text += " | H2 duty: "; text += String(finalDuty2 * 100.0f, 0); text += "%";
  text += " | cooldown: "; text += ((int32_t)(cooldownUntilMs - millis()) > 0) ? "YES" : "NO";
  text += "\nOutputs: Solar="; text += solarOutput ? "1" : "0"; text += " Load="; text += loadOutput ? "1" : "0";
  text += " H1="; text += heater1Output ? "1" : "0"; text += " H2="; text += heater2Output ? "1" : "0";
  text += "\n\n"; text += activeFaultsText();
  return text;
}

String menuText() {
  return
    "========== ELEC3117 BMS ==========\n"
    "r : live status and active faults\n"
    "f : detailed active faults\n"
    "a : enable automatic control\n"
    "q : disable automatic control\n"
    "s : request a protected Solar probe\n"
    "l : toggle Load request (still protected)\n"
    "1 : protected Heater1 10 s test\n"
    "2 : protected Heater2 10 s test\n"
    "b : protected both-heaters 10 s test\n"
    "0 : enter SAFE MODE, all outputs off\n"
    "x : leave SAFE MODE (fault blocks remain)\n"
    "c : clear inactive latched/diagnostic faults\n"
    "w : toggle Wi-Fi access point\n"
    "m : show menu\n"
    "==================================";
}

String executeCommand(char command, bool fromWeb) {
  uint32_t now = millis();
  switch (command) {
    case 'r': return statusText();
    case 'f': return activeFaultsText();
    case 'a':
      if (safeMode) return "Cannot enable automatic control while SAFE MODE is active.";
      autoControlEnabled = true;
      return "Automatic control enabled. Safety arbitration remains active.";
    case 'q':
      autoControlEnabled = false;
      packHeatingDemand = false;
      balanceTarget = 0;
      return "Automatic control disabled. Existing manual Load request is unchanged.";
    case 's':
      manualSolarUntilMs = now + MANUAL_SOLAR_MS;
      if (solarState == SolarState::OFF || solarState == SolarState::FAULT) enterSolarState(SolarState::WAIT_RECOVERY, now);
      return "Protected Solar probe requested. INA, cell voltage and fault checks still apply.";
    case 'l':
      loadUserRequest = !loadUserRequest;
      return String("Load request ") + (loadUserRequest ? "ON" : "OFF") + ". Final output depends on safety arbitration.";
    case '1':
      manualHeater1UntilMs = now + MANUAL_HEATER_MS;
      return "Protected Heater1/R26 test requested for up to 10 s at 50% duty.";
    case '2':
      manualHeater2UntilMs = now + MANUAL_HEATER_MS;
      return "Protected Heater2/R32 test requested for up to 10 s at 50% duty.";
    case 'b':
      manualHeater1UntilMs = manualHeater2UntilMs = now + MANUAL_HEATER_MS;
      return "Protected two-heater test requested for up to 10 s at 50% duty.";
    case '0':
      safeMode = true;
      autoControlEnabled = false;
      loadUserRequest = false;
      manualSolarUntilMs = manualHeater1UntilMs = manualHeater2UntilMs = 0;
      allPowerOutputsOff();
      return "SAFE MODE entered. All power requests removed and outputs OFF.";
    case 'x':
      safeMode = false;
      return "SAFE MODE cleared. Automatic control remains OFF; active faults still block outputs.";
    case 'c':
      watchdogResetDetected = false;
      heater1IneffectiveCondition = false;
      heater2IneffectiveCondition = false;
      heatersIneffectiveCondition = false;
      heaterStuckCondition = false;
      clearInactiveLatchedFaults(now);
      return "Inactive latched faults and diagnostic flags cleared. Present fault conditions remain active.";
    case 'w':
      if (wifiEnabled) {
        wifiOffAtMs = now + 800;
        return fromWeb ? "Wi-Fi will close after this response. Use Serial 'w' to restart it." : "Wi-Fi will turn OFF.";
      }
      return "WIFI_START_REQUEST";
    case 'm': return menuText();
    default: return "Unknown command. Use m for the menu.";
  }
}

String buildFaultsJson() {
  String json = "[";
  bool first = true;
  for (size_t i = 0; i < FAULT_COUNT; ++i) {
    if (!faults[i].active) continue;
    if (!first) json += ',';
    first = false;
    FaultCode code = (FaultCode)i;
    json += "{\"code\":\""; json += faultCodeName(code);
    json += "\",\"severity\":\""; json += severityName(faultSeverity(code));
    json += "\",\"location\":\""; json += jsonEscape(faultLocation(code));
    json += "\",\"measured\":"; json += jsonFloat(faults[i].measured, 3);
    json += ",\"threshold\":"; json += jsonFloat(faults[i].threshold, 3);
    json += ",\"action\":\""; json += jsonEscape(actionText(faultActions(code)));
    json += "\",\"latched\":"; json += faults[i].latched ? "true" : "false";
    json += "}";
  }
  json += "]";
  return json;
}

String buildStatusJson() {
  String json;
  json.reserve(5000);
  json += "{\"system\":{\"state\":\""; json += systemStateName();
  json += "\",\"auto\":"; json += autoControlEnabled ? "true" : "false";
  json += ",\"safe_mode\":"; json += safeMode ? "true" : "false";
  json += ",\"uptime_s\":"; json += millis() / 1000;
  json += "},\"battery\":{\"b1_v\":"; json += jsonFloat(meas.cell1V);
  json += ",\"b2_v\":"; json += jsonFloat(meas.cell2V);
  json += ",\"pack_v\":"; json += jsonFloat(meas.packV);
  json += ",\"difference_v\":"; json += jsonFloat(meas.cellDiffV);
  json += ",\"soc1\":"; json += jsonFloat(meas.soc1, 1);
  json += ",\"soc2\":"; json += jsonFloat(meas.soc2, 1);
  json += ",\"pack_soc\":"; json += jsonFloat(meas.packSoc, 1);
  json += ",\"soc_confidence\":\""; json += socConfidenceName();
  json += "\",\"limiting_cell\":\""; json += limitingCellName(); json += "\"";
  json += "},\"thermal\":{\"ntc1_v\":"; json += jsonFloat(meas.ntc1V, 4);
  json += ",\"ntc2_v\":"; json += jsonFloat(meas.ntc2V, 4);
  json += ",\"t1_c\":"; json += jsonFloat(meas.temp1C, 2);
  json += ",\"t2_c\":"; json += jsonFloat(meas.temp2C, 2);
  json += ",\"min_c\":"; json += jsonFloat(meas.tempMinC, 2);
  json += ",\"max_c\":"; json += jsonFloat(meas.tempMaxC, 2);
  json += ",\"difference_c\":"; json += jsonFloat(meas.tempDiffC, 2);
  json += ",\"heating_demand\":"; json += packHeatingDemand ? "true" : "false";
  json += "},\"input\":{\"valid\":"; json += meas.inaValid ? "true" : "false";
  json += ",\"voltage_v\":"; json += jsonFloat(meas.inputV);
  json += ",\"current_ma\":"; json += jsonFloat(meas.inputCurrentmA, 1);
  json += ",\"power_w\":"; json += jsonFloat(meas.inputPowerW);
  json += ",\"charging_estimate\":\""; json += chargingEstimateName();
  json += "\",\"nominal_charge_current_a\":"; json += jsonFloat(NOMINAL_CHARGE_CURRENT_A, 2);
  json += "},\"control\":{\"solar_state\":\""; json += solarStateName();
  json += "\",\"balance_state\":\""; json += balanceStateName();
  json += "\",\"h1_duty\":"; json += jsonFloat(finalDuty1 * 100.0f, 0);
  json += ",\"h2_duty\":"; json += jsonFloat(finalDuty2 * 100.0f, 0);
  json += ",\"cooldown\":"; json += ((int32_t)(cooldownUntilMs - millis()) > 0) ? "true" : "false";
  json += "},\"outputs\":{\"solar\":"; json += solarOutput ? "1" : "0";
  json += ",\"load\":"; json += loadOutput ? "1" : "0";
  json += ",\"heater1\":"; json += heater1Output ? "1" : "0";
  json += ",\"heater2\":"; json += heater2Output ? "1" : "0";
  json += "},\"settings\":{\"heater_start_c\":"; json += jsonFloat(settings.heaterStartC, 1);
  json += ",\"heater_stop_c\":"; json += jsonFloat(settings.heaterStopC, 1);
  json += ",\"mppt_vref\":"; json += jsonFloat(settings.mpptVref, 1);
  json += ",\"balance_start_v\":"; json += jsonFloat(settings.balanceStartV, 3);
  json += ",\"balance_stop_v\":"; json += jsonFloat(settings.balanceStopV, 3);
  json += ",\"probe_interval_s\":"; json += settings.probeIntervalS;
  json += ",\"probe_mode\":"; json += String((uint8_t)settings.probeMode);
  json += ",\"balance_mode\":"; json += String((uint8_t)settings.balanceMode);
  json += "},\"stats\":{\"energy_wh\":"; json += String(stats.inputEnergyWh, 4);
  json += ",\"max_input_power_w\":"; json += jsonFloat(stats.maxInputPowerW);
  json += ",\"h1_on_s\":"; json += stats.heater1OnMs / 1000;
  json += ",\"h2_on_s\":"; json += stats.heater2OnMs / 1000;
  json += "},\"faults\":"; json += buildFaultsJson();
  json += "}";
  return json;
}

// -----------------------------------------------------------------------------
// 9. Wi-Fi dashboard and API
// -----------------------------------------------------------------------------

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ELEC3117 BMS</title>
<style>
:root{color-scheme:dark;--bg:#0b1020;--card:#151d31;--line:#2a3856;--muted:#a9b5cc;--ok:#3fd18c;--warn:#f0b44c;--bad:#ff6673}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(145deg,#08101e,#121a2e);font:14px system-ui;color:#edf3ff}
main{max-width:1100px;margin:auto;padding:16px}h1{font-size:1.45rem;margin:0}h2{font-size:1rem;margin:0 0 10px}.sub{color:var(--muted);margin:5px 0 14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}.card{background:var(--card);border:1px solid var(--line);border-radius:13px;padding:13px;margin-bottom:10px}
.value{font-size:1.45rem;font-weight:700}.small{color:var(--muted);font-size:.82rem}.states{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:7px}
.state{border:1px solid var(--line);border-radius:9px;padding:8px;text-align:center}.on{background:#174d39;border-color:#2aa875}.off{color:var(--muted)}
.buttons{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:8px}button{padding:10px;border-radius:9px;border:1px solid #43577d;background:#223250;color:white;cursor:pointer}
button.warn{background:#6e4918}button.danger{background:#762936}input,select{width:100%;padding:8px;margin-top:4px;background:#0d1527;color:white;border:1px solid var(--line);border-radius:7px}
label{display:block;color:var(--muted)}table{width:100%;border-collapse:collapse;font-size:.82rem}th,td{text-align:left;padding:7px;border-bottom:1px solid var(--line);vertical-align:top}.critical{color:var(--bad)}.warning{color:var(--warn)}
pre{white-space:pre-wrap;background:#080d18;padding:10px;border-radius:8px;max-height:230px;overflow:auto;color:#cbd7ee}@media(max-width:600px){main{padding:10px}table{display:block;overflow-x:auto}}
</style></head><body><main>
<h1>ELEC3117 2S BMS Supervisor</h1><p class="sub"><span id="conn">Connecting...</span> · AP 192.168.4.1 · BQ hardware nominal charge limit 0.60 A · INA reads total input current</p>

<section class="grid">
 <div class="card"><h2>Battery 1</h2><div class="value" id="b1">-- V</div><div class="small" id="s1">SOC --%</div></div>
 <div class="card"><h2>Battery 2</h2><div class="value" id="b2">-- V</div><div class="small" id="s2">SOC --%</div></div>
 <div class="card"><h2>Pack</h2><div class="value" id="pack">-- V</div><div class="small" id="soc">SOC --% · ΔV --</div></div>
 <div class="card"><h2>Input / INA219</h2><div class="value" id="vin">-- V</div><div class="small" id="pin">-- mA · -- W</div></div>
 <div class="card"><h2>NTC1 / Battery 1</h2><div class="value" id="t1">-- °C</div><div class="small" id="ntc1">CH3 -- V</div></div>
 <div class="card"><h2>NTC2 / Battery 2</h2><div class="value" id="t2">-- °C</div><div class="small" id="ntc2">CH4 -- V</div></div>
</section>

<section class="card"><h2>Control state</h2><div class="states">
 <div class="state" id="sys">System<br><b>--</b></div><div class="state" id="solar">Solar<br><b>--</b></div>
 <div class="state" id="load">Load<br><b>--</b></div><div class="state" id="h1">Heater1<br><b>--</b></div>
 <div class="state" id="h2">Heater2<br><b>--</b></div><div class="state" id="bal">Balance<br><b>--</b></div>
 </div><p class="small" id="thermal">Temperature min/max/difference: --</p></section>

<section class="card"><h2>Protected controls</h2><div class="buttons">
 <button onclick="act('a')">Enable automatic</button><button onclick="act('q')">Disable automatic</button>
 <button class="warn" onclick="act('s')">Solar probe</button><button onclick="act('l')">Toggle load request</button>
 <button class="warn" onclick="act('1')">Test Heater1 10 s</button><button class="warn" onclick="act('2')">Test Heater2 10 s</button>
 <button class="warn" onclick="act('b')">Test both 10 s</button><button class="danger" onclick="act('0')">SAFE MODE / all off</button>
 <button onclick="act('x')">Leave safe mode</button><button onclick="act('c')">Clear inactive faults</button>
 <button onclick="location.href='/telemetry.csv'">Download CSV</button><button class="danger" onclick="act('w')">Wi-Fi off</button>
 </div><pre id="log">Dashboard ready.</pre></section>

<section class="card"><h2>Active faults</h2><div style="overflow:auto"><table><thead><tr><th>Code / severity</th><th>Location</th><th>Evidence</th><th>Automatic response</th></tr></thead><tbody id="faults"></tbody></table></div></section>

<section class="card"><h2>Editable settings</h2><div class="grid">
 <label>Heat start °C<input id="heatStart" type="number" step="0.5" min="-10" max="15"></label>
 <label>Heat stop °C<input id="heatStop" type="number" step="0.5" min="-8" max="25"></label>
 <label>MPPT Vref<input id="vref" type="number" step="0.1" min="6" max="14"></label>
 <label>Balance start ΔV<input id="balStart" type="number" step="0.005" min="0.03" max="0.15"></label>
 <label>Balance stop ΔV<input id="balStop" type="number" step="0.005" min="0.01" max="0.14"></label>
 <label>Probe interval s<input id="probeS" type="number" step="1" min="2" max="60"></label>
 <label>Probe mode<select id="probeMode"><option value="0">Voltage recovery</option><option value="1">Fixed interval</option></select></label>
 <label>Balance mode<select id="balMode"><option value="0">Off</option><option value="1">During heating only</option><option value="2">Independent only</option><option value="3">Both</option></select></label>
 </div><button style="margin-top:10px" onclick="saveConfig()">Validate and save settings</button>
 <p class="small">Absolute voltage, temperature, sensor-validity and 30 s runtime limits cannot be changed here.</p></section>
</main><script>
const $=id=>document.getElementById(id);let configLoaded=false,busy=false;
const fmt=(v,d,u='')=>Number.isFinite(v)?v.toFixed(d)+u:'--'+u;
function outputState(id,on,label){const e=$(id);e.className='state '+(on?'on':'off');e.querySelector('b').textContent=label??(on?'ON':'OFF')}
async function refresh(){if(busy)return;try{const r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw Error(r.status);const d=await r.json();
 $('conn').textContent='Connected · '+d.system.state+(d.system.auto?' · AUTO ON':' · AUTO OFF');
 $('b1').textContent=fmt(d.battery.b1_v,3,' V');$('b2').textContent=fmt(d.battery.b2_v,3,' V');$('pack').textContent=fmt(d.battery.pack_v,3,' V');
 $('s1').textContent='SOC '+fmt(d.battery.soc1,1,'%');$('s2').textContent='SOC '+fmt(d.battery.soc2,1,'%');$('soc').textContent='SOC '+fmt(d.battery.pack_soc,1,'%')+' · '+d.battery.soc_confidence+' · limiting '+d.battery.limiting_cell+' · ΔV '+fmt(d.battery.difference_v,3,' V');
 $('vin').textContent=fmt(d.input.voltage_v,3,' V');$('pin').textContent=fmt(d.input.current_ma,1,' mA')+' · '+fmt(d.input.power_w,3,' W')+' · '+d.input.charging_estimate;
 $('t1').textContent=fmt(d.thermal.t1_c,2,' °C');$('t2').textContent=fmt(d.thermal.t2_c,2,' °C');$('ntc1').textContent='CH3 '+fmt(d.thermal.ntc1_v,4,' V');$('ntc2').textContent='CH4 '+fmt(d.thermal.ntc2_v,4,' V');
 outputState('sys',d.system.state==='NORMAL',d.system.state);outputState('solar',d.outputs.solar,d.control.solar_state);outputState('load',d.outputs.load);outputState('h1',d.outputs.heater1,fmt(d.control.h1_duty,0,'%'));outputState('h2',d.outputs.heater2,fmt(d.control.h2_duty,0,'%'));outputState('bal',d.control.balance_state!=='OFF',d.control.balance_state);
 $('thermal').textContent='Temperature min/max/difference: '+fmt(d.thermal.min_c,2)+' / '+fmt(d.thermal.max_c,2)+' / '+fmt(d.thermal.difference_c,2)+' °C'+(d.control.cooldown?' · resistor cooldown active':'');
 const body=$('faults');body.innerHTML='';if(!d.faults.length)body.innerHTML='<tr><td colspan="4">No active fault</td></tr>';d.faults.forEach(f=>{const tr=document.createElement('tr');tr.innerHTML=`<td class="${f.severity.toLowerCase()}"><b>${f.code}</b><br>${f.severity}${f.latched?' · latched':''}</td><td>${f.location}</td><td>${fmt(f.measured,3)} / limit ${fmt(f.threshold,3)}</td><td>${f.action}</td>`;body.appendChild(tr)});
 if(!configLoaded){$('heatStart').value=d.settings.heater_start_c;$('heatStop').value=d.settings.heater_stop_c;$('vref').value=d.settings.mppt_vref;$('balStart').value=d.settings.balance_start_v;$('balStop').value=d.settings.balance_stop_v;$('probeS').value=d.settings.probe_interval_s;$('probeMode').value=d.settings.probe_mode;$('balMode').value=d.settings.balance_mode;configLoaded=true}
 }catch(e){$('conn').textContent='Disconnected';}}
async function act(c){if(c==='0'&&!confirm('Enter SAFE MODE and turn all power outputs off?'))return;busy=true;try{const r=await fetch('/api/action?c='+encodeURIComponent(c),{method:'POST'});const d=await r.json();$('log').textContent=d.message}catch(e){$('log').textContent=c==='w'?'Wi-Fi closed. Use Serial w to restart.':'Command failed: '+e.message}finally{busy=false;if(c!=='w')refresh()}}
async function saveConfig(){const p=new URLSearchParams({heatStart:$('heatStart').value,heatStop:$('heatStop').value,vref:$('vref').value,balStart:$('balStart').value,balStop:$('balStop').value,probeS:$('probeS').value,probeMode:$('probeMode').value,balMode:$('balMode').value});
 try{const r=await fetch('/api/config?'+p,{method:'POST'});const d=await r.json();$('log').textContent=d.message;configLoaded=false;refresh()}catch(e){$('log').textContent='Save failed: '+e.message}}
refresh();setInterval(refresh,1500);
</script></body></html>
)HTML";

void sendNoCache() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
}

void handleRoot() {
  sendNoCache();
  server.send_P(200, "text/html; charset=utf-8", CONTROL_PAGE);
}

void handleStatus() {
  sendNoCache();
  server.send(200, "application/json", buildStatusJson());
}

void handleAction() {
  if (!server.hasArg("c") || server.arg("c").length() != 1) {
    server.send(400, "application/json", "{\"ok\":false,\"message\":\"Missing command\"}");
    return;
  }
  char command = server.arg("c").charAt(0);
  String message = executeCommand(command, true);
  Serial.printf("\n[WEB %c]\n%s\n", command, message.c_str());
  String response = "{\"ok\":true,\"message\":\"" + jsonEscape(message) + "\"}";
  sendNoCache();
  server.send(200, "application/json", response);
}

bool parseFloatArg(const char *name, float &destination) {
  if (!server.hasArg(name)) return false;
  destination = server.arg(name).toFloat();
  return true;
}

void handleConfig() {
  UserSettings candidate = settings;
  parseFloatArg("heatStart", candidate.heaterStartC);
  parseFloatArg("heatStop", candidate.heaterStopC);
  parseFloatArg("vref", candidate.mpptVref);
  parseFloatArg("balStart", candidate.balanceStartV);
  parseFloatArg("balStop", candidate.balanceStopV);
  if (server.hasArg("probeS")) candidate.probeIntervalS = server.arg("probeS").toInt();
  if (server.hasArg("probeMode")) candidate.probeMode = (ProbeMode)server.arg("probeMode").toInt();
  if (server.hasArg("balMode")) candidate.balanceMode = (BalanceMode)server.arg("balMode").toInt();

  bool valid = candidate.heaterStartC >= -10.0f && candidate.heaterStartC <= 15.0f &&
    candidate.heaterStopC >= candidate.heaterStartC + 2.0f && candidate.heaterStopC <= 25.0f &&
    candidate.mpptVref >= 6.0f && candidate.mpptVref <= 14.0f &&
    candidate.balanceStartV >= 0.030f && candidate.balanceStartV <= 0.150f &&
    candidate.balanceStopV >= 0.010f && candidate.balanceStopV <= candidate.balanceStartV - 0.010f &&
    candidate.probeIntervalS >= 2 && candidate.probeIntervalS <= 60 &&
    (uint8_t)candidate.probeMode <= 1 && (uint8_t)candidate.balanceMode <= 3;
  if (!valid) {
    server.send(400, "application/json", "{\"ok\":false,\"message\":\"Invalid setting range or hysteresis. Nothing was saved.\"}");
    return;
  }
  settings = candidate;
  saveSettings();
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Settings validated and saved to NVS.\"}");
}

void handleTelemetryCsv() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=ELEC3117_BMS_telemetry.csv");
  server.send(200, "text/csv", "seconds,b1_v,b2_v,temp1_c,temp2_c,input_v,input_ma,soc_percent,solar,load,heater1,heater2\n");
  size_t start = (telemetryHead + TELEMETRY_ROWS - telemetryCount) % TELEMETRY_ROWS;
  for (size_t i = 0; i < telemetryCount; ++i) {
    const TelemetryRow &row = telemetry[(start + i) % TELEMETRY_ROWS];
    String line = String(row.seconds) + "," + jsonFloat(row.b1) + "," + jsonFloat(row.b2) + "," +
      jsonFloat(row.t1, 2) + "," + jsonFloat(row.t2, 2) + "," + jsonFloat(row.inputV) + "," +
      jsonFloat(row.inputmA, 1) + "," + jsonFloat(row.soc, 1) + "," + String(row.solar) + "," +
      String(row.load) + "," + String(row.h1) + "," + String(row.h2) + "\n";
    server.sendContent(line);
  }
  server.sendContent("");
}

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/action", HTTP_POST, handleAction);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/telemetry.csv", HTTP_GET, handleTelemetryCsv);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
}

bool startWiFi() {
  if (wifiEnabled) return true;
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
    WiFi.mode(WIFI_OFF);
    return false;
  }
  server.begin();
  wifiEnabled = true;
  wifiOffAtMs = 0;
  Serial.printf("Wi-Fi ON: %s / %s / http://%s\n", WIFI_SSID, WIFI_PASSWORD, WiFi.softAPIP().toString().c_str());
  return true;
}

void stopWiFi() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  wifiOffAtMs = 0;
  Serial.println("Wi-Fi OFF. Enter w in Serial Monitor to restart it.");
}

// -----------------------------------------------------------------------------
// 10. Setup and cooperative main loop
// -----------------------------------------------------------------------------

uint32_t lastControlMs = 0;
uint32_t lastFaultMs = 0;
uint32_t lastLedMs = 0;

void setup() {
  pinMode(PIN_SOLAR_EN, OUTPUT);
  pinMode(PIN_LOAD_EN, OUTPUT);
  pinMode(PIN_HEATER1_EN, OUTPUT);
  pinMode(PIN_HEATER2_EN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  allPowerOutputsOff();
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  delay(300);
  loadSettings();
  ADCBus.begin(PIN_ADC_SDA, PIN_ADC_SCL, 100000);
  CurrentBus.begin(PIN_CURRENT_SDA, PIN_CURRENT_SCL, 100000);
  configureWebServer();

  esp_reset_reason_t resetReason = esp_reset_reason();
  watchdogResetDetected = resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_INT_WDT || resetReason == ESP_RST_WDT;
  dutyWindowStartMs = millis();

  Serial.println("\nELEC3117 BMS supervisor started. All power outputs are OFF.");
  Serial.println("Automatic control is OFF after boot; use 'a' only after checking live data.");
  Serial.println(menuText());
  if (WIFI_START_ON_BOOT) startWiFi();
}

void loop() {
  uint32_t now = millis();
  serviceMCP3424(now);
  serviceINA219(now);
  deriveMeasurements(now);

  if (now - lastFaultMs >= 200) {
    lastFaultMs = now;
    updateFaults(now);
    updateSystemState(now);
  }
  if (now - lastControlMs >= 100) {
    lastControlMs = now;
    updateSolarController(now);
    calculateHeaterAndBalanceRequests(now);
  }

  arbitrateOutputs(now);
  updateHeaterDiagnostics(now);
  updateStatistics(now);
  appendTelemetry(now);

  if (wifiEnabled) server.handleClient();
  if (wifiOffAtMs != 0 && (int32_t)(now - wifiOffAtMs) >= 0) stopWiFi();

  while (Serial.available()) {
    char command = Serial.read();
    if (command == '\n' || command == '\r') continue;
    String response = executeCommand(command, false);
    if (response == "WIFI_START_REQUEST") response = startWiFi() ? "Wi-Fi hotspot is ON." : "Wi-Fi failed to start.";
    Serial.println(); Serial.println(response);
  }

  if (now - lastLedMs >= 500) {
    lastLedMs = now;
    if (anyActiveCriticalFault()) digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    else digitalWrite(PIN_LED, autoControlEnabled ? HIGH : LOW);
  }
  delay(2);
}