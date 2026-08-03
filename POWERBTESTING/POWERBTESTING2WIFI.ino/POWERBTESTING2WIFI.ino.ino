#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdio.h>

// ======================================================
// GPIO assignment
// These are ESP32 GPIO numbers, not module physical pin numbers.
// ======================================================
//热点：ELEC3117-BMS
//密码：elec3117
//面板：http://192.168.4.1
// MCP3424 I2C
const int PIN_ADC_SDA = 32;
const int PIN_ADC_SCL = 33;

// INA219 I2C
const int PIN_CURRENT_SDA = 16;
const int PIN_CURRENT_SCL = 17;

// Control outputs
const int PIN_SOLAR_EN = 25;
const int PIN_LOAD_EN  = 26;
const int PIN_HB_EN    = 27;
const int PIN_LB_EN    = 13;

// Onboard LED: GPIO18 -> 2.2k ohm -> LED -> GND
const int PIN_LED = 18;

// CH1 pack-voltage divider:
// 75k/25k -> 4.0; if 75k was changed to 82k, use 4.28.
const float PACK_DIVIDER = 4.0;

// INA219 shunt resistor from the schematic
const float INA_SHUNT_RESISTANCE_OHM = 0.05;

// ======================================================
// NTC configuration: B57540G1103F000
// 3V3MCU -> NTC -> CHx+ -> 3.3k ohm -> GND, CHx- -> GND
// 10k ohm at 25 degC, B25/100 = 3492 K
// ======================================================

const float NTC_SUPPLY_VOLTAGE = 3.30;
const float NTC_FIXED_RESISTOR = 3300.0;
const float NTC_R25            = 10000.0;
const float NTC_BETA           = 3492.0;
const float NTC_T25_K          = 298.15;

// ======================================================
// Wi-Fi access point
// Phone: connect to this SSID, then open http://192.168.4.1
// Password must contain at least 8 characters.
// ======================================================

const char *WIFI_AP_SSID     = "ELEC3117-BMS";
const char *WIFI_AP_PASSWORD = "12345678";
const bool WIFI_START_ON_BOOT = true;

// Two independent ESP32 I2C controllers
TwoWire ADCBus(0);
TwoWire CurrentBus(1);

WebServer server(80);
bool wifiEnabled = false;
bool webRoutesConfigured = false;
unsigned long wifiOffAtMs = 0;

// ======================================================
// Reading structures
// ======================================================

struct INAReading {
  bool found;
  bool valid;
  uint8_t address;
  float busVoltageV;
  float shuntVoltagemV;
  float currentmA;
};

struct MCPReading {
  bool found;
  uint8_t address;
  bool valid[4];
  float voltage[4];
};

struct NTCReading {
  bool valid;
  float voltage;
  float resistanceOhm;
  float temperatureC;
  String status;
};

// ======================================================
// Safe output control
// ======================================================

void allPowerOutputsOff() {
  digitalWrite(PIN_SOLAR_EN, LOW);
  digitalWrite(PIN_LOAD_EN, LOW);
  digitalWrite(PIN_HB_EN, LOW);
  digitalWrite(PIN_LB_EN, LOW);
}

void blinkLED(int times, int intervalMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(intervalMs);
    digitalWrite(PIN_LED, LOW);
    delay(intervalMs);
  }
}

String pulseOutput(
  const char *name,
  int pin,
  unsigned long durationMs
) {
  digitalWrite(pin, HIGH);
  digitalWrite(PIN_LED, HIGH);

  delay(durationMs);

  digitalWrite(pin, LOW);
  digitalWrite(PIN_LED, LOW);

  String result = String(name);
  result += " pulsed HIGH for ";
  result += String(durationMs);
  result += " ms, then returned LOW.";
  return result;
}

// ======================================================
// General helpers
// ======================================================

bool timeReached(unsigned long target) {
  return target != 0 &&
         (long)(millis() - target) >= 0;
}

String jsonEscape(const String &input) {
  String output;
  output.reserve(input.length() + 16);

  for (size_t i = 0; i < input.length(); i++) {
    char c = input.charAt(i);

    switch (c) {
      case '\\': output += "\\\\"; break;
      case '"':  output += "\\\""; break;
      case '\n': output += "\\n"; break;
      case '\r': break;
      case '\t': output += "\\t"; break;
      default:
        if ((uint8_t)c >= 0x20) {
          output += c;
        }
        break;
    }
  }

  return output;
}

String jsonNumber(float value, int decimals) {
  if (!isfinite(value)) {
    return "null";
  }
  return String(value, decimals);
}

String gpioStateText() {
  String result;
  result.reserve(180);
  result += "SolarSideEN GPIO25: ";
  result += String(digitalRead(PIN_SOLAR_EN));
  result += "\nLoad EN GPIO26:     ";
  result += String(digitalRead(PIN_LOAD_EN));
  result += "\nHB EN GPIO27:       ";
  result += String(digitalRead(PIN_HB_EN));
  result += "\nLB EN GPIO13:       ";
  result += String(digitalRead(PIN_LB_EN));
  result += "\nLED GPIO18:         ";
  result += String(digitalRead(PIN_LED));
  return result;
}

// ======================================================
// I2C scan
// ======================================================

String scanI2CText(TwoWire &bus, const char *busName) {
  String result = String("Scanning ") + busName + "\n";
  int found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();

    if (error == 0) {
      char line[40];
      snprintf(line, sizeof(line), "  Device found at 0x%02X\n", address);
      result += line;
      found++;
    }
  }

  if (found == 0) {
    result += "  No I2C device found.\n";
  }

  return result;
}

String scanBothI2CBusesText() {
  String result;
  result.reserve(300);
  result += scanI2CText(
    ADCBus,
    "ADC bus: GPIO32 SDA / GPIO33 SCL"
  );
  result += "\n";
  result += scanI2CText(
    CurrentBus,
    "Current bus: GPIO16 SDA / GPIO17 SCL"
  );
  return result;
}

// ======================================================
// INA219
// ======================================================

uint8_t findINA219() {
  for (uint8_t address = 0x40; address <= 0x4F; address++) {
    CurrentBus.beginTransmission(address);

    if (CurrentBus.endTransmission() == 0) {
      return address;
    }
  }

  return 0;
}

bool readRegister16(
  TwoWire &bus,
  uint8_t deviceAddress,
  uint8_t registerAddress,
  uint16_t &value
) {
  bus.beginTransmission(deviceAddress);
  bus.write(registerAddress);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  if (bus.requestFrom(deviceAddress, (uint8_t)2) != 2) {
    return false;
  }

  value = ((uint16_t)bus.read() << 8) | bus.read();
  return true;
}

INAReading getINA219Reading() {
  INAReading reading = {};
  reading.address = findINA219();
  reading.found = reading.address != 0;

  if (!reading.found) {
    return reading;
  }

  uint16_t shuntRegister = 0;
  uint16_t busRegister = 0;

  bool shuntOK = readRegister16(
    CurrentBus,
    reading.address,
    0x01,
    shuntRegister
  );

  bool busOK = readRegister16(
    CurrentBus,
    reading.address,
    0x02,
    busRegister
  );

  if (!shuntOK || !busOK) {
    return reading;
  }

  int16_t signedShunt = (int16_t)shuntRegister;

  // Shunt voltage LSB = 10 uV = 0.01 mV
  reading.shuntVoltagemV = signedShunt * 0.01;

  // mV / ohm = mA
  reading.currentmA =
    reading.shuntVoltagemV /
    INA_SHUNT_RESISTANCE_OHM;

  // Bus voltage LSB = 4 mV
  reading.busVoltageV = (busRegister >> 3) * 0.004;
  reading.valid = true;
  return reading;
}

String inaReadingText(const INAReading &reading) {
  String result = "----- INA219 -----\n";

  if (!reading.found) {
    result += "INA219 not found.";
    return result;
  }

  char addressLine[40];
  snprintf(
    addressLine,
    sizeof(addressLine),
    "INA219 address: 0x%02X\n",
    reading.address
  );
  result += addressLine;

  if (!reading.valid) {
    result += "INA219 register read failed.";
    return result;
  }

  result += "Bus voltage: ";
  result += String(reading.busVoltageV, 3);
  result += " V\nShunt voltage: ";
  result += String(reading.shuntVoltagemV, 3);
  result += " mV\nEstimated current: ";
  result += String(reading.currentmA, 2);
  result += " mA";
  return result;
}

// ======================================================
// MCP3424 and NTC
// ======================================================

uint8_t findMCP3424() {
  for (uint8_t address = 0x68; address <= 0x6F; address++) {
    ADCBus.beginTransmission(address);

    if (ADCBus.endTransmission() == 0) {
      return address;
    }
  }

  return 0;
}

bool readMCP3424Channel(
  uint8_t address,
  uint8_t channel,
  float &voltage
) {
  if (channel < 1 || channel > 4) {
    return false;
  }

  /*
    bit 7    RDY: start conversion
    bit 6:5  channel
    bit 4    continuous conversion
    bit 3:2  16-bit mode
    bit 1:0  PGA gain x1
  */

  uint8_t config =
    0x80 |
    ((channel - 1) << 5) |
    0x10 |
    0x08;

  ADCBus.beginTransmission(address);
  ADCBus.write(config);

  if (ADCBus.endTransmission() != 0) {
    return false;
  }

  delay(80);

  for (int attempt = 0; attempt < 5; attempt++) {
    if (ADCBus.requestFrom(address, (uint8_t)3) != 3) {
      return false;
    }

    uint8_t msb = ADCBus.read();
    uint8_t lsb = ADCBus.read();
    uint8_t returnedConfig = ADCBus.read();

    if ((returnedConfig & 0x80) == 0) {
      int16_t rawValue =
        (int16_t)(((uint16_t)msb << 8) | lsb);

      // 16-bit, PGA x1: 1 LSB = 62.5 uV
      voltage = rawValue * 0.0000625;
      return true;
    }

    delay(25);
  }

  return false;
}

MCPReading getMCP3424Reading() {
  MCPReading reading = {};
  reading.address = findMCP3424();
  reading.found = reading.address != 0;

  if (!reading.found) {
    return reading;
  }

  for (int channel = 1; channel <= 4; channel++) {
    reading.valid[channel - 1] = readMCP3424Channel(
      reading.address,
      channel,
      reading.voltage[channel - 1]
    );
  }

  return reading;
}

bool ntcVoltageToTemperature(
  float voltage,
  float &resistance,
  float &temperatureC
) {
  if (!isfinite(voltage) ||
      voltage <= 0.01 ||
      voltage >= NTC_SUPPLY_VOLTAGE - 0.01) {
    return false;
  }

  // Vout = Vcc * Rfixed / (Rntc + Rfixed)
  resistance =
    NTC_FIXED_RESISTOR *
    (NTC_SUPPLY_VOLTAGE / voltage - 1.0);

  if (!isfinite(resistance) || resistance <= 0.0) {
    return false;
  }

  float inverseTemperature =
    (1.0 / NTC_T25_K) +
    (log(resistance / NTC_R25) / NTC_BETA);

  if (!isfinite(inverseTemperature) ||
      inverseTemperature <= 0.0) {
    return false;
  }

  temperatureC = (1.0 / inverseTemperature) - 273.15;
  return isfinite(temperatureC);
}

NTCReading ntcFromVoltage(bool adcValid, float voltage) {
  NTCReading reading = {};
  reading.voltage = voltage;

  if (!adcValid) {
    reading.status = "ADC read failed";
    return reading;
  }

  if (voltage >= 2.04) {
    reading.status =
      "ADC near full scale; check NTC short/high temperature";
    return reading;
  }

  reading.valid = ntcVoltageToTemperature(
    voltage,
    reading.resistanceOhm,
    reading.temperatureC
  );

  if (!reading.valid) {
    reading.status = "Invalid reading; check NTC wiring";
    return reading;
  }

  reading.status = "OK";
  return reading;
}

String mcpReadingText(const MCPReading &reading) {
  String result = "----- MCP3424 -----\n";

  if (!reading.found) {
    result += "MCP3424 not found.";
    return result;
  }

  char addressLine[40];
  snprintf(
    addressLine,
    sizeof(addressLine),
    "MCP3424 address: 0x%02X\n",
    reading.address
  );
  result += addressLine;

  for (int channel = 0; channel < 4; channel++) {
    result += "CH";
    result += String(channel + 1);
    result += ": ";

    if (reading.valid[channel]) {
      result += String(reading.voltage[channel], 5);
      result += " V";
    } else {
      result += "read failed";
    }
    result += "\n";
  }

  if (reading.valid[0]) {
    result += "\nCH1 estimated input: ";
    result += String(reading.voltage[0] * PACK_DIVIDER, 3);
    result += " V";
  }

  return result;
}

String oneNTCText(
  const char *name,
  uint8_t channel,
  const NTCReading &reading
) {
  String result = String(name);
  result += " (CH";
  result += String(channel);
  result += ")\n  Voltage:     ";
  result += String(reading.voltage, 5);
  result += " V\n";

  if (!reading.valid) {
    result += "  Status:      ";
    result += reading.status;
    return result;
  }

  result += "  Resistance:  ";
  result += String(reading.resistanceOhm / 1000.0, 3);
  result += " kOhm\n  Temperature: ";
  result += String(reading.temperatureC, 2);
  result += " degC";
  return result;
}

String temperatureTextFromMCP(const MCPReading &mcp) {
  String result = "----- NTC TEMPERATURES -----\n";

  if (!mcp.found) {
    result += "MCP3424 not found.";
    return result;
  }

  NTCReading th1 = ntcFromVoltage(
    mcp.valid[2],
    mcp.voltage[2]
  );
  NTCReading th2 = ntcFromVoltage(
    mcp.valid[3],
    mcp.voltage[3]
  );

  result += oneNTCText("TH1", 3, th1);
  result += "\n";
  result += oneNTCText("TH2", 4, th2);
  result +=
    "\n\nCalibration: set NTC_SUPPLY_VOLTAGE to measured 3V3MCU.";
  return result;
}

// Five-sample averaging for the serial/menu temperature command.
String readAveragedNTCTemperaturesText() {
  String result = "----- NTC TEMPERATURES -----\n";
  uint8_t address = findMCP3424();

  if (address == 0) {
    result += "MCP3424 not found.";
    return result;
  }

  const int SAMPLE_COUNT = 5;
  float sum[2] = {0.0, 0.0};
  bool valid[2] = {true, true};

  for (int sensor = 0; sensor < 2; sensor++) {
    uint8_t channel = sensor + 3;

    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
      float voltage = 0.0;

      if (!readMCP3424Channel(address, channel, voltage)) {
        valid[sensor] = false;
        break;
      }
      sum[sensor] += voltage;
    }
  }

  NTCReading th1 = ntcFromVoltage(
    valid[0],
    valid[0] ? sum[0] / SAMPLE_COUNT : 0.0
  );
  NTCReading th2 = ntcFromVoltage(
    valid[1],
    valid[1] ? sum[1] / SAMPLE_COUNT : 0.0
  );

  result += oneNTCText("TH1", 3, th1);
  result += "\n";
  result += oneNTCText("TH2", 4, th2);
  result +=
    "\n\nCalibration: set NTC_SUPPLY_VOLTAGE to measured 3V3MCU.";
  return result;
}

// ======================================================
// Shared command system for Serial and web panel
// ======================================================

String menuText() {
  String result;
  result.reserve(650);
  result += "========== MCU BOARD TEST ==========\n";
  result += "i : scan both I2C buses\n";
  result += "r : read INA219 and MCP3424\n";
  result += "t : read TH1/TH2 NTC temperatures\n";
  result += "d : show output GPIO states\n";
  result += "e : blink onboard LED 3 times\n";
  result += "s : pulse SolarSideEN for 2 seconds\n";
  result += "l : pulse Load Switch for 2 seconds\n";
  result += "h : pulse HB EN for 0.5 second\n";
  result += "b : pulse LB EN for 0.5 second\n";
  result += "0 : immediately turn power outputs OFF\n";
  result += "w : turn Wi-Fi hotspot ON/OFF\n";
  result += "m : show this menu\n";
  result += "====================================";
  return result;
}

void printMenu() {
  Serial.println();
  Serial.println(menuText());
}

void scheduleWiFiOff() {
  // Delay shutdown so an HTTP response can reach the phone.
  wifiOffAtMs = millis() + 800;
}

String executeCommand(char command, bool fromWeb) {
  switch (command) {
    case 'i':
      return scanBothI2CBusesText();

    case 'r': {
      INAReading ina = getINA219Reading();
      MCPReading mcp = getMCP3424Reading();
      return inaReadingText(ina) + "\n\n" + mcpReadingText(mcp);
    }

    case 't':
      return readAveragedNTCTemperaturesText();

    case 'd':
      return String("----- GPIO STATES -----\n") + gpioStateText();

    case 'e':
      blinkLED(3, 300);
      return "Onboard LED blinked 3 times.";

    case 's':
      return pulseOutput(
        "SolarSideEN GPIO25",
        PIN_SOLAR_EN,
        2000
      );

    case 'l':
      return pulseOutput(
        "Load Switch GPIO26",
        PIN_LOAD_EN,
        2000
      );

    case 'h':
      return pulseOutput(
        "HB EN GPIO27",
        PIN_HB_EN,
        500
      );

    case 'b':
      return pulseOutput(
        "LB EN GPIO13",
        PIN_LB_EN,
        500
      );

    case '0':
      allPowerOutputsOff();
      return "All power-control outputs are LOW.";

    case 'w':
      if (wifiEnabled) {
        scheduleWiFiOff();
        if (fromWeb) {
          return
            "Wi-Fi hotspot is closing. Use Serial command w to turn it on again.";
        }
        return "Wi-Fi hotspot will turn OFF.";
      }
      // startWiFi() is called by the Serial command handler below.
      return "WIFI_START_REQUEST";

    case 'm':
      return menuText();

    default:
      return "Unknown command. Enter m for menu.";
  }
}

// ======================================================
// Web page
// ======================================================

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ELEC3117 BMS Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b1020;
      --card: #151d31;
      --line: #27334f;
      --text: #eef3ff;
      --muted: #9eabc6;
      --blue: #4e8cff;
      --green: #42d392;
      --red: #ff5f6d;
      --amber: #f4b942;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: linear-gradient(145deg, #09101f, #11182b);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main { max-width: 980px; margin: auto; padding: 18px; }
    header {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: flex-start;
      margin-bottom: 16px;
    }
    h1 { font-size: 1.45rem; margin: 0 0 5px; }
    h2 { font-size: 1rem; margin: 0 0 12px; }
    p { margin: 3px 0; color: var(--muted); }
    .dot {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--amber);
      margin-right: 7px;
    }
    .dot.ok { background: var(--green); }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(205px, 1fr));
      gap: 12px;
      margin-bottom: 12px;
    }
    .card {
      background: rgba(21, 29, 49, 0.95);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 14px;
      box-shadow: 0 10px 28px rgba(0,0,0,.18);
    }
    .value { font-size: 1.55rem; font-weight: 700; margin: 5px 0; }
    .small { font-size: .84rem; color: var(--muted); }
    .states {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 7px;
    }
    .state {
      padding: 8px 4px;
      border-radius: 9px;
      text-align: center;
      background: #0d1425;
      border: 1px solid var(--line);
      color: var(--muted);
      font-size: .78rem;
    }
    .state.on { color: white; background: #18583f; border-color: #2ba874; }
    .buttons {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(145px, 1fr));
      gap: 9px;
    }
    button {
      border: 1px solid #3b4a6b;
      border-radius: 10px;
      padding: 11px 9px;
      background: #1d2944;
      color: white;
      font: inherit;
      cursor: pointer;
      touch-action: manipulation;
    }
    button:active { transform: translateY(1px); }
    button.primary { background: #245bb6; border-color: #4e8cff; }
    button.warn { background: #714814; border-color: #bd7c26; }
    button.danger { background: #7b2630; border-color: #dc4e5c; }
    button:disabled { opacity: .48; cursor: wait; }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      min-height: 80px;
      max-height: 290px;
      overflow-y: auto;
      background: #080d18;
      border-radius: 10px;
      padding: 11px;
      color: #c8d5ef;
      font-size: .79rem;
    }
    @media (max-width: 560px) {
      header { display: block; }
      .states { grid-template-columns: repeat(3, 1fr); }
      main { padding: 12px; }
    }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>ELEC3117 BMS Monitor & Control</h1>
      <p><span id="dot" class="dot"></span><span id="connection">Connecting...</span></p>
    </div>
    <p id="network">AP: ELEC3117-BMS · 192.168.4.1</p>
  </header>

  <section class="grid">
    <div class="card">
      <h2>Pack voltage · MCP3424 CH1</h2>
      <div class="value" id="pack">-- V</div>
      <div class="small" id="ch1">ADC: -- V</div>
    </div>
    <div class="card">
      <h2>INA219 current</h2>
      <div class="value" id="current">-- mA</div>
      <div class="small" id="inaBus">Bus: -- V · Shunt: -- mV</div>
    </div>
    <div class="card">
      <h2>TH1 · MCP3424 CH3</h2>
      <div class="value" id="th1">-- °C</div>
      <div class="small" id="th1sub">Waiting for ADC</div>
    </div>
    <div class="card">
      <h2>TH2 · MCP3424 CH4</h2>
      <div class="value" id="th2">-- °C</div>
      <div class="small" id="th2sub">Waiting for ADC</div>
    </div>
  </section>

  <section class="card" style="margin-bottom:12px">
    <h2>Output states</h2>
    <div class="states">
      <div class="state" id="stSolar">Solar<br><b>--</b></div>
      <div class="state" id="stLoad">Load<br><b>--</b></div>
      <div class="state" id="stHB">HB<br><b>--</b></div>
      <div class="state" id="stLB">LB<br><b>--</b></div>
      <div class="state" id="stLED">LED<br><b>--</b></div>
    </div>
  </section>

  <section class="card" style="margin-bottom:12px">
    <h2>Menu-equivalent controls</h2>
    <div class="buttons">
      <button class="primary" onclick="run('r')">r · Read all sensors</button>
      <button class="primary" onclick="run('t')">t · Read temperatures</button>
      <button onclick="run('i')">i · Scan I²C</button>
      <button onclick="run('d')">d · GPIO states</button>
      <button onclick="run('e')">e · Blink LED</button>
      <button class="warn" onclick="confirmRun('s','Pulse SolarSideEN for 2 seconds?')">s · Solar pulse</button>
      <button class="warn" onclick="confirmRun('l','Pulse Load Switch for 2 seconds?')">l · Load pulse</button>
      <button class="warn" onclick="confirmRun('h','Pulse HB EN for 0.5 second?')">h · HB pulse</button>
      <button class="warn" onclick="confirmRun('b','Pulse LB EN for 0.5 second?')">b · LB pulse</button>
      <button class="danger" onclick="run('0')">0 · ALL OUTPUTS OFF</button>
      <button onclick="run('m')">m · Show menu</button>
      <button class="danger" onclick="confirmRun('w','Close the Wi-Fi hotspot? Re-enable it with Serial command w.')">w · Wi-Fi OFF</button>
    </div>
  </section>

  <section class="card">
    <h2>Activity log</h2>
    <pre id="log">Dashboard started. Sensor data refreshes every 2 seconds.</pre>
  </section>
</main>

<script>
  const $ = id => document.getElementById(id);
  let requestBusy = false;

  function fmt(value, decimals, unit) {
    return Number.isFinite(value) ? value.toFixed(decimals) + unit : "--" + unit;
  }

  function setState(id, value) {
    const el = $(id);
    const on = value === 1;
    el.classList.toggle("on", on);
    el.querySelector("b").textContent = on ? "HIGH" : "LOW";
  }

  function setNtc(prefix, ntc) {
    if (ntc && ntc.valid) {
      $(prefix).textContent = fmt(ntc.temperature_c, 2, " °C");
      $(prefix + "sub").textContent =
        fmt(ntc.voltage_v, 4, " V") + " · " +
        fmt(ntc.resistance_kohm, 2, " kΩ");
    } else {
      $(prefix).textContent = "-- °C";
      $(prefix + "sub").textContent =
        ntc && ntc.status ? ntc.status : "No valid reading";
    }
  }

  async function refresh() {
    if (requestBusy) return;
    try {
      const response = await fetch("/api/status", {cache:"no-store"});
      if (!response.ok) throw new Error("HTTP " + response.status);
      const data = await response.json();

      $("dot").classList.add("ok");
      $("connection").textContent = "Connected · auto refresh 2 s";
      $("network").textContent = "AP: " + data.wifi.ssid + " · " + data.wifi.ip;

      $("pack").textContent = fmt(data.mcp.pack_v, 3, " V");
      $("ch1").textContent = "ADC: " + fmt(data.mcp.ch1_v, 5, " V");
      $("current").textContent = fmt(data.ina.current_ma, 2, " mA");
      $("inaBus").textContent =
        "Bus: " + fmt(data.ina.bus_v, 3, " V") +
        " · Shunt: " + fmt(data.ina.shunt_mv, 3, " mV");

      setNtc("th1", data.ntc.th1);
      setNtc("th2", data.ntc.th2);

      setState("stSolar", data.gpio.solar);
      setState("stLoad", data.gpio.load);
      setState("stHB", data.gpio.hb);
      setState("stLB", data.gpio.lb);
      setState("stLED", data.gpio.led);
    } catch (error) {
      $("dot").classList.remove("ok");
      $("connection").textContent = "Disconnected";
    }
  }

  async function run(command) {
    requestBusy = true;
    document.querySelectorAll("button").forEach(b => b.disabled = true);
    $("log").textContent = "Running command " + command + "...";

    try {
      const response = await fetch("/api/action?c=" + encodeURIComponent(command), {
        method: "POST",
        cache: "no-store"
      });
      const data = await response.json();
      $("log").textContent = data.message || "Command completed.";
    } catch (error) {
      $("log").textContent =
        command === "w"
          ? "Wi-Fi hotspot closed. Use Serial command w to re-enable it."
          : "Command failed: " + error.message;
    } finally {
      requestBusy = false;
      document.querySelectorAll("button").forEach(b => b.disabled = false);
      if (command !== "w") refresh();
    }
  }

  function confirmRun(command, message) {
    if (confirm(message)) run(command);
  }

  refresh();
  setInterval(refresh, 2000);
</script>
</body>
</html>
)HTML";

// ======================================================
// Web API
// ======================================================

String ntcJson(const NTCReading &reading) {
  String json = "{";
  json += "\"valid\":";
  json += reading.valid ? "true" : "false";
  json += ",\"voltage_v\":";
  json += jsonNumber(reading.voltage, 5);
  json += ",\"resistance_kohm\":";
  json += reading.valid
    ? jsonNumber(reading.resistanceOhm / 1000.0, 3)
    : "null";
  json += ",\"temperature_c\":";
  json += reading.valid
    ? jsonNumber(reading.temperatureC, 2)
    : "null";
  json += ",\"status\":\"";
  json += jsonEscape(reading.status);
  json += "\"}";
  return json;
}

String buildStatusJson() {
  INAReading ina = getINA219Reading();
  MCPReading mcp = getMCP3424Reading();

  NTCReading th1 = ntcFromVoltage(
    mcp.found && mcp.valid[2],
    mcp.voltage[2]
  );
  NTCReading th2 = ntcFromVoltage(
    mcp.found && mcp.valid[3],
    mcp.voltage[3]
  );

  String json;
  json.reserve(900);
  json += "{";

  json += "\"wifi\":{\"enabled\":";
  json += wifiEnabled ? "true" : "false";
  json += ",\"ssid\":\"";
  json += jsonEscape(WIFI_AP_SSID);
  json += "\",\"ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\"},";

  json += "\"ina\":{\"found\":";
  json += ina.found ? "true" : "false";
  json += ",\"valid\":";
  json += ina.valid ? "true" : "false";
  json += ",\"bus_v\":";
  json += ina.valid ? jsonNumber(ina.busVoltageV, 3) : "null";
  json += ",\"shunt_mv\":";
  json += ina.valid ? jsonNumber(ina.shuntVoltagemV, 3) : "null";
  json += ",\"current_ma\":";
  json += ina.valid ? jsonNumber(ina.currentmA, 2) : "null";
  json += "},";

  json += "\"mcp\":{\"found\":";
  json += mcp.found ? "true" : "false";
  json += ",\"ch1_v\":";
  json += (mcp.found && mcp.valid[0])
    ? jsonNumber(mcp.voltage[0], 5)
    : "null";
  json += ",\"ch2_v\":";
  json += (mcp.found && mcp.valid[1])
    ? jsonNumber(mcp.voltage[1], 5)
    : "null";
  json += ",\"pack_v\":";
  json += (mcp.found && mcp.valid[0])
    ? jsonNumber(mcp.voltage[0] * PACK_DIVIDER, 3)
    : "null";
  json += "},";

  json += "\"ntc\":{\"th1\":";
  json += ntcJson(th1);
  json += ",\"th2\":";
  json += ntcJson(th2);
  json += "},";

  json += "\"gpio\":{";
  json += "\"solar\":" + String(digitalRead(PIN_SOLAR_EN));
  json += ",\"load\":" + String(digitalRead(PIN_LOAD_EN));
  json += ",\"hb\":" + String(digitalRead(PIN_HB_EN));
  json += ",\"lb\":" + String(digitalRead(PIN_LB_EN));
  json += ",\"led\":" + String(digitalRead(PIN_LED));
  json += "}";

  json += "}";
  return json;
}

void sendNoCacheHeaders() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
}

void handleRoot() {
  sendNoCacheHeaders();
  server.send_P(200, "text/html; charset=utf-8", CONTROL_PAGE);
}

void handleStatus() {
  sendNoCacheHeaders();
  server.send(200, "application/json", buildStatusJson());
}

void handleAction() {
  if (!server.hasArg("c") || server.arg("c").length() != 1) {
    server.send(
      400,
      "application/json",
      "{\"ok\":false,\"message\":\"Missing command.\"}"
    );
    return;
  }

  char command = server.arg("c").charAt(0);
  String result = executeCommand(command, true);

  // Keep a record in Serial Monitor as well.
  Serial.println();
  Serial.print("[WEB command ");
  Serial.print(command);
  Serial.println("]");
  Serial.println(result);

  String response = "{\"ok\":true,\"message\":\"";
  response += jsonEscape(result);
  response += "\"}";

  sendNoCacheHeaders();
  server.send(200, "application/json", response);
}

void configureWebRoutes() {
  if (webRoutesConfigured) {
    return;
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/action", HTTP_POST, handleAction);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  webRoutesConfigured = true;
}

bool startWiFi() {
  if (wifiEnabled) {
    return true;
  }

  configureWebRoutes();
  WiFi.mode(WIFI_AP);

  bool started = WiFi.softAP(
    WIFI_AP_SSID,
    WIFI_AP_PASSWORD
  );

  if (!started) {
    Serial.println("Wi-Fi hotspot failed to start.");
    WiFi.mode(WIFI_OFF);
    return false;
  }

  server.begin();
  wifiEnabled = true;
  wifiOffAtMs = 0;

  Serial.println();
  Serial.println("Wi-Fi hotspot ON");
  Serial.print("SSID: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("Password: ");
  Serial.println(WIFI_AP_PASSWORD);
  Serial.print("Control panel: http://");
  Serial.println(WiFi.softAPIP());
  return true;
}

void stopWiFi() {
  if (!wifiEnabled) {
    wifiOffAtMs = 0;
    return;
  }

  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  wifiOffAtMs = 0;

  Serial.println();
  Serial.println("Wi-Fi hotspot OFF.");
  Serial.println("Enter w to turn it on again.");
}

// ======================================================
// Setup and loop
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SOLAR_EN, OUTPUT);
  pinMode(PIN_LOAD_EN, OUTPUT);
  pinMode(PIN_HB_EN, OUTPUT);
  pinMode(PIN_LB_EN, OUTPUT);

  // Safe state immediately after power-up
  allPowerOutputsOff();
  digitalWrite(PIN_LED, LOW);

  ADCBus.begin(
    PIN_ADC_SDA,
    PIN_ADC_SCL,
    100000
  );

  CurrentBus.begin(
    PIN_CURRENT_SDA,
    PIN_CURRENT_SCL,
    100000
  );

  Serial.println();
  Serial.println("MCU board test started.");
  Serial.println("All power-control outputs are LOW.");

  blinkLED(2, 200);
  printMenu();

  Serial.println();
  Serial.println(scanBothI2CBusesText());

  if (WIFI_START_ON_BOOT) {
    startWiFi();
  }
}

void loop() {
  if (wifiEnabled) {
    server.handleClient();
  }

  if (timeReached(wifiOffAtMs)) {
    stopWiFi();
  }

  if (Serial.available()) {
    char command = Serial.read();

    if (command != '\n' && command != '\r') {
      String result = executeCommand(command, false);

      if (result == "WIFI_START_REQUEST") {
        if (startWiFi()) {
          result = "Wi-Fi hotspot is ON.";
        } else {
          result = "Wi-Fi hotspot could not be started.";
        }
      }

      Serial.println();
      Serial.println(result);
    }
  }

  delay(5);
}