#include <Arduino.h>
#include <Wire.h>

// ======================================================
// GPIO分配：根据你的MCU原理图
// 这里写的是GPIO编号，不是ESP32模块物理脚号
// ======================================================

// MCP3424 I2C
const int PIN_ADC_SDA = 32;
const int PIN_ADC_SCL = 33;

// INA219 I2C
const int PIN_CURRENT_SDA = 16;
const int PIN_CURRENT_SCL = 17;

// 控制输出
const int PIN_SOLAR_EN = 25;
const int PIN_LOAD_EN  = 26;
const int PIN_HB_EN    = 27;
const int PIN_LB_EN    = 13;

// 板载LED：GPIO18 → 2.2kΩ → LED → GND
const int PIN_LED = 18;

// 如果CH1使用75kΩ/25kΩ分压，则倍数为4
// 如果75kΩ已经改成82kΩ，则改成4.28
const float PACK_DIVIDER = 4.0;

// 两组独立I2C控制器
TwoWire ADCBus(0);
TwoWire CurrentBus(1);


// ======================================================
// 输出安全控制
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

void pulseOutput(
  const char *name,
  int pin,
  unsigned long durationMs
) {
  Serial.println();
  Serial.print(name);
  Serial.println(" = HIGH");

  digitalWrite(pin, HIGH);
  digitalWrite(PIN_LED, HIGH);

  delay(durationMs);

  digitalWrite(pin, LOW);
  digitalWrite(PIN_LED, LOW);

  Serial.print(name);
  Serial.println(" = LOW");
}


// ======================================================
// I2C扫描
// ======================================================

void scanI2C(TwoWire &bus, const char *busName) {
  Serial.println();
  Serial.print("Scanning ");
  Serial.println(busName);

  int found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();

    if (error == 0) {
      Serial.print("  Device found at 0x");

      if (address < 0x10) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  No I2C device found.");
  }
}


// ======================================================
// INA219
// ======================================================

uint8_t findINA219() {
  // INA219地址范围通常为0x40～0x4F
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

void readINA219() {
  Serial.println();
  Serial.println("----- INA219 -----");

  uint8_t address = findINA219();

  if (address == 0) {
    Serial.println("INA219 not found.");
    return;
  }

  Serial.print("INA219 address: 0x");
  Serial.println(address, HEX);

  uint16_t shuntRegister;
  uint16_t busRegister;

  bool shuntOK = readRegister16(
    CurrentBus,
    address,
    0x01,
    shuntRegister
  );

  bool busOK = readRegister16(
    CurrentBus,
    address,
    0x02,
    busRegister
  );

  if (!shuntOK || !busOK) {
    Serial.println("INA219 register read failed.");
    return;
  }

  int16_t signedShunt = (int16_t)shuntRegister;

  // Shunt voltage LSB = 10uV = 0.01mV
  float shuntVoltage_mV = signedShunt * 0.01;

  // 按之前原理图中的0.05Ω采样电阻计算
  float estimatedCurrent_mA = shuntVoltage_mV / 0.05;

  // Bus voltage LSB = 4mV
  float busVoltage_V = (busRegister >> 3) * 0.004;

  Serial.print("Bus voltage: ");
  Serial.print(busVoltage_V, 3);
  Serial.println(" V");

  Serial.print("Shunt voltage: ");
  Serial.print(shuntVoltage_mV, 3);
  Serial.println(" mV");

  Serial.print("Estimated current: ");
  Serial.print(estimatedCurrent_mA, 2);
  Serial.println(" mA");
}


// ======================================================
// MCP3424
// ======================================================

uint8_t findMCP3424() {
  // MCP3424地址范围为0x68～0x6F
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
    bit 7    RDY：开始转换
    bit 6:5  通道选择
    bit 4    连续转换
    bit 3:2  16-bit模式
    bit 1:0  PGA增益x1
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

    // RDY=0表示转换结束
    if ((returnedConfig & 0x80) == 0) {
      int16_t rawValue =
        (int16_t)(((uint16_t)msb << 8) | lsb);

      // 16-bit、PGA x1：1 LSB = 62.5uV
      voltage = rawValue * 0.0000625;

      return true;
    }

    delay(25);
  }

  return false;
}

void readMCP3424() {
  Serial.println();
  Serial.println("----- MCP3424 -----");

  uint8_t address = findMCP3424();

  if (address == 0) {
    Serial.println("MCP3424 not found.");
    return;
  }

  Serial.print("MCP3424 address: 0x");
  Serial.println(address, HEX);

  float channelVoltage[4] = {0, 0, 0, 0};

  for (int channel = 1; channel <= 4; channel++) {
    bool ok = readMCP3424Channel(
      address,
      channel,
      channelVoltage[channel - 1]
    );

    Serial.print("CH");
    Serial.print(channel);
    Serial.print(": ");

    if (ok) {
      Serial.print(channelVoltage[channel - 1], 5);
      Serial.println(" V");
    } else {
      Serial.println("read failed");
    }
  }

  Serial.println();
  Serial.print("CH1 estimated input: ");
  Serial.print(channelVoltage[0] * PACK_DIVIDER, 3);
  Serial.println(" V");
}


// ======================================================
// GPIO当前状态
// ======================================================

void printOutputStates() {
  Serial.println();
  Serial.println("----- GPIO STATES -----");

  Serial.print("SolarSideEN GPIO23: ");
  Serial.println(digitalRead(PIN_SOLAR_EN));

  Serial.print("Load EN GPIO25:     ");
  Serial.println(digitalRead(PIN_LOAD_EN));

  Serial.print("HB EN GPIO26:       ");
  Serial.println(digitalRead(PIN_HB_EN));

  Serial.print("LB EN GPIO13:       ");
  Serial.println(digitalRead(PIN_LB_EN));

  Serial.print("LED GPIO18:         ");
  Serial.println(digitalRead(PIN_LED));
}


// ======================================================
// 串口菜单
// ======================================================

void printMenu() {
  Serial.println();
  Serial.println("========== MCU BOARD TEST ==========");
  Serial.println("i : scan both I2C buses");
  Serial.println("r : read INA219 and MCP3424");
  Serial.println("d : show output GPIO states");
  Serial.println("e : blink onboard LED 3 times");
  Serial.println("s : pulse SolarSideEN for 2 seconds");
  Serial.println("l : pulse Load Switch for 2 seconds");
  Serial.println("h : pulse HB EN for 0.5 second");
  Serial.println("b : pulse LB EN for 0.5 second");
  Serial.println("0 : immediately turn power outputs OFF");
  Serial.println("m : show this menu");
  Serial.println("====================================");
}


// ======================================================
// 初始化
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_SOLAR_EN, OUTPUT);
  pinMode(PIN_LOAD_EN, OUTPUT);
  pinMode(PIN_HB_EN, OUTPUT);
  pinMode(PIN_LB_EN, OUTPUT);

  // 上电后立即关闭所有功率控制信号
  allPowerOutputsOff();
  digitalWrite(PIN_LED, LOW);

  // 初始化两组独立I2C
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

  // LED闪烁两次表示程序已经运行
  blinkLED(2, 200);

  printMenu();

  // 上电自动扫描，但不自动打开任何控制信号
  scanI2C(ADCBus, "ADC bus: GPIO27 SDA / GPIO32 SCL");
  scanI2C(CurrentBus, "Current bus: GPIO16 SDA / GPIO17 SCL");
}


// ======================================================
// 主循环
// ======================================================

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    switch (command) {
      case 'i':
        scanI2C(
          ADCBus,
          "ADC bus: GPIO27 SDA / GPIO32 SCL"
        );

        scanI2C(
          CurrentBus,
          "Current bus: GPIO16 SDA / GPIO17 SCL"
        );
        break;

      case 'r':
        readINA219();
        readMCP3424();
        break;

      case 'd':
        printOutputStates();
        break;

      case 'e':
        Serial.println("Testing onboard LED...");
        blinkLED(3, 300);
        break;

      case 's':
        pulseOutput(
          "SolarSideEN GPIO23",
          PIN_SOLAR_EN,
          2000
        );
        break;

      case 'l':
        pulseOutput(
          "Load Switch GPIO25",
          PIN_LOAD_EN,
          2000
        );
        break;

      case 'h':
        pulseOutput(
          "HB EN GPIO26",
          PIN_HB_EN,
          500
        );
        break;

      case 'b':
        pulseOutput(
          "LB EN GPIO13",
          PIN_LB_EN,
          500
        );
        break;

      case '0':
        allPowerOutputsOff();
        Serial.println("All power-control outputs are LOW.");
        break;

      case 'm':
        printMenu();
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.println("Unknown command. Enter m for menu.");
        break;
    }
  }

  delay(10);
}