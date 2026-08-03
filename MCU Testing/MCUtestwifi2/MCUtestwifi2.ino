#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ESP32模块物理Pin 30对应GPIO18
const int LED_PIN = 18;

// ESP32创建的Wi-Fi热点
const char *AP_SSID = "ESP32_LED_Control";
const char *AP_PASSWORD = "12345678";

// PWM参数
const uint32_t PWM_CARRIER_HZ = 5000;
const uint8_t PWM_RESOLUTION = 8;  // 8位：0～255

WebServer server(80);

// 当前控制参数
int brightnessPercent = 70;       // 0～100%
float blinkFrequency = 1.0;       // 0～10 Hz，0表示常亮

bool ledState = true;
unsigned long lastToggleTime = 0;

// 手机控制网页
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport"
        content="width=device-width, initial-scale=1.0">

  <title>ESP32 LED Control</title>

  <style>
    body {
      font-family: Arial, sans-serif;
      background: #f2f4f7;
      margin: 0;
      padding: 20px;
      text-align: center;
    }

    .card {
      max-width: 420px;
      margin: 30px auto;
      padding: 25px;
      background: white;
      border-radius: 16px;
      box-shadow: 0 4px 18px rgba(0,0,0,0.12);
    }

    h1 {
      font-size: 24px;
      margin-bottom: 25px;
    }

    .control {
      margin: 25px 0;
      text-align: left;
    }

    label {
      display: block;
      font-size: 18px;
      margin-bottom: 10px;
    }

    input[type="range"] {
      width: 100%;
    }

    .value {
      font-weight: bold;
    }

    button {
      width: 100%;
      padding: 14px;
      margin-top: 10px;
      border: none;
      border-radius: 10px;
      font-size: 17px;
      cursor: pointer;
    }

    .apply {
      background: #1976d2;
      color: white;
    }

    .steady {
      background: #43a047;
      color: white;
    }

    .off {
      background: #e53935;
      color: white;
    }

    #status {
      margin-top: 20px;
      color: #333;
      min-height: 24px;
    }
  </style>
</head>

<body>
  <div class="card">
    <h1>ESP32 LED控制</h1>

    <div class="control">
      <label>
        亮度：
        <span class="value" id="brightnessValue">70%</span>
      </label>

      <input
        id="brightness"
        type="range"
        min="0"
        max="100"
        value="70"
        oninput="updateLabels()">
    </div>

    <div class="control">
      <label>
        闪烁频率：
        <span class="value" id="frequencyValue">1.0 Hz</span>
      </label>

      <input
        id="frequency"
        type="range"
        min="0"
        max="10"
        step="0.1"
        value="1.0"
        oninput="updateLabels()">

      <p>频率设为0 Hz时，LED保持常亮。</p>
    </div>

    <button class="apply" onclick="applySettings()">
      应用设置
    </button>

    <button class="steady" onclick="setSteady()">
      常亮
    </button>

    <button class="off" onclick="turnOff()">
      关闭LED
    </button>

    <div id="status">等待控制</div>
  </div>

  <script>
    function updateLabels() {
      const brightness =
        document.getElementById("brightness").value;

      const frequency =
        document.getElementById("frequency").value;

      document.getElementById("brightnessValue").innerText =
        brightness + "%";

      document.getElementById("frequencyValue").innerText =
        Number(frequency).toFixed(1) + " Hz";
    }

    async function applySettings() {
      const brightness =
        document.getElementById("brightness").value;

      const frequency =
        document.getElementById("frequency").value;

      document.getElementById("status").innerText =
        "正在发送设置……";

      try {
        const response = await fetch(
          "/set?brightness=" + brightness +
          "&frequency=" + frequency
        );

        const result = await response.json();

        document.getElementById("status").innerText =
          "已设置：亮度 " +
          result.brightness +
          "%，频率 " +
          result.frequency +
          " Hz";
      }
      catch (error) {
        document.getElementById("status").innerText =
          "通信失败";
      }
    }

    function setSteady() {
      document.getElementById("frequency").value = 0;
      updateLabels();
      applySettings();
    }

    function turnOff() {
      document.getElementById("brightness").value = 0;
      updateLabels();
      applySettings();
    }

    updateLabels();
  </script>
</body>
</html>
)rawliteral";

uint32_t brightnessToDuty(int percent)
{
  percent = constrain(percent, 0, 100);

  // 将0～100%转换成0～255
  return (percent * 255UL) / 100UL;
}

void updateLedOutput()
{
  uint32_t duty = brightnessToDuty(brightnessPercent);

  if (brightnessPercent == 0 || !ledState)
  {
    ledcWrite(LED_PIN, 0);
  }
  else
  {
    ledcWrite(LED_PIN, duty);
  }
}

void handleRoot()
{
  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML
  );
}

void handleSet()
{
  if (server.hasArg("brightness"))
  {
    brightnessPercent =
      constrain(server.arg("brightness").toInt(), 0, 100);
  }

  if (server.hasArg("frequency"))
  {
    blinkFrequency =
      constrain(server.arg("frequency").toFloat(),
                0.0f,
                10.0f);
  }

  // 每次修改后从亮灯状态重新开始
  ledState = true;
  lastToggleTime = millis();
  updateLedOutput();

  Serial.printf(
    "New setting | Brightness: %d%% | Frequency: %.1f Hz\n",
    brightnessPercent,
    blinkFrequency
  );

  String response;
  response += "{\"brightness\":";
  response += brightnessPercent;
  response += ",\"frequency\":";
  response += String(blinkFrequency, 1);
  response += "}";

  server.send(
    200,
    "application/json",
    response
  );
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32 LED web control starting");
  Serial.println("==============================");

  // 绑定GPIO18到LEDC PWM
  bool pwmReady = ledcAttach(
    LED_PIN,
    PWM_CARRIER_HZ,
    PWM_RESOLUTION
  );

  if (!pwmReady)
  {
    Serial.println("ERROR: LEDC PWM setup failed.");
    while (true)
    {
      delay(1000);
    }
  }

  updateLedOutput();

  // 建立ESP32自己的Wi-Fi热点
  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(
    AP_SSID,
    AP_PASSWORD
  );

  if (!apStarted)
  {
    Serial.println("ERROR: Wi-Fi AP start failed.");
    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("Wi-Fi hotspot started");
  Serial.printf("SSID: %s\n", AP_SSID);
  Serial.printf("Password: %s\n", AP_PASSWORD);
  Serial.print("Control page: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSet);

  server.onNotFound([]()
  {
    server.send(
      404,
      "text/plain",
      "Page not found"
    );
  });

  server.begin();

  Serial.println("Web server started.");
}

void loop()
{
  // 处理手机网页请求
  server.handleClient();

  if (brightnessPercent == 0)
  {
    if (ledState)
    {
      ledState = false;
      updateLedOutput();
    }

    return;
  }

  // 0 Hz表示常亮
  if (blinkFrequency <= 0.01f)
  {
    if (!ledState)
    {
      ledState = true;
      updateLedOutput();
    }

    return;
  }

  // 一个闪烁周期包含亮和灭两个半周期
  unsigned long halfPeriod =
    static_cast<unsigned long>(
      500.0f / blinkFrequency
    );

  if (millis() - lastToggleTime >= halfPeriod)
  {
    lastToggleTime = millis();
    ledState = !ledState;
    updateLedOutput();
  }
}