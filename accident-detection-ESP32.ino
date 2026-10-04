#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// ======================================================
// WIFI CONFIGURATION
// ======================================================

const char* WIFI_SSID = "ESP32_TEST";
const char* WIFI_PASSWORD = "12345678";

const char* SERVER_URL =
  "http:// 10.34.119.141:5000/api/esp32";


// ======================================================
// PIN CONFIGURATION
// ======================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define BUZZER_PIN 5     // D5 / GPIO5 - ACTIVE LOW
#define MOTOR_PIN 17     // TX2 / GPIO17
#define LED1_PIN 27
#define LED2_PIN 16


// ======================================================
// MPU6500
// ======================================================

#define MPU6500_ADDR 0x68

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B


// ======================================================
// ACCIDENT DETECTION SETTINGS
// ======================================================

// MPU6500 accelerometer = +/-4g
// 8192 LSB/g

// Strong physical shock required
const float SHOCK_THRESHOLD = 2.50;

// Sudden change in total acceleration required
const float DELTA_G_THRESHOLD = 1.20;

// Significant rotation required
const float GYRO_THRESHOLD = 180.0;

// All three conditions must occur inside this window
const unsigned long IMPACT_WINDOW = 120;

// Emergency timing
const unsigned long COOLDOWN_TIME = 5000;
const unsigned long ALERT_DURATION = 5000;


// ======================================================
// TELEMETRY
// ======================================================

const unsigned long TELEMETRY_INTERVAL = 500;

volatile float sharedAX = 0.0;
volatile float sharedAY = 0.0;
volatile float sharedAZ = 1.0;

volatile float sharedGX = 0.0;
volatile float sharedGY = 0.0;
volatile float sharedGZ = 0.0;

volatile float sharedTotalG = 1.0;
volatile float sharedLinearG = 0.0;

volatile bool sharedImpact = false;
volatile bool sharedAlert = false;

unsigned long lastTelemetrySend = 0;


// ======================================================
// SYSTEM VARIABLES
// ======================================================

volatile bool alertActive = false;

unsigned long alertStartTime = 0;
unsigned long lastAlertTime = 0;

float previousTotalG = 1.0;

unsigned long lastSerialPrint = 0;


// ======================================================
// IMPACT WINDOW VARIABLES
// ======================================================

bool impactWindowActive = false;

unsigned long impactWindowStart = 0;

bool shockCondition = false;
bool deltaCondition = false;
bool gyroCondition = false;


// ======================================================
// BUZZER
// ACTIVE LOW
// ======================================================

void buzzerON()
{
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerOFF()
{
  digitalWrite(BUZZER_PIN, HIGH);
}


// ======================================================
// OUTPUTS OFF
// ======================================================

void outputsOFF()
{
  buzzerOFF();

  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
}


// ======================================================
// WIFI CONNECTION
// ======================================================

void connectWiFi()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("       ESP32 Wi-Fi");
  Serial.println("================================");

  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  unsigned long startTime = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - startTime < 15000
  )
  {
    delay(250);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Wi-Fi CONNECTED!");

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
  else
  {
    Serial.println("Wi-Fi connection failed.");
    Serial.println("Safety detection will continue.");
  }

  Serial.println();
}


// ======================================================
// WIFI RECONNECT
// ======================================================

void checkWiFi()
{
  static unsigned long lastCheck = 0;

  if (millis() - lastCheck < 5000)
  {
    return;
  }

  lastCheck = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  Serial.println("Wi-Fi disconnected.");
  Serial.println("Trying to reconnect...");

  WiFi.disconnect();
  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );
}


// ======================================================
// MPU WRITE
// ======================================================

void writeMPU(
  uint8_t reg,
  uint8_t value
)
{
  Wire.beginTransmission(
    MPU6500_ADDR
  );

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


// ======================================================
// MPU READ REGISTER
// ======================================================

uint8_t readMPU(
  uint8_t reg
)
{
  Wire.beginTransmission(
    MPU6500_ADDR
  );

  Wire.write(reg);

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return 0xFF;
  }

  Wire.requestFrom(
    MPU6500_ADDR,
    (uint8_t)1
  );

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0xFF;
}


// ======================================================
// READ MPU6500 ACCEL + GYRO
// ======================================================

bool readMPU6500(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
)
{
  Wire.beginTransmission(
    MPU6500_ADDR
  );

  Wire.write(
    ACCEL_XOUT_H
  );

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return false;
  }

  Wire.requestFrom(
    MPU6500_ADDR,
    (uint8_t)14
  );

  if (Wire.available() < 14)
  {
    return false;
  }


  int16_t rawAX =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawAY =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawAZ =
    (Wire.read() << 8) |
    Wire.read();


  // Temperature - skip
  Wire.read();
  Wire.read();


  int16_t rawGX =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawGY =
    (Wire.read() << 8) |
    Wire.read();

  int16_t rawGZ =
    (Wire.read() << 8) |
    Wire.read();


  // ------------------------------------------
  // Accelerometer +/-4g
  // ------------------------------------------

  ax = rawAX / 8192.0;
  ay = rawAY / 8192.0;
  az = rawAZ / 8192.0;


  // ------------------------------------------
  // Gyroscope +/-500 deg/s
  // ------------------------------------------

  gx = rawGX / 65.5;
  gy = rawGY / 65.5;
  gz = rawGZ / 65.5;


  return true;
}


// ======================================================
// EMERGENCY ON
// ======================================================

void emergencyON()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("       !!! ACCIDENT DETECTED !!!");
  Serial.println("       EMERGENCY ALERT ACTIVE");
  Serial.println("======================================");


  // LED1
  digitalWrite(
    LED1_PIN,
    HIGH
  );


  // LED2
  digitalWrite(
    LED2_PIN,
    HIGH
  );


  // Vibrator
  digitalWrite(
    MOTOR_PIN,
    HIGH
  );


  // Active LOW buzzer
  buzzerON();


  alertActive = true;

  alertStartTime =
    millis();

  lastAlertTime =
    millis();

  sharedAlert = true;


  // Reset impact window
  impactWindowActive = false;

  shockCondition = false;
  deltaCondition = false;
  gyroCondition = false;
}


// ======================================================
// EMERGENCY OFF
// ======================================================

void emergencyOFF()
{
  outputsOFF();

  alertActive = false;

  sharedAlert = false;

  Serial.println(
    "Emergency alert OFF"
  );

  Serial.println();
}


// ======================================================
// RESET IMPACT WINDOW
// ======================================================

void resetImpactWindow()
{
  impactWindowActive = false;

  shockCondition = false;
  deltaCondition = false;
  gyroCondition = false;
}


// ======================================================
// ACCIDENT DETECTION
// ======================================================

bool detectAccident(
  float linearG,
  float deltaG,
  float gyroMagnitude
)
{
  unsigned long now =
    millis();


  // --------------------------------------------------
  // Check individual conditions
  // --------------------------------------------------

  bool shock =
    linearG >= SHOCK_THRESHOLD;

  bool delta =
    deltaG >= DELTA_G_THRESHOLD;

  bool gyro =
    gyroMagnitude >= GYRO_THRESHOLD;


  // --------------------------------------------------
  // Start a new impact window
  // --------------------------------------------------

  if (
    !impactWindowActive &&
    (shock || delta || gyro)
  )
  {
    impactWindowActive = true;

    impactWindowStart =
      now;

    shockCondition = shock;
    deltaCondition = delta;
    gyroCondition = gyro;
  }


  // --------------------------------------------------
  // Update conditions during window
  // --------------------------------------------------

  if (impactWindowActive)
  {
    if (shock)
    {
      shockCondition = true;
    }

    if (delta)
    {
      deltaCondition = true;
    }

    if (gyro)
    {
      gyroCondition = true;
    }
  }


  // --------------------------------------------------
  // Check if all 3 happened within 120 ms
  // --------------------------------------------------

  if (
    impactWindowActive &&
    shockCondition &&
    deltaCondition &&
    gyroCondition
  )
  {
    resetImpactWindow();

    return true;
  }


  // --------------------------------------------------
  // Window expired
  // --------------------------------------------------

  if (
    impactWindowActive &&
    now - impactWindowStart >
      IMPACT_WINDOW
  )
  {
    resetImpactWindow();
  }


  return false;
}


// ======================================================
// SEND TELEMETRY
// ======================================================

void sendTelemetry()
{
  if (
    WiFi.status() != WL_CONNECTED
  )
  {
    return;
  }


  float ax = sharedAX;
  float ay = sharedAY;
  float az = sharedAZ;

  float gx = sharedGX;
  float gy = sharedGY;
  float gz = sharedGZ;

  float totalG =
    sharedTotalG;

  float linearG =
    sharedLinearG;

  bool impact =
    sharedImpact;

  bool alert =
    sharedAlert;


  HTTPClient http;

  http.setConnectTimeout(300);
  http.setTimeout(500);

  http.begin(
    SERVER_URL
  );

  http.addHeader(
    "Content-Type",
    "application/json"
  );


  String json = "{";

  json +=
    "\"device_id\":\"RG-001\",";

  json +=
    "\"accel_x\":" +
    String(ax, 3) +
    ",";

  json +=
    "\"accel_y\":" +
    String(ay, 3) +
    ",";

  json +=
    "\"accel_z\":" +
    String(az, 3) +
    ",";

  json +=
    "\"total_g\":" +
    String(totalG, 3) +
    ",";

  json +=
    "\"linear_g\":" +
    String(linearG, 3) +
    ",";

  json +=
    "\"gyro_x\":" +
    String(gx, 1) +
    ",";

  json +=
    "\"gyro_y\":" +
    String(gy, 1) +
    ",";

  json +=
    "\"gyro_z\":" +
    String(gz, 1) +
    ",";

  json +=
    "\"impact\":";

  json +=
    impact
      ? "true,"
      : "false,";

  json +=
    "\"alert_active\":";

  json +=
    alert
      ? "true,"
      : "false,";

  json +=
    "\"uptime_ms\":" +
    String(millis());

  json += "}";


  int responseCode =
    http.POST(json);


  if (
    responseCode == 200 ||
    responseCode == 201
  )
  {
    Serial.print(
      "Telemetry OK: "
    );

    Serial.println(
      responseCode
    );
  }
  else if (responseCode > 0)
  {
    Serial.print(
      "Telemetry HTTP error: "
    );

    Serial.println(
      responseCode
    );
  }
  else
  {
    Serial.print(
      "Telemetry failed: "
    );

    Serial.println(
      http.errorToString(
        responseCode
      )
    );
  }


  http.end();
}


// ======================================================
// TELEMETRY TASK
// ======================================================

void telemetryTask(
  void* parameter
)
{
  while (true)
  {
    if (
      millis() -
      lastTelemetrySend >=
      TELEMETRY_INTERVAL
    )
    {
      lastTelemetrySend =
        millis();

      sendTelemetry();
    }


    vTaskDelay(
      pdMS_TO_TICKS(20)
    );
  }
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1000);


  Serial.println();

  Serial.println(
    "======================================"
  );

  Serial.println(
    "   SMART VEHICLE SAFETY SYSTEM"
  );

  Serial.println(
    "======================================"
  );


  // ====================================================
  // OUTPUT PINS
  // ====================================================

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  pinMode(
    MOTOR_PIN,
    OUTPUT
  );

  pinMode(
    LED1_PIN,
    OUTPUT
  );

  pinMode(
    LED2_PIN,
    OUTPUT
  );


  outputsOFF();


  // ====================================================
  // I2C
  // ====================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(
    400000
  );

  delay(100);


  // ====================================================
  // CHECK MPU6500
  // ====================================================

  Serial.println(
    "Checking MPU6500..."
  );

  uint8_t id =
    readMPU(
      WHO_AM_I
    );


  Serial.print(
    "WHO_AM_I = 0x"
  );

  Serial.println(
    id,
    HEX
  );


  if (id == 0x70)
  {
    Serial.println(
      "MPU6500 detected successfully!"
    );
  }
  else
  {
    Serial.println(
      "ERROR: MPU6500 NOT detected!"
    );
  }


  // ====================================================
  // WAKE SENSOR
  // ====================================================

  writeMPU(
    PWR_MGMT_1,
    0x00
  );

  delay(100);


  // ====================================================
  // ACCELEROMETER +/-4g
  // ====================================================

  writeMPU(
    ACCEL_CONFIG,
    0x08
  );


  // ====================================================
  // GYROSCOPE +/-500 deg/s
  // ====================================================

  writeMPU(
    GYRO_CONFIG,
    0x08
  );

  delay(100);


  // ====================================================
  // WIFI
  // ====================================================

  connectWiFi();


  // ====================================================
  // START TELEMETRY TASK
  // ====================================================

  xTaskCreatePinnedToCore(
    telemetryTask,
    "TelemetryTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );


  // ====================================================
  // SYSTEM READY
  // ====================================================

  Serial.println();

  Serial.println(
    "======================================"
  );

  Serial.println(
    "          SYSTEM READY"
  );

  Serial.println(
    "======================================"
  );


  Serial.print(
    "Shock threshold: "
  );

  Serial.print(
    SHOCK_THRESHOLD,
    2
  );

  Serial.println(
    " g"
  );


  Serial.print(
    "Delta threshold: "
  );

  Serial.print(
    DELTA_G_THRESHOLD,
    2
  );

  Serial.println(
    " g"
  );


  Serial.print(
    "Gyro threshold: "
  );

  Serial.print(
    GYRO_THRESHOLD,
    1
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "Impact window: "
  );

  Serial.print(
    IMPACT_WINDOW
  );

  Serial.println(
    " ms"
  );


  Serial.println();
}


// ======================================================
// MAIN SENSOR LOOP
// ======================================================

void loop()
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;


  // ====================================================
  // WIFI CHECK
  // ====================================================

  checkWiFi();


  // ====================================================
  // READ SENSOR
  // ====================================================

  if (
    !readMPU6500(
      ax,
      ay,
      az,
      gx,
      gy,
      gz
    )
  )
  {
    Serial.println(
      "MPU6500 READ ERROR"
    );

    delay(5);

    return;
  }


  // ====================================================
  // TOTAL ACCELERATION
  // ====================================================

  float totalG =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );


  // ====================================================
  // LINEAR SHOCK
  // ====================================================

  float linearG =
    fabs(
      totalG - 1.0
    );


  // ====================================================
  // GYRO MAGNITUDE
  // ====================================================

  float gyroMagnitude =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );


  // ====================================================
  // DELTA G
  // ====================================================

  float deltaG =
    fabs(
      totalG -
      previousTotalG
    );


  previousTotalG =
    totalG;


  // ====================================================
  // MULTI-CONDITION ACCIDENT DETECTION
  // ====================================================

  bool accidentDetected =
    detectAccident(
      linearG,
      deltaG,
      gyroMagnitude
    );


  // ====================================================
  // UPDATE TELEMETRY DATA
  // ====================================================

  sharedAX = ax;
  sharedAY = ay;
  sharedAZ = az;

  sharedGX = gx;
  sharedGY = gy;
  sharedGZ = gz;

  sharedTotalG =
    totalG;

  sharedLinearG =
    linearG;

  sharedImpact =
    accidentDetected;


  // ====================================================
  // EMERGENCY ON
  // ====================================================

  if (
    accidentDetected &&
    !alertActive &&
    millis() -
      lastAlertTime >=
      COOLDOWN_TIME
  )
  {
    emergencyON();
  }


  // ====================================================
  // ALERT TIMEOUT
  // ====================================================

  if (
    alertActive &&
    millis() -
      alertStartTime >=
      ALERT_DURATION
  )
  {
    emergencyOFF();
  }


  // ====================================================
  // SERIAL MONITOR
  // ====================================================

  if (
    millis() -
    lastSerialPrint >=
    100
  )
  {
    lastSerialPrint =
      millis();


    Serial.print(
      "AX: "
    );

    Serial.print(
      ax,
      2
    );

    Serial.print(
      " g | AY: "
    );

    Serial.print(
      ay,
      2
    );

    Serial.print(
      " g | AZ: "
    );

    Serial.print(
      az,
      2
    );

    Serial.print(
      " g | TOTAL: "
    );

    Serial.print(
      totalG,
      2
    );

    Serial.print(
      " g | LINEAR: "
    );

    Serial.print(
      linearG,
      2
    );

    Serial.print(
      " g | DELTA: "
    );

    Serial.print(
      deltaG,
      2
    );

    Serial.print(
      " g | GX: "
    );

    Serial.print(
      gx,
      1
    );

    Serial.print(
      " | GY: "
    );

    Serial.print(
      gy,
      1
    );

    Serial.print(
      " | GZ: "
    );

    Serial.print(
      gz,
      1
    );

    Serial.print(
      " | IMPACT: "
    );

    Serial.print(
      accidentDetected
        ? "YES"
        : "NO"
    );

    Serial.print(
      " | ALERT: "
    );

    Serial.println(
      alertActive
        ? "ON"
        : "OFF"
    );
  }


  delay(2);
}