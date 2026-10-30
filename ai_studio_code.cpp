#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

#define WIFI_CHANNEL 1

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f

// MPU6500: SDA = GPIO21, SCL = GPIO22
#define MPU_SDA 21
#define MPU_SCL 22

// BMP280: SDA = GPIO25, SCL = GPIO26
#define BMP_SDA 25
#define BMP_SCL 26

// DHT11: DATA = GPIO4
#define DHTPIN 4
#define DHTTYPE DHT11

// GPS TX -> ESP32 GPIO16 (RX)
#define GPS_RX_PIN 16

DHT dht(DHTPIN, DHTTYPE);

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Separate I2C buses
TwoWire I2C_MPU = TwoWire(0);
TwoWire I2C_BMP = TwoWire(1);

// BMP280
Adafruit_BMP280 bmp(&I2C_BMP);
bool bmpReady = false;

uint8_t receiverMAC[] = {
  0x20, 0x9B, 0xA9, 0x68, 0xE4, 0x50
};

struct SensorPacket {
  uint32_t packetNo;
  float temperature;
  float humidity;

  float ax;
  float ay;
  float az;
  float totalAcc;

  float gx;
  float gy;
  float gz;

  float roll;
  float pitch;

  double latitude;
  double longitude;
  float altitude;
  float speed;
  uint32_t satellites;
  uint8_t gpsFix;

  // BMP280 telemetry
  float bmpTemperature;
  float pressure;
  float bmpAltitude;

  uint8_t mpuReady;
  uint8_t mpuAddress;
  uint8_t mpuWhoAmI;
};

SensorPacket data;
uint32_t packetCount = 0;

float roll = 0.0f;
float pitch = 0.0f;
unsigned long lastImuTime = 0;
unsigned long lastTelemetryTime = 0;

uint8_t mpuAddress = 0;
uint8_t mpuWhoAmI = 0;
bool mpuInitialized = false;

// Calibration offsets
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Filtered environmental registers
float filteredBmpTemp = 0.0f;
float filteredPressure = 0.0f;
float filteredBmpAlt = 0.0f;

void writeRegister(byte reg, byte value) {
  I2C_MPU.beginTransmission(mpuAddress);
  I2C_MPU.write(reg);
  I2C_MPU.write(value);
  I2C_MPU.endTransmission();
}

byte readRegister(byte reg) {
  I2C_MPU.beginTransmission(mpuAddress);
  I2C_MPU.write(reg);
  I2C_MPU.endTransmission(false);
  I2C_MPU.requestFrom(mpuAddress, (uint8_t)1);
  if (I2C_MPU.available()) {
    return I2C_MPU.read();
  }
  return 0xFF;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  I2C_MPU.beginTransmission(mpuAddress);
  I2C_MPU.write(0x3B);
  if (I2C_MPU.endTransmission(false) != 0) return false;

  uint8_t received = I2C_MPU.requestFrom(mpuAddress, (uint8_t)14);
  if (received < 14) {
    while (I2C_MPU.available()) I2C_MPU.read();
    return false;
  }

  ax = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());
  ay = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());
  az = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());

  I2C_MPU.read(); // Skip temperature
  I2C_MPU.read();

  gx = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());
  gy = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());
  gz = (int16_t)((I2C_MPU.read() << 8) | I2C_MPU.read());

  return true;
}

bool initializeMPU() {
  const uint8_t addresses[] = { 0x68, 0x69 };
  for (uint8_t address : addresses) {
    I2C_MPU.beginTransmission(address);
    if (I2C_MPU.endTransmission() != 0) continue;

    mpuAddress = address;
    mpuWhoAmI = readRegister(0x75);
    if (mpuWhoAmI != 0x68 && mpuWhoAmI != 0x70) continue;

    writeRegister(0x6B, 0x00); // Wake up
    writeRegister(0x1C, 0x00); // 2g
    writeRegister(0x1B, 0x00); // 250 dps
    delay(50);
    return true;
  }
  return false;
}

// 200-sample stationary gyro calibration
void calibrateGyro() {
  if (!mpuInitialized) return;
  Serial.println("Calibrating Gyroscope (Keep CubeSat Stationary)...");

  long sumGx = 0, sumGy = 0, sumGz = 0;
  int validSamples = 0;

  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumGx += gx;
      sumGy += gy;
      sumGz += gz;
      validSamples++;
    }
    delay(5);
  }

  if (validSamples > 50) {
    gyroBiasX = (sumGx / (float)validSamples) / GYRO_SCALE;
    gyroBiasY = (sumGy / (float)validSamples) / GYRO_SCALE;
    gyroBiasZ = (sumGz / (float)validSamples) / GYRO_SCALE;
    Serial.printf("Gyro Biases: X=%.2f Y=%.2f Z=%.2f deg/s\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  }
}

void readBMP280Filtered() {
  if (!bmpReady) return;

  float rawTemp = bmp.readTemperature();
  float rawPress = bmp.readPressure() / 100.0F;
  float rawAlt = bmp.readAltitude(1013.25F);

  if (filteredPressure == 0.0f) {
    filteredBmpTemp = rawTemp;
    filteredPressure = rawPress;
    filteredBmpAlt = rawAlt;
  } else {
    // Low-pass exponential filter
    filteredBmpTemp = 0.90f * filteredBmpTemp + 0.10f * rawTemp;
    filteredPressure = 0.90f * filteredPressure + 0.10f * rawPress;
    filteredBmpAlt = 0.90f * filteredBmpAlt + 0.10f * rawAlt;
  }

  data.bmpTemperature = filteredBmpTemp;
  data.pressure = filteredPressure;
  data.bmpAltitude = filteredBmpAlt;
}

void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);

  dht.begin();
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1);

  I2C_MPU.begin(MPU_SDA, MPU_SCL);
  I2C_MPU.setClock(400000); // 400kHz Fast I2C

  mpuInitialized = initializeMPU();
  if (mpuInitialized) {
    calibrateGyro();
    // Initialize angles from stationary accelerometer
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (readMPU(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      float ax = axRaw / ACCEL_SCALE;
      float ay = ayRaw / ACCEL_SCALE;
      float az = azRaw / ACCEL_SCALE;
      roll = atan2(ay, az) * 180.0f / PI;
      pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    }
  }

  I2C_BMP.begin(BMP_SDA, BMP_SCL);
  I2C_BMP.setClock(100000);
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmpReady = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onSent);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  lastImuTime = millis();
  lastTelemetryTime = millis();
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  unsigned long currentMillis = millis();

  // 1. FAST 50 Hz (20ms) IMU Sensor Fusion Loop
  if (currentMillis - lastImuTime >= 20) {
    float dt = (currentMillis - lastImuTime) / 1000.0f;
    lastImuTime = currentMillis;

    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (mpuInitialized && readMPU(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      float ax = axRaw / ACCEL_SCALE;
      float ay = ayRaw / ACCEL_SCALE;
      float az = azRaw / ACCEL_SCALE;

      // Subtract calibrated gyroscope biases
      float gx = (gxRaw / GYRO_SCALE) - gyroBiasX;
      float gy = (gyRaw / GYRO_SCALE) - gyroBiasY;
      float gz = (gzRaw / GYRO_SCALE) - gyroBiasZ;

      // Noise deadband: removes stationary MEMS jitter
      if (fabs(gx) < 0.25f) gx = 0.0f;
      if (fabs(gy) < 0.25f) gy = 0.0f;
      if (fabs(gz) < 0.25f) gz = 0.0f;

      float totalAcc = sqrt(ax * ax + ay * ay + az * az);

      // Accelerometer tilt reference
      float accelRoll = atan2(ay, az) * 180.0f / PI;
      float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

      // Complementary Filter with true 20ms dt
      roll += gx * dt;
      pitch += gy * dt;

      // Only trust accelerometer gravity vector when total acceleration is near 1g
      if (totalAcc > 0.85f && totalAcc < 1.15f) {
        roll = 0.98f * roll + 0.02f * accelRoll;
        pitch = 0.98f * pitch + 0.02f * accelPitch;
      }

      data.ax = ax;
      data.ay = ay;
      data.az = az;
      data.totalAcc = totalAcc;
      data.gx = gx;
      data.gy = gy;
      data.gz = gz;
      data.roll = roll;
      data.pitch = pitch;
      data.mpuReady = 1;
    } else {
      data.mpuReady = 0;
    }
  }

  // 2. Telemetry Dispatch Loop (every 1 second)
  if (currentMillis - lastTelemetryTime >= 1000) {
    lastTelemetryTime = currentMillis;

    data.packetNo = ++packetCount;

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      data.temperature = t;
      data.humidity = h;
    }

    readBMP280Filtered();

    data.gpsFix = gps.location.isValid() ? 1 : 0;
    data.latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    data.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    data.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    data.speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

    data.mpuAddress = mpuAddress;
    data.mpuWhoAmI = mpuWhoAmI;

    esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
  }
}