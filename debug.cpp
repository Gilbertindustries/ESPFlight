#include <SPI.h>
#include <Wire.h>
#include <BMI160Gen.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>

// LED Pin definitions
const int LED_PIN_1 = 21;
const int LED_PIN_2 = 47;

// Voltage Divider Pin & Resistors (IO3)
const int VOLTAGE_PIN = 3;
const float R1 = 10000.0; // 10k ohm
const float R2 = 1000.0;  // 1k ohm
const float DIVIDER_RATIO = (R1 + R2) / R2; // 11.0

// SPI Pin definitions (BMI160)
const int BMI_CS_PIN   = 10;
const int BMI_MOSI_PIN = 11;
const int BMI_SCK_PIN  = 12;
const int BMI_MISO_PIN = 13;

// I2C Pin definitions (BMP280)
const int BMP_SDA_PIN = 1;
const int BMP_SCL_PIN = 2;

// Servo Pin definitions
const int SERVO_PINS[6] = {5, 6, 7, 8, 9, 14};
const int NUM_SERVOS = 6;

Servo servos[NUM_SERVOS];

int servoAngle = 0;
int servoDirection = 1;

Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));

  Serial.println("\n--- ESP32-S3 Flight Controller Test Suite ---");

  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  // Configure ADC resolution for ESP32-S3 (12-bit: 0 - 4095)
  analogReadResolution(12);

  // 1. Initialize Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2500);
  }

  // 2. Initialize Hardware SPI for BMI160
  SPI.begin(BMI_SCK_PIN, BMI_MISO_PIN, BMI_MOSI_PIN, BMI_CS_PIN);

  if (!BMI160.begin(BMI160GenClass::SPI_MODE, BMI_CS_PIN)) {
    Serial.println("BMI160 (SPI) initialization failed!");
  } else {
    Serial.println("BMI160 (SPI) initialized successfully!");
  }

  // 3. Initialize Hardware I2C for BMP280
  Wire.begin(BMP_SDA_PIN, BMP_SCL_PIN);

  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("BMP280 (I2C) initialization failed!");
  } else {
    Serial.println("BMP280 (I2C) initialized successfully!");

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

void loop() {
  // --- Servo Sweep Logic ---
  servoAngle += servoDirection * 2;
  if (servoAngle >= 180) {
    servoAngle = 180;
    servoDirection = -1;
  } else if (servoAngle <= 0) {
    servoAngle = 0;
    servoDirection = 1;
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(servoAngle);
  }

  digitalWrite(LED_PIN_1, servoDirection > 0 ? HIGH : LOW);
  digitalWrite(LED_PIN_2, servoDirection < 0 ? HIGH : LOW);

  // --- BMI160 Readings ---
  int ax, ay, az, gx, gy, gz;
  BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);

  // --- BMP280 Readings ---
  float temp = bmp.readTemperature();
  float press = bmp.readPressure() / 100.0F;
  float alt = bmp.readAltitude(1013.25);

  // --- Battery Voltage Reading (IO3) ---
  int rawADC = analogRead(VOLTAGE_PIN);
  float measuredPinVoltage = (rawADC / 4095.0) * 3.3; // Convert raw 12-bit ADC to volts
  float inputVoltage = measuredPinVoltage * DIVIDER_RATIO; // Calculate actual battery voltage

  // Output Telemetry
  Serial.print("Batt: "); Serial.print(inputVoltage, 2); Serial.print("V");
  Serial.print(" | Angle: "); Serial.print(servoAngle); Serial.print("°");
  Serial.print(" | Accel: "); Serial.print(ax); Serial.print("\t"); Serial.print(ay); Serial.print("\t"); Serial.print(az);
  Serial.print(" | Gyro: "); Serial.print(gx); Serial.print("\t"); Serial.print(gy); Serial.print("\t"); Serial.print(gz);
  Serial.print(" | Temp: "); Serial.print(temp, 1); Serial.print("C");
  Serial.print(" | Alt: "); Serial.print(alt, 1); Serial.println("m");

  delay(50);
}
