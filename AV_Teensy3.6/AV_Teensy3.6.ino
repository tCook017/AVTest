#include <Adafruit_Sensor.h>     // sensor abstraction library
#include <Adafruit_FXAS21002C.h> // gyroscope
#include <Adafruit_FXOS8700.h>   // accelerometer and magnetometer
#include <Adafruit_BMP3XX.h>     // barometric pressure sensor
#include <Wire.h>

// The FXAS21002C and the FXOS8700 constitute the NXP 9-DOF board
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accel = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_BMP3XX bmp = Adafruit_BMP3XX();

void setup() {
  Serial.begin(9600);

  // Setup I2C bus (Used by the NXP 9-DOF & the BMP388 breakout boards)
  Wire.setSDA(18);
  Wire.setSCL(19);

  // Wait for the hardware to initialize
  bool initialized = false;
  while (!initialized) {
    delay(1000);
    initialized = true;
    if (!gyro.begin()) {
      Serial.println("Error detecting FXAS21002C");
      initialized = false;
    }
    if (!acc_mag.begin(ACCEL_RANGE_8G)) {
      Serial.println("Error detecting FXOS8700");
      initialized = false;
    }
    if (!bmp.begin_I2C(0x77)) {
      Serial.println("Error detecting BMP388");
      initialized = false;
    }
  }

  // Setup XBee
  Serial1.clear();
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(9600);
}

void loop() {
  sensors_event_t gyro_event, accel_event;
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event, NULL);

  Serial.print(millis());
  Serial.print(" - ");
  Serial.print(gyro_event.gyro.x);
  Serial.print(", ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(", ");
  Serial.println(gyro_event.gyro.z);

  Serial1.print(millis());
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.x);
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.y);
  Serial1.print(',');
  Serial1.print(gyro_event.gyro.z);
  Serial1.print("\n");

  delay(1000);
}
