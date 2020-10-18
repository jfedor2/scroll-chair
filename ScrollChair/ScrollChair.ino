// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <bluefruit.h>

#define SENSITIVITY 0.01

// PIN_BUTTON1 is the "user switch" on the Feather nRF52840 Sense
#define RECENTER_BUTTON PIN_BUTTON1

Adafruit_LSM6DS33 lsm6ds;
Adafruit_LIS3MDL lis3mdl;
Adafruit_Sensor_Calibration_SDFat cal;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_NXPSensorFusion filter;
//Adafruit_Madgwick filter;
//Adafruit_Mahony filter;

BLEDis bledis;
BLEHidAdafruit blehid;

// zero position
float pitch0 = 0.0;

float accumulated_tilt = 0.0;

uint32_t prev_millis;

void setup() {
  pinMode(RECENTER_BUTTON, INPUT_PULLUP);

  // assuming we followed the magnetometer calibration procedure at
  // https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions
  cal.begin();
  cal.loadCalibration();

  lsm6ds.begin_I2C();
  lis3mdl.begin_I2C();
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  filter.begin(66); // Hz

  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.begin();
  Bluefruit.setTxPower(8);
  Bluefruit.setName("Scroll Chair");

  // Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Feather Sense");
  bledis.begin();

  blehid.begin();

  Bluefruit.Periph.setConnInterval(12, 12); // 12*1.25ms=15ms

  startAdvertising();

  Wire.setClock(400000);
}

void startAdvertising(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop() {
  // we target 66 Hz
  if ((millis() - prev_millis) < 15) {
    return;
  }
  prev_millis = millis();

  sensors_event_t accel, gyro, mag;

  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz,
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float pitch = filter.getPitch();

  if (digitalRead(RECENTER_BUTTON) == LOW) {
    pitch0 = pitch;
  }

  accumulated_tilt += (pitch - pitch0) * SENSITIVITY;
  accumulated_tilt = max(0, accumulated_tilt);

  if (accumulated_tilt >= 1.0) {
    int integer_tilt = floor(accumulated_tilt);
    blehid.mouseScroll(-integer_tilt);
    accumulated_tilt -= integer_tilt;
  }
}
