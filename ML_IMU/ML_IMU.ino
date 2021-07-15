//Libraries Definition
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Wire.h>
#include "model.h"


//Constants Definition
#define NUM_SAMPLES 10
#define NUM_AXES 3
#define TRUNCATE 20
#define ACCEL_THRESHOLD 5
#define INTERVAL 60

//variables definition
float baseline[NUM_AXES];
float features[NUM_SAMPLES * NUM_AXES];
Adafruit_LSM6DS33 imu;
Eloquent::ML::Port::SVM clf;

void initIMU() {
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DS33 test!");

  if (!imu.begin_I2C()) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
    // if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Adafruit LSM6DS33 FOUND!");
  // lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (imu.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  // lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (imu.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break;  // unsupported range for the DS33
  }

  // lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (imu.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  // lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (imu.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  imu.configInt1(false, false, true);  // accelerometer DRDY on INT1
  imu.configInt2(false, true, false);  // gyro DRDY on INT2
}


void imu_read(float *ax, float *ay, float *az) {
  int16_t _ax, _ay, _az, _gx, _gy, _gz;
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  *ax = a.acceleration.x;
  *ay = a.acceleration.y;
  *az = a.acceleration.z;
}

void setup() {
  Serial.begin(115200);
  delay(10);
  initIMU();
  calibrate();
}

void loop() {
  float ax, ay, az;
  imu_read(&ax, &ay, &az);
  ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
  ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
  az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);

  if (!motionDetected(ax, ay, az)) {
    delay(10);
    return;
  }
  recordIMU();
  printFeatures();
  // un-comment to run classification
  // classify();
  delay(2000);
}

void calibrate() {
  Serial.println("Callibrating.....miso....");
  float ax, ay, az;
  for (int i = 0; i < 10; i++) {
    imu_read(&ax, &ay, &az);
    delay(100);
  }
  baseline[0] = ax;
  baseline[1] = ay;
  baseline[2] = az;
  Serial.println("ok....ksekina");
}


/**
 * Detect if motion is happening
 * @return
 */
bool motionDetected(float ax, float ay, float az) {
  return (abs(ax) + abs(ay) + abs(az)) > ACCEL_THRESHOLD;
}

/**
 * Fill the feature vector
 */
void recordIMU() {
  float ax, ay, az;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    imu_read(&ax, &ay, &az);

    ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
    ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
    az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);

    features[i * NUM_AXES + 0] = ax;
    features[i * NUM_AXES + 1] = ay;
    features[i * NUM_AXES + 2] = az;

    delay(INTERVAL);
  }
}

/**
 * Dump the feature vector to Serial monitor
 */
void printFeatures() {
  const uint16_t numFeatures = sizeof(features) / sizeof(float);

  for (int i = 0; i < numFeatures; i++) {
    Serial.print(features[i]);
    Serial.print(i == numFeatures - 1 ? '\n' : ',');
  }
}


void classify() {
  Serial.print("Detected gesture: ");
  Serial.println(clf.predictLabel(features));
}
