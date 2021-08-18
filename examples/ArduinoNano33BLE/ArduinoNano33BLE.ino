#include <Arduino_LSM9DS1.h>
#include "MadgwickAHRS.h"

// initialize a Madgwick filter:
Madgwick filter;
// sensor's sample rate is fixed at 104 Hz:
const float sensorRate = 104.00;
// float min1,min2;

void setup() {
  Serial.begin(9600);
  // attempt to start the IMU:
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }
  // start the filter to run at the sample rate:
  // min1=min(IMU.gyroscopeSampleRate(),IMU.accelerationSampleRate());
  // min2=min(min1,IMU.magneticFieldSampleRate());
  filter.begin(sensorRate);

}

void loop() {
  // values for acceleration and rotation:
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
  float xMag, yMag, zMag;
  // values for orientation:
  float roll, pitch, heading;
  
  // check if the IMU is ready to read:
  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    // read accelerometer &and gyrometer:
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readGyroscope(xGyro, yGyro, zGyro);
    IMU.readMagneticField(xMag, yMag, zMag);

    // update the filter, which computes orientation:
//    filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
    filter.update(yGyro, xGyro, -zGyro, yAcc, xAcc, -zAcc, yMag, -xMag, -zMag);
// discussedd in https://forum.arduino.cc/t/ble-sense-ahrs/636949 

    // print the heading, pitch and roll
    // Serial.println(min2);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
//    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
  }
}
