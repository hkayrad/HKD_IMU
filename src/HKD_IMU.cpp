/************************************************************************
                                                                        Source
Code Form License Notice
                                                -------------------------------------------

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.

If it is not possible or desirable to put the notice in a particular
file, then You may include the notice in a location (such as a LICENSE
file in a relevant directory) where a recipient would be likely to look
for such a notice.
*************************************************************************/

//* EXTERNAL LIBRARIES
#include "HKD_IMU.h"
#include <Arduino.h>

IMU::IMU() = default;
KalmanFilter kalmanFilter;

void IMU::startIMU(int calibrationDelay) {
  while (IntegratedIMU.begin() != IMU_SUCCESS) { // Check if IMU is connected
    Serial.println("IMU connection failed");
    delay(500);
  }

  calibrateGyro(calibrationDelay);    // Calibrate gyro
  initMagnetometer(calibrationDelay); // Initialize magnetometer

  telemetryData.anglePRY[2] = 0; // Set yaw angle to 0
}

void IMU::initMagnetometer(int calibrationDelay) {
  while (!Magnetometer.begin(0x60)) {
    Serial.println("Magnetometer connection failed");
    delay(calibrationDelay);
  }
}

void IMU::calibrateGyro(int calibrationDelay) {

  Serial.println("Gyro calibration will start please don't touch the plane");
  delay(calibrationDelay);
  Serial.println("Gyro calibration started");

  int gyroCalibrationIteration = 0;

  for (int i = 0; i < 3; i++) { // Set gyro error values to 0
    telemetryData.gyroErrorPRY[i] = 0;
  }

  for (gyroCalibrationIteration = 0; gyroCalibrationIteration < 2000;
       gyroCalibrationIteration++) {
    float *allAxesFloatData = new float[7]; // All axes float values
    IntegratedIMU.readAllAxesFloatData(allAxesFloatData);
    for (int i = 0; i < 3;
         i++) { // Assign values to their corresponding variables
      telemetryData.gyroErrorPRY[i] += allAxesFloatData[i];
    }
    delete[] allAxesFloatData; // Free the memory of allAxes
    delay(1);
  }

  for (int i = 0; i < 3; i++) { // Calculate average gyro error values
    telemetryData.gyroErrorPRY[i] /= gyroCalibrationIteration;
  }

  Serial.println("Gyro calibration finished");
}

void IMU::calculateAngle() {
  telemetryData.anglePRY[0] =
      atan(telemetryData.accelG[1] / (sqrt(telemetryData.accelG[0] * telemetryData.accelG[0] + telemetryData.accelG[2] * telemetryData.accelG[2]))) *
      180 / PI;
  telemetryData.anglePRY[1] =
      -atan(telemetryData.accelG[0] / (sqrt(telemetryData.accelG[1] * telemetryData.accelG[1] + telemetryData.accelG[2] * telemetryData.accelG[2]))) *
      180 / PI;
  /*
    mag_x = magReadX*cos(pitch) + magReadY*sin(roll)*sin(pitch) +
    magReadZ*cos(roll)*sin(pitch)
    mag_y = magReadY * cos(roll) - magReadZ * sin(roll)
    yaw = 180 * atan2(-mag_y,mag_x)/M_PI;
  */

  // Magnetometer
  Magnetometer.RegRead();
  telemetryData.magnetometer[0] = Magnetometer.readMagnetometerX();
  telemetryData.magnetometer[1] = Magnetometer.readMagnetometerY();
  telemetryData.magnetometer[2] = Magnetometer.readMagnetometerZ();

  // Yaw
  //* D = arctan(yGaussData/xGaussData)∗(180/π)
  /*
  Direction (y>0) = 90 - [arcTAN(x/y)]*180/π
  Direction (y<0) = 270 - [arcTAN(x/y)]*180/π
  Direction (y=0, x<0) = 180.0
  Direction (y=0, x>0) = 0.0
  */
  if (telemetryData.magnetometer[1] > 0) {
    telemetryData.anglePRY[2] = 90 - atan(telemetryData.magnetometer[0] / telemetryData.magnetometer[1]) * 180 / PI;
  } else if (telemetryData.magnetometer[1] < 0) {
    telemetryData.anglePRY[2] =
        270 - atan(telemetryData.magnetometer[0] / telemetryData.magnetometer[1]) * 180 / PI;
  } else if (telemetryData.magnetometer[1] == 0 && telemetryData.magnetometer[0] < 0) {
    telemetryData.anglePRY[2] = 180;
  } else if (telemetryData.magnetometer[1] == 0 && telemetryData.magnetometer[0] > 0) {
    telemetryData.anglePRY[2] = 0;
  }

  // anglePRY[2] = 180 * atan2(magnetometerXYZ[1], magnetometerXYZ[0]) / PI;
}

void IMU::readValues() {
  telemetryData.tZero = millis();
  // Gyro & Accelerometer
  float *allAxesFloatData = new float[7]; // All axes float values
  IntegratedIMU.readAllAxesFloatData(allAxesFloatData);
  for (int i = 0; i < 3;
       i++) { // Assign values to their corresponding variables
    telemetryData.gyroPRY[i] = ((allAxesFloatData[i] - //! Primitive error filtering
                   telemetryData.gyroErrorPRY[i]) > 1 ||
                  (allAxesFloatData[i] - // TODO: Change this filtering with a
                                         // kalman filter
                   telemetryData.gyroErrorPRY[i]) < -1)
                     ? allAxesFloatData[i] - telemetryData.gyroErrorPRY[i]
                     : 0;
    telemetryData.accelG[i] = (allAxesFloatData[i + 3]); // Error correction
    telemetryData.accelMps2[i] = telemetryData.accelG[i] * g;          // Convert acceleration to m/s^2
  }

  calculateAngle();

  kalmanFilter.kalman_1d(kalmanFilter.KalmanAnglePR[0],
                         kalmanFilter.KalmanUncertainityAnglePR[0], telemetryData.gyroPRY[0],
                         telemetryData.anglePRY[0]);
  kalmanFilter.KalmanAnglePR[0] = kalmanFilter.Kalman1DOutput[0];
  kalmanFilter.KalmanUncertainityAnglePR[0] = kalmanFilter.Kalman1DOutput[1];

  kalmanFilter.kalman_1d(kalmanFilter.KalmanAnglePR[1],
                         kalmanFilter.KalmanUncertainityAnglePR[1], telemetryData.gyroPRY[1],
                         telemetryData.anglePRY[1]);
  kalmanFilter.KalmanAnglePR[1] = kalmanFilter.Kalman1DOutput[0];
  kalmanFilter.KalmanUncertainityAnglePR[1] = kalmanFilter.Kalman1DOutput[1];

  delete[] allAxesFloatData; // Free the memory of allAxes
}

void IMU::plotValuesToThePlotter() { // Plot values to the plotter
  Serial.println(">Gyro Pitch [°/s]: " + String(telemetryData.gyroPRY[0]));
  Serial.println(">Gyro Roll [°/s]: " + String(telemetryData.gyroPRY[1]));
  Serial.println(">Gyro Yaw [°/s]: " + String(telemetryData.gyroPRY[2]));
  Serial.println(">Accel X [m/s^2]: " + String(telemetryData.accelMps2[0]));
  Serial.println(">Accel Y [m/s^2]: " + String(telemetryData.accelMps2[1]));
  Serial.println(">Accel Z [m/s^2]: " + String(telemetryData.accelMps2[2]));
  Serial.println(">Angle Pitch [°]: " + String(telemetryData.anglePRY[0]));
  Serial.println(">Angle Roll [°]: " + String(telemetryData.anglePRY[1]));
  Serial.println(">Angle Yaw [°]: " + String(telemetryData.anglePRY[2]));
  Serial.println(">Kalman Angle Pitch [°]: " +
                 String(kalmanFilter.KalmanAnglePR[0]));
  Serial.println(">Kalman Angle Roll [°]: " +
                 String(kalmanFilter.KalmanAnglePR[1]));
  Serial.println(">Magnetometer X [mG]: " + String(telemetryData.magnetometer[0]));
  Serial.println(">Magnetometer Y [mG]: " + String(telemetryData.magnetometer[1]));
  Serial.println(">Magnetometer Z [mG]: " + String(telemetryData.magnetometer[2]));
}
