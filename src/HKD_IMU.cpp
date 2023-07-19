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

  calibrateGyro(calibrationDelay); // Calibrate gyro
}

void IMU::calibrateGyro(int calibrationDelay) {

  Serial.println("Gyro calibration will start please don't touch the plane");
  delay(calibrationDelay);
  Serial.println("Gyro calibration started");

  int gyroCalibrationIteration;

  for (int i = 0; i < 3; i++) { // Set gyro error values to 0
    gyroErrorPRY[i] = 0;
  }

  for (gyroCalibrationIteration = 0; gyroCalibrationIteration < 2000;
       gyroCalibrationIteration++) {
    float *allAxesFloatData = new float[7]; // All axes float values
    IntegratedIMU.readAllAxesFloatData(allAxesFloatData);
    for (int i = 0; i < 3;
         i++) { // Assign values to their corresponding variables
      gyroErrorPRY[i] += allAxesFloatData[i];
    }
    delete[] allAxesFloatData; // Free the memory of allAxes
    delay(1);
  }

  for (int i = 0; i < 3; i++) { // Calculate average gyro error values
    gyroErrorPRY[i] /= gyroCalibrationIteration;
  }

  Serial.println("Gyro calibration finished");
}

void IMU::calculateAngle() {
  anglePR[0] =
      atan(accelG[1] / (sqrt(accelG[0] * accelG[0] + accelG[2] * accelG[2]))) * 180 /
      PI;
  anglePR[1] =
      -atan(accelG[0] / (sqrt(accelG[1] * accelG[1] + accelG[2] * accelG[2]))) *
      180 / PI;
}

void IMU::readValues() {
  float *allAxesFloatData = new float[7]; // All axes float values
  IntegratedIMU.readAllAxesFloatData(allAxesFloatData);
  for (int i = 0; i < 3;
       i++) { // Assign values to their corresponding variables
    gyroPRY[i] = ((allAxesFloatData[i] - //! Primitive error filtering
                   gyroErrorPRY[i]) > 0.25 ||
                  (allAxesFloatData[i] - // TODO: Change this filtering with a
                                         // kalman filter
                   gyroErrorPRY[i]) < -0.25)
                     ? allAxesFloatData[i] - gyroErrorPRY[i]
                     : 0;
    accelG[i] = (allAxesFloatData[i + 3]); // Error correction
    accelMps2[i] = accelG[i] * g;          // Convert acceleration to m/s^2
  }

  calculateAngle();

  kalmanFilter.kalman_1d(kalmanFilter.KalmanAnglePR[0],
                         kalmanFilter.KalmanUncertainityAnglePR[0], gyroPRY[0],
                         anglePR[0]);
  kalmanFilter.KalmanAnglePR[0] = kalmanFilter.Kalman1DOutput[0];
  kalmanFilter.KalmanUncertainityAnglePR[0] = kalmanFilter.Kalman1DOutput[1];

  kalmanFilter.kalman_1d(kalmanFilter.KalmanAnglePR[1],
                         kalmanFilter.KalmanUncertainityAnglePR[1], gyroPRY[1],
                         anglePR[1]);
  kalmanFilter.KalmanAnglePR[1] = kalmanFilter.Kalman1DOutput[0];
  kalmanFilter.KalmanUncertainityAnglePR[1] = kalmanFilter.Kalman1DOutput[1];

  delete[] allAxesFloatData; // Free the memory of allAxes
}

void IMU::plotValuesToThePlotter() { // Plot values to the plotter
  Serial.println(">Gyro Pitch [°/s]: " + String(gyroPRY[0]));
  Serial.println(">Gyro Roll [°/s]: " + String(gyroPRY[1]));
  Serial.println(">Gyro Yaw [°/s]: " + String(gyroPRY[2]));
  Serial.println(">Accel X [m/s^2]: " + String(accelMps2[0]));
  Serial.println(">Accel Y [m/s^2]: " + String(accelMps2[1]));
  Serial.println(">Accel Z [m/s^2]: " + String(accelMps2[2]));
  Serial.println(">Angle Pitch [°]: " + String(anglePR[0]));
  Serial.println(">Angle Roll [°]: " + String(anglePR[1]));
  Serial.println(">Kalman Angle Pitch [°]: " +
                 String(kalmanFilter.KalmanAnglePR[0]));
  Serial.println(">Kalman Angle Roll [°]: " +
                 String(kalmanFilter.KalmanAnglePR[1]));
}
