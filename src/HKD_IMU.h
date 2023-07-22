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

#ifndef HKD_IMU_H
#define HKD_IMU_H

//* EXTERNAL LIBRARIES
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <Deneyap_9EksenAtaletselOlcumBirimi.h>

//* LOCAL LIBRARIES
#include "HKD_KalmanFilter.h"

class IMU {
public:
  IMU();

  LSM6DSM IntegratedIMU; // Create IMU object from
                         // Deneyap_6EksenAtaletselOlcumBirimi.h
  MAGNETOMETER Magnetometer; // Create Magnetometer object from
                             // Deneyap_9EksenAtaletselOlcumBirimi.h

  float *gyroPRY =
      new float[3]; // Gyro values //? P: Pitch, R: Roll, Y: Yaw, Units: deg/s
  float *gyroErrorPRY = new float[3]; // Gyro error values
  float *accelG = new float[3];    // Acceleration values //? X: East-West, Y:
                                   //? North-South, Z: Up-Down, Units: g
  float *accelMps2 = new float[3]; //? Units: m/s^2
  float *anglePRY =
      new float[3]; // Angle values //? P: Pitch, R: Roll, Units: deg
  int *magnetometerXYZ = new int[3]; // Magnetometer values
  unsigned long tZero;

  //? Runs once
  void startIMU(int);

  //? Can run indefinitely
  void readValues();
  void plotValuesToThePlotter();

private:
  void initMagnetometer(int);
  void calibrateGyro(int);
  void calculateAngle();
  const float g = 9.80665; // Gravitational acceleration [m/s^2];
};

#endif