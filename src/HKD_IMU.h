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

class IMU {
public:
  IMU();

  LSM6DSM IntegratedIMU; // Create IMU object from Deneyap_6EksenAtaletselOlcumBirimi.h

  const float g = 9.80665; // Gravitational acceleration [m/s^2];

  float *gyroPRY = new float[3]; // Gyro values //? P: Pitch, R: Roll, Y: Yaw
  float *gyroErrorPRY = new float[3]; // Gyro error values
  float *accel = new float[3];   // Acceleration values //? X: East-West, Y:
                                 // North-South, Z: Up-Down
  float *angleRP = new float[2]; // Angle values //? P: Pitch, R: Roll

  //? Runs once
  void startIMU(int);
  void calibrateGyro(int);

  //? Can run indefinitely
  void calculateAngle();
  void readValues();
  void plotValuesToThePlotter();
};

#endif