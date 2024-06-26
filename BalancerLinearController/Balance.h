#pragma once

#include <stdint.h>
#include <LSM6.h>
#include <Balboa32U4.h>

const float UPDATE_TIME_MS = 10.0;  // [ms]
const float TICKS_RADIAN = 161.0;   // 12*51.45*41/25
const float BITS = 29000.0;         //±32768.0-> 2**15
const float DPS = 1000;
const float rad2deg = 57.296;        // 180/pi
const int32_t START_BALANCING_ANGLE = 5;     //[degrees]
const int32_t STOP_BALANCING_ANGLE = 70;     //[degrees]
const int16_t DISTANCE_DIFF_RESPONSE = 80;
const uint8_t CALIBRATION_ITERATIONS = 100;  // # measurements to calibrate gyro


// These variables must be defined in your sketch.
extern LSM6 imu;
extern Balboa32U4Motors motors;
extern Balboa32U4Encoders encoders;
extern Balboa32U4ButtonA buttonA;
extern Balboa32U4ButtonB buttonB;
extern Balboa32U4ButtonC buttonC;

// Usefull variable
extern float param;

// Call this in your setup() to initialize and calibrate the IMU.
void balanceSetup();

// Call this in loop() to run the full balancing algorithm.
void balanceUpdate();
void balance();
void lyingDown();
void beginBalance();
void integrateGyro();
void integrateEncoders();
void refTrack();
void refTrackRandom();
float generateGaussianNoise(float mean, float std);
void avoidOscillations();
// Sometimes you will want to take control of the motors but keep
// updating the balancing code's encoders and angle measurements
// so you don't lose track of the robot's position and angle.
// Call this every 10ms (UPDATE_TIME_MS) to update the sensors,
// and you will be able to resume balancing immediately when you
// are done.
void balanceUpdateSensors();

// Call this function to reset the encoders.  This is useful
// after a large motion, so that robot does not try to make a
// huge correction to get back to "zero".
void balanceResetEncoders();
void standUp();
