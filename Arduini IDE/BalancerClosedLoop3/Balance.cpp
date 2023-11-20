#include <Wire.h>
#include "Balance.h"

int32_t gYZero;
int32_t angle;
int32_t angleRate;
int32_t distanceLeft;
float speedLeft;
int32_t driveLeft;
int32_t distanceRight;
float speedRight;
int32_t driveRight;
int16_t motorSpeed;
bool isBalancingStatus = false;
bool balanceUpdateDelayedStatus;

bool isBalancing()
{
  return isBalancingStatus;
}

bool balanceUpdateDelayed()
{
  return balanceUpdateDelayedStatus;
}

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
}

// This function contains the core algorithm for balancing a
// Balboa 32U4 robot.
void balance()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  angle = angle * 999 / 1000;

  // This variable measures how close we are to our basic
  // balancing goal - being on a trajectory that would cause us
  // to rise up to the vertical position with zero speed left at
  // the top.  This is similar to the fallingAngleOffset used
  // for LED feedback and a calibration procedure discussed at
  // the end of Balancer.ino.
  //
  // It is in units of millidegrees, like the angle variable, and
  // you can think of it as an angular estimate of how far off we
  // are from being balanced.
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  // Combine risingAngleOffset with the distance and speed
  // variables, using the calibration constants defined in
  // Balance.h, to get our motor response.  Rather than becoming
  // the new motor speed setting, the response is an amount that
  // is added to the motor speeds, since a *change* in speed is
  // what causes the robot to tilt one way or the other.


  // Cst to try to find good range of speed
  int32_t spdFct = 1;

  // Controller for left wheel
  float phiLeft       = distanceLeft / R;        //[Rad] = [mm] / [mm]                  -> Measured range: [-3,3]
  float thetaLeft     = angle / 5730;            //[Rad] = [100deg] / [100deg/Rad]      -> Measured range: [-1,1]
  float phi_dotLeft   = speedLeft / R;           //[rad/s] = [mm/s] / [mm]              -> Measured range: [-20,20]
  float theta_dotLeft = angleRate / 57.296;      //[rad/s] = [deg/s] / [deg/Rad]        -> Measured range: [-4,4] 

  float TorqueLeft = 1.3 * phiLeft - 4.6 * thetaLeft - 0.15 * phi_dotLeft - 0.23 * theta_dotLeft; // Not sure about those params -> Measured range: [-0.7,0.7]
  
  int32_t motorSpeedLeft = -round(spdFct * Ra/K_tau / alpha * (TorqueLeft + K_tau/Ra*K * speedLeft) ); // With spdFct = 1 -> -> Measured range: [-5,5]

  // Controller for right wheel
  float phiRight       = distanceRight / R;      //[Rad] = [mm] / [mm]                  -> Measured range: [-3,3]
  float thetaRight     = angle / 5730;           //[Rad] = [100deg] / [100deg/Rad]      -> Measured range: [-1,1]
  float phi_dotRight   = speedRight / R;         //[rad/s] = [mm/s] / [mm]              -> Measured range: [-20,20]
  float theta_dotRight = angleRate / 57.296;     //[rad/s] = [deg/s] / [deg/Rad]        -> Measured range: [-4,4] 

  float TorqueRight = 1.3 * phiRight - 4.6 * thetaRight - 0.15 * phi_dotRight - 0.23 * theta_dotRight; // Not sure about those params -> Measured range: [-0.7,0.7]
  
  int32_t motorSpeedRight = -round(spdFct * Ra/K_tau / alpha * (TorqueRight + K_tau/Ra*K * speedRight) ); // With spdFct = 1 -> -> Measured range: [-5,5]

  char report1[300];
  snprintf(report1, sizeof(report1), "motorSpeedLeft : %6d",
    round(motorSpeedLeft));
  Serial.println(report1);

  if (motorSpeedLeft > MOTOR_SPEED_LIMIT)
  {
    motorSpeedLeft = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeedLeft < -MOTOR_SPEED_LIMIT)
  {
    motorSpeedLeft = -MOTOR_SPEED_LIMIT;
  }

    if (motorSpeedRight > MOTOR_SPEED_LIMIT)
  {
    motorSpeedRight = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeedRight < -MOTOR_SPEED_LIMIT)
  {
    motorSpeedRight = -MOTOR_SPEED_LIMIT;
  }

  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  int16_t distanceDiff = distanceLeft - distanceRight;

  // Need for different speeds for the 2 wheels as they do not exert 
  // the same power depending on the direction of rotation
  motors.setSpeeds(
    motorSpeedLeft + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeedRight - distanceDiff * DISTANCE_DIFF_RESPONSE / 100);

}

void lyingDown()
{
  // Reset things so it doesn't go crazy.
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);

  if (angleRate > -2 && angleRate < 2)
  {
    // It's really calm, so use the accelerometer to measure the
    // robot's rest angle.  The atan2 function returns a result
    // in radians, so we multiply it by 18000/pi to convert it
    // to millidegrees.
    angle = atan2(imu.a.z, imu.a.x) * 5730; // [100deg] -> is rounded to stay in integer

    distanceLeft = 0;
    distanceRight = 0;
  }
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/ms to deg/s.
  angleRate = (imu.g.y - gYZero) / 29; // [1000deg/ms]

  angle += angleRate * UPDATE_TIME_MS/10; // [100deg] = [1000deg/ms] * [ms] / 10 -> can't use [1000deg] otherwise overflow problems
}

void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = round(encoders.getCountsLeft()/4.03);  // [mm]
  speedLeft = (countsLeft - lastCountsLeft)*100;   // [mm/s] -> frequency of 100
  distanceLeft += countsLeft - lastCountsLeft; //[mm]
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = round(encoders.getCountsRight()/4.03); //[mm]
  speedRight = (countsRight - lastCountsRight)*100; //[mm/s] -> frequency of 100
  distanceRight += countsRight - lastCountsRight; //[mm]
  lastCountsRight = countsRight;
}

void balanceDrive(int16_t leftSpeed, int16_t rightSpeed)
{
  driveLeft = leftSpeed;
  driveRight = rightSpeed;
}

void balanceDoDriveTicks()
{
  distanceLeft -= driveLeft;
  distanceRight -= driveRight;
  speedLeft -= driveLeft;
  speedRight -= driveRight;
}

void balanceResetEncoders()
{
  distanceLeft = 0;
  distanceRight = 0;
}

void balanceUpdateSensors()
{
  imu.read();
  integrateGyro();
  integrateEncoders();
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  balanceUpdateSensors();
  balanceDoDriveTicks();

  if (isBalancingStatus)
  {
    balance();

    // Stop trying to balance if we have been farther from
    // vertical than STOP_BALANCING_ANGLE for 5 counts.
    if (abs(angle) > STOP_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = false;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
  else
  {
    lyingDown();

    // Start trying to balance if we have been closer to
    // vertical than START_BALANCING_ANGLE for 5 counts.
    if (abs(angle) < START_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = true;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
}
