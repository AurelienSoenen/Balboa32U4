#include <Wire.h>
#include "Balance.h"
#include <math.h>

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

int32_t gYZero;
float phi;
float phiLeft;
float phiRight;
float phiDif;
float phiDot;
float theta;
float thetaDot;
int16_t motorSpeed;

bool isBalancingStatus = false;
float ref;
int32_t startBalancingTime;
int32_t oscillationTime;

void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    delay(200);
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

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  lastMillis = ms;

  if (isBalancingStatus)
  {
    balanceUpdateSensors();
    balance();

    // Stop trying to balance if we have been farther from
    // vertical than STOP_BALANCING_ANGLE for 5 counts.
    if (abs(theta) > STOP_BALANCING_ANGLE/rad2deg)
    {
      if (++count > 5)
      {
        isBalancingStatus = false;
        count = 0;
        motors.setSpeeds(0, 0);
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
    if (abs(theta) < START_BALANCING_ANGLE/rad2deg)
    {
      if (++count > 5)
      {
        count = 0;
        beginBalance();
      }
    }
    else
    {
      count = 0;
    }
  }
}

void balanceUpdateSensors()
{
  integrateGyro();
  integrateEncoders();
}

void integrateGyro()
{
  imu.read();
  thetaDot = (imu.g.y - gYZero) / BITS * DPS  / rad2deg;               // [Rad/s]
  theta += thetaDot * UPDATE_TIME_MS / 1000;                           // [Rad]
}

void integrateEncoders()
{
  static float lastCountsLeft, phiDotLeft;
  float countsLeft = encoders.getCountsLeft()/TICKS_RADIAN;           // [Rad]
  phiDotLeft = (countsLeft - lastCountsLeft)*1000/UPDATE_TIME_MS;     // [Rad/s]
  phiLeft += countsLeft - lastCountsLeft;                             // Trick to reset the starting position
  lastCountsLeft = countsLeft;

  static float lastCountsRight, phiDotRight;
  float countsRight = encoders.getCountsRight()/TICKS_RADIAN;         // [Rad]
  phiDotRight = (countsRight - lastCountsRight)*1000/UPDATE_TIME_MS;  // [Rad/s]
  phiRight += countsRight - lastCountsRight;                          // Trick to reset the starting position
  lastCountsRight = countsRight;

  phi = (phiLeft + phiRight)/2.0;
  phiDif = phiLeft - phiRight;
  phiDot = (phiDotLeft + phiDotRight)/2.0;
}


void lyingDown()             // Setup when lying down
{
  if (abs(thetaDot) < 0.05)  // use accelerometer for angle measurement when calm
  {
    imu.read();
    theta = atan2(imu.a.z, imu.a.x);                                   // [Rad]
    //thetaDot = (imu.g.y - gYZero) / BITS * DPS  / rad2deg;            // [Rad/s]
  }
  else{
    integrateGyro();
  }
}

void beginBalance()          // Setup before balancing
{
  balanceResetEncoders();
  startBalancingTime = millis();
  oscillationTime = millis();
  isBalancingStatus = true;
}

void balanceResetEncoders()  // Reset the starting position
{
  phiLeft = 0;
  phiRight = 0;
}

void balance()
{
  theta = theta * 999 / 1000;

  float K1;       
  float K2;     
  float K3;     
  float K4;
  
  float ts = 0 + param;
  float ua;
  if (ts == 0.0){
    K1 = 0.09;
    K2 = 3.50;
    K3 = 0.21;
    K4 = 0.32;

    refTrack();
    ua =            
      + K1 * (phi - ref)       
      + K2 * theta           
      + K3 * phiDot            
      + K4 * thetaDot;  
  }
  if (ts == 1.0){
    K1 = 0.338;
    K2 = 5.373;
    K3 = 0.296;
    K4 = 0.469;

    ua =            
    + K1 * phi       
    + K2 * theta           
    + K3 * phiDot            
    + K4 * thetaDot;  
    
    refTrackRandom();
    ua = ua+ref;
  }
  if (ts == 2.0){
    K1 = 0.067;
    K2 = 2.853;
    K3 = 0.221;
    K4 = 0.230;

    ua =            
    + K1 * phi       
    + K2 * theta           
    + K3 * phiDot            
    + K4 * thetaDot;  
  }

  float offset = 0.4;
  if (ua>0) ua += offset; 
  else      ua -= offset; 
  motorSpeed = 400/(readBatteryMillivolts()/1000.0) * ua;

  avoidOscillations();

  motors.setSpeeds(
    motorSpeed - phiDif * DISTANCE_DIFF_RESPONSE,
    motorSpeed + phiDif * DISTANCE_DIFF_RESPONSE);

  // Print all parameters on Serial Port
  Serial.print(millis());
  Serial.print(","); 
  Serial.print(phi);
  Serial.print(","); 
  Serial.print(theta);
  Serial.print(","); 
  Serial.print(phiDot);
  Serial.print(","); 
  Serial.print(thetaDot);
  Serial.print(","); 
  Serial.print(motorSpeed);
  Serial.print(","); 
  Serial.print(ref);
  Serial.print(","); 
  Serial.println(ts);   
}


// This function contains the core algorithm for balancing a
// Balboa 32U4 robot.
void refTrack()
{
  float R  = 40.0;    // [mm]

  int32_t stabilisingTime = 7500;      // [ms]
  int32_t distance = 350;              // [mm]
  float speed = 350.0;        // [mm/s]
  int32_t startRefTrack1 = stabilisingTime;     // [ms]

  int32_t pauseRefTrack = startRefTrack1 + distance/speed*1000;           // [ms]
  int32_t startRefTrack2 = pauseRefTrack + stabilisingTime;          // [ms]
  int32_t endRefTrack = startRefTrack2 + distance/speed*1000;             // [ms] 

  int32_t time = millis() - startBalancingTime;      // [ms]
  if (time < startRefTrack1){
    ref = 0;        // [rad]
  }
  else if (time >  startRefTrack1 && time <= pauseRefTrack){
    int32_t t = time - startRefTrack1;
    ref = speed / R * t/1000.0;
  }
  else if (time >  pauseRefTrack && time <= startRefTrack2){
    ref = distance / R;
  }
  else if (time >  startRefTrack2 && time <= endRefTrack){
    int32_t t = time - startRefTrack2;
    ref = distance / R - speed / R * t/1000.0;
  } 
  else if (time > endRefTrack){
    ref = 0;        // [rad]
  }
}
// Define a function to generate Gaussian random numbers
float generateGaussianNoise(float mean, float std) {
    static bool hasSpare = false;
    static float spare;

    if(hasSpare) {
        hasSpare = false;
        return mean + std * spare;
    }

    hasSpare = true;
    float u, v, s;
    do {
        u = (random(10001) / 10000.0) * 2.0 - 1.0;
        v = (random(10001) / 10000.0) * 2.0 - 1.0;
        s = u * u + v * v;
    } while(s >= 1.0 || s == 0);

    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    return mean + std * u * s;
}

void refTrackRandom() {

  int32_t stabilisingTime = 5000;      // [ms]

  int32_t time = millis() - startBalancingTime;      // [ms]
  if (time < stabilisingTime){
    ref = 0;        // [rad]
  }
  else{
    float std = 0.5; // Standard deviation for Gaussian noise
    ref = generateGaussianNoise(0, std); // Continuously generate random ref value
  }
}
void avoidOscillations(){
  static int32_t lastDirection = 0; // bool of Ms is positif or negatif
  int32_t presentDirection = 0;
  if (motorSpeed >= 0){presentDirection = 1;}
  
  if (presentDirection != lastDirection){
    if (millis() - oscillationTime<150){
      
      motorSpeed = 0;
    }
    else{
      oscillationTime = millis();
      lastDirection = presentDirection; 
    }
  }
}

void standUp()
{
  uint16_t startTime = millis();
  uint16_t ms = millis();
  while((uint16_t)(ms-startTime) < 100*UPDATE_TIME_MS){

    static uint16_t lastMillis;
    while ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) {ms = millis();}
    lastMillis = ms;

    if ( (uint16_t)(ms-startTime) < 20*UPDATE_TIME_MS ){
      motorSpeed = -300+75;
    }
    else {
      motorSpeed = 150;
    }


    balanceUpdateSensors();
    float ts = 0;
    Serial.print(millis());
    Serial.print(","); 
    Serial.print(phi);
    Serial.print(","); 
    Serial.print(theta);
    Serial.print(","); 
    Serial.print(phiDot);
    Serial.print(","); 
    Serial.print(thetaDot);
    Serial.print(","); 
    Serial.print(motorSpeed);
    Serial.print(","); 
    Serial.print(ref);
    Serial.print(","); 
    Serial.println(ts); 

    motors.setSpeeds(
      motorSpeed - phiDif * DISTANCE_DIFF_RESPONSE,
      motorSpeed + phiDif * DISTANCE_DIFF_RESPONSE);

    if(abs(theta) < 30/rad2deg)
    {
      beginBalance();
      break;
    }
  }
  if ((ms-startTime) >= 100*UPDATE_TIME_MS)
  {motors.setSpeeds(0, 0);}
}


