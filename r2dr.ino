#include "QTRSensors_teensy3.h"
#include "DRV8835_teensy3.h"
#include <Encoder.h>
#include <Pushbutton.h>

const uint8_t NumLineSensors = 3;
const int16_t LineSensorTimeout = 1000;
const int16_t LineSensorMin = 200;
const int16_t LineSensorMax = 1000;

QTRSensorsRC lineSensors((unsigned char[]) {14, 15, 16}, NumLineSensors, LineSensorTimeout);
uint16_t lineSensorValues[NumLineSensors];

DRV8835 driveMotors(7, 5, 8, 6);
const int16_t MaxSpeed = 200;

Encoder lwEnc(18, 19);
Encoder rwEnc(21, 20);

Pushbutton btn(0);

enum { WaitForButton, FindLine, StartFollowLine, FollowLine, GoHome, Done } state;

int16_t ls, rs;

const uint16_t AngleScale = 20000;
const uint16_t StepsPerRadian = 1000;
const uint16_t LeapTickPerL = 0;
const uint16_t LeapTickPerR = 150;
const int32_t FollowMaxY = 13000000L;
const int16_t FollowMaxS = 15000L;

int16_t c = AngleScale;
int16_t s = 0;
int32_t x = 0, y = 0;

#define sign(x) ((x) < 0 ? -1 : 1)

void followLine(int16_t accelMaxSpeed = MaxSpeed);

void setup()
{
  Serial.begin(115200);
  setLineSensorCalibration();
  driveMotors.init(23437);
  
  pinMode(13, OUTPUT);
  
  state = WaitForButton;
}

void loop()
{
  static uint16_t stateStartMillis = 0;
  static boolean accel = false;
  static uint16_t accelStartMillis = 0;
  static int16_t accelMaxSpeed = 0;
  
  if (state != WaitForButton && state != Done)
    updateWheelEncoders();
  
  if (accel)
  {
    accelMaxSpeed = (millis() - accelStartMillis) / 4; // 0 to 255 over ~1 second
    if (accelMaxSpeed >= MaxSpeed)
    {
      accelMaxSpeed = MaxSpeed;
      accel = false;
    }
  }
  
  switch(state)
  {
    case WaitForButton:
    {
      btn.waitForButton();
      delay(1000);
      lwEnc.write(0);
      rwEnc.write(0);
      state = FindLine;
      accel = true;
      accelStartMillis = millis();
    }
    break;

    case FindLine:
    {
      driveMotors.setSpeeds(accelMaxSpeed, accelMaxSpeed);
        
      lineSensors.readCalibrated(lineSensorValues);
      if (onLine())
      {
        state = StartFollowLine;
        stateStartMillis = millis();
      }
    }
    break;
      
    case StartFollowLine:
    {
      followLine(accelMaxSpeed);
      if ((millis() - stateStartMillis) > 2000)
      {
        state = FollowLine;
        digitalWrite(13, HIGH);
      }
    }
    break;
    
    case FollowLine:
    {
      followLine();
      if (!onLine())
      {
        printDebug();
        Serial.println("end of line");
        
        transform();
        printDebug();
        
        state = GoHome;
        stateStartMillis = millis();
        digitalWrite(13, LOW);
      }
    }
    break;
    
    case GoHome:
    {
      goHome();
      if(x > -1000000)
      {
        state = Done;
      }
    }
    break;
    
    case Done:
    {
      driveMotors.setSpeeds(0, 0);
    }
    break;
  }
}

void setLineSensorCalibration()
{
  lineSensors.calibrate(); // force allocate calibrated values
  
  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    lineSensors.calibratedMinimumOn[i] = LineSensorMin;
    lineSensors.calibratedMaximumOn[i] = LineSensorMax;
  }
}

void updateWheelEncoders()
{
  static int8_t prevLCount = 0, prevRCount = 0;
  static uint16_t leapLCount = 0, leapRCount = 0;
  
  int8_t dLCount = (int8_t)(lwEnc.read() - prevLCount);
  int8_t dRCount = (int8_t)(rwEnc.read() - prevRCount);
  
  if (dLCount != 0)
  {
    int16_t dc = + divide(dLCount*s - dLCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = - divide(dLCount*c + dLCount*s/2/StepsPerRadian, StepsPerRadian);
  
    c += dc;
    s += ds;
    x += dLCount * c;
    y += dLCount * s;
    
    prevLCount += dLCount;
    
    if (LeapTickPerL > 0)
    {
      leapLCount += dLCount;
      if (leapLCount >= LeapTickPerL)
      {
        prevLCount -= 1;
        leapLCount -= LeapTickPerL;
      }
    }
  }
  
  if (dRCount != 0)
  {
    int16_t dc = - divide((int32_t)dRCount*s + (int32_t)dRCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = + divide((int32_t)dRCount*c - (int32_t)dRCount*s/2/StepsPerRadian, StepsPerRadian);
    
    c += dc;
    s += ds;
    x += dRCount * c;
    y += dRCount * s;
  
    prevRCount += dRCount;
    
    if (LeapTickPerR > 0)
    {
      leapRCount += dRCount;
      if (leapRCount >= LeapTickPerR)
      {
        prevRCount -= 1;
        leapRCount -= LeapTickPerR;
      }
    }
  }
  
  static uint16_t lastPrint = 0;
  if ((uint16_t)(millis() - lastPrint) > 500)
  {
    printDebug();
    lastPrint = millis();
  }
}

int32_t divide(int32_t a, int32_t b)
{
  return (a + sign(a)*(b/2-1)) / b;
}

void transform()
{
  double r = hypot((double)x, (double)y);
  double nx = (double)x/r; // x = cos(tt)
  double ny = (double)y/r; // y = sin(tt)
  int32_t new_c = -nx*c-ny*s; // -cos(tt)*cos(f)-sin(tt)*sin(f) = -cos(tt-f) = cos(180-(tt+f))
  int32_t new_s = ny*c-nx*s; // sin(tt)*cos(f)-cos(tt)*sin(f) = sin(tt-f) = sin(180 - (tt+f))
  c = new_c;
  s = new_s;
  y = 0;
  x = -r;
}

void goHome()
{
  int16_t speed = MaxSpeed;
  int32_t err;
  
  if(x > -20000000)
    speed = speed/2;
  
  if(c < 0)
  {
    // pointed backwards
    err = (s > 0 ? speed/2 : -speed/2);
  }
  else
  {    
    int32_t target_s = -max(min(y / 10000 * FollowMaxS / (FollowMaxY / 10000), FollowMaxS), -FollowMaxS);
    err = (s - target_s)/100;
    err = max(min(err,speed),-speed);
  }
  if(err > 0)
    driveMotors.setSpeeds(speed, speed - err);
  else
    driveMotors.setSpeeds(speed + err, speed);
}

boolean onLine()
{
  return (lineSensorValues[0] > 500) || (lineSensorValues[1] > 500) || (lineSensorValues[2] > 500);
}

void followLine(int16_t accelMaxSpeed)
{
  static uint16_t last_proportional = 0;
  static long integral = 0;
  static const int16_t lfMaxSpeed = accelMaxSpeed, lfMaxDiff = accelMaxSpeed;
  
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  uint16_t position = lineSensors.readLine(lineSensorValues);

  // The "proportional" term should be 0 when we are on the line.
  int16_t proportional = ((int)position) - 1000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int16_t derivative = proportional - last_proportional;
  integral += proportional;

  // Remember the last position.
  last_proportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  int16_t power_difference = proportional/4 + derivative*4;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if(power_difference > lfMaxDiff)
    power_difference = lfMaxDiff;
  if(power_difference < -lfMaxDiff)
    power_difference = -lfMaxDiff;

  if(power_difference < 0)
  {
    ls = lfMaxSpeed + power_difference;
    rs = lfMaxSpeed;
  }
  else
  {
    ls = lfMaxSpeed;
    rs = lfMaxSpeed - power_difference;
  }
  driveMotors.setSpeeds(ls, rs);
}

void printDebug()
{
  Serial.print(s); Serial.print("\t");
  Serial.print(c); Serial.print("\t");
  Serial.print(hypot(s, c)); Serial.print("\t");
  Serial.print(atan2(s, c) * 180 / PI); Serial.print("\t");
  Serial.print(x); Serial.print("\t");
  Serial.print(y); Serial.print("\t");
  Serial.println();
  
}
