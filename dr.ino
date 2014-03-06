#include "QTRSensors_teensy3.h"
#include "DRV8835_teensy3.h"
#include <Encoder.h>
#include <Pushbutton.h>

const byte NumLineSensors = 3;
const short LineSensorTimeout = 1000;
const short LineSensorMin = 200;
const short LineSensorMax = 1000;

QTRSensorsRC lineSensors((byte[]) {14, 15, 16}, NumLineSensors, LineSensorTimeout);
unsigned short lineSensorValues[NumLineSensors];

DRV8835 driveMotors(7, 5, 8, 6);

Encoder lwEnc(18, 19);
Encoder rwEnc(21, 20);

Pushbutton btn(0);

enum { WaitForButton, FindLine, StartFollowLine, FollowLine, GoHome, Done } state;

void setup()
{
  Serial.begin(115200);
  setLineSensorCalibration();
  driveMotors.init(23437);
  
  state = WaitForButton;
}

void loop()
{
  static unsigned short millisStart = 0;
  
  switch(state)
  {
    case WaitForButton:
    {
      btn.waitForButton();
      delay(1000);
      state = FindLine;
      millisStart = millis();
    }
    break;

    case FindLine:
    {
      short s = (millis() - millisStart) / 4;
      if (s > 255)
        s = 255;
      driveMotors.setSpeeds(s, s);
      lineSensors.readCalibrated(lineSensorValues);
      if (onLine())
      {
        state = StartFollowLine;
        millisStart = millis();
      }
    }
    break;
      
    case StartFollowLine:
    {
      followLine();
    }
    break;
  }
}

void setLineSensorCalibration()
{
  lineSensors.calibrate(); // force allocate calibrated values
  
  for (byte i = 0; i < NumLineSensors; i++)
  {
    lineSensors.calibratedMinimumOn[i] = LineSensorMin;
    lineSensors.calibratedMaximumOn[i] = LineSensorMax;
  }
}

boolean onLine()
{
  return (lineSensorValues[0] > 500) || (lineSensorValues[1] > 500);
}

void followLine()
{
  static unsigned short last_proportional = 0;
  static long integral = 0;
  
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned short position = lineSensors.readLine(lineSensorValues);

  // The "proportional" term should be 0 when we are on the line.
  short proportional = ((int)position) - 1000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  short derivative = proportional - last_proportional;
  integral += proportional;

  // Remember the last position.
  last_proportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  short power_difference = proportional/5;// + derivative*6;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const short max = 255, diff_max = max;
  if(power_difference > diff_max)
    power_difference = diff_max;
  if(power_difference < -diff_max)
    power_difference = -diff_max;

  if(power_difference < 0)
    driveMotors.setSpeeds(max + power_difference, max);
  else
    driveMotors.setSpeeds(max, max - power_difference);
}


