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

short ls, rs;

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
  static unsigned short millisInitial = 0;
  static short lsInitial, rsInitial;
  
  switch(state)
  {
    case WaitForButton:
    {
      btn.waitForButton();
      delay(1000);
      state = FindLine;
      millisInitial = millis();
    }
    break;

    case FindLine:
    {
      short s = (millis() - millisInitial) / 4;
      if (s <= 255)
        driveMotors.setSpeeds(s, s);
        
      lineSensors.readCalibrated(lineSensorValues);
      if (onLine())
      {
        state = StartFollowLine;
        millisInitial = millis();
      }
    }
    break;
      
    case StartFollowLine:
    {
      followLine();
      if ((millis() - millisInitial) > 2000)
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
        state = GoHome;
        millisInitial = millis();
        lsInitial = ls;
        rsInitial = rs;
        digitalWrite(13, LOW);
      }
    }
    break;
    
    case GoHome:
    {
      // decel by subtracting 1 from original speeds every 4 ms
      short ds = (millis() - millisInitial) / 4;
      
      ls = lsInitial - ds;
      if (ls < 0)
        ls = 0;
        
      rs = rsInitial - ds;
      if (rs < 0)
        rs = 0;
        
      driveMotors.setSpeeds(ls, rs);
      
      if (ls == 0 && rs == 0)
        state = Done;
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
  return (lineSensorValues[0] > 500) || (lineSensorValues[1] > 500) || (lineSensorValues[2] > 500);
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
  short power_difference = proportional/4;// + derivative*6;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const short max = 255, diff_max = max;
  if(power_difference > diff_max)
    power_difference = diff_max;
  if(power_difference < -diff_max)
    power_difference = -diff_max;

  if(power_difference < 0)
  {
    ls = max + power_difference;
    rs = max;
  }
  else
  {
    ls = max;
    rs = max - power_difference;
  }
  driveMotors.setSpeeds(ls, rs);
}


