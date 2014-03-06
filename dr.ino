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

Encoder lwEnc(18, 19);
Encoder rwEnc(21, 20);

Pushbutton btn(0);

enum { WaitForButton, FindLine, StartFollowLine, FollowLine, Decel, GoHome, Done } state;

int16_t ls, rs;

const int16_t AngleScale = 20000;
const uint16_t StepsPerRadian = 810; //TODO: tune

int16_t c = AngleScale;
int16_t s = 0;
int32_t x = 0, y = 0;

#define sign(x) ((x) < 0 ? -1 : 1)

void setup()
{
  Serial.begin(115200);
  setLineSensorCalibration();
  driveMotors.init(23437);
  
  pinMode(13, OUTPUT);
  
  state = WaitForButton;
}

void loop()
{//updateWheelEncoders();return;
  static uint16_t millisInitial = 0;
  static int16_t lsInitial, rsInitial;
  
  if (state != WaitForButton)
    updateWheelEncoders();
  
  switch(state)
  {
    case WaitForButton:
    {
      btn.waitForButton();
      delay(1000);
      lwEnc.write(0);
      rwEnc.write(0);
      state = FindLine;
      millisInitial = millis();
    }
    break;

    case FindLine:
    {
      int16_t s = (millis() - millisInitial) / 4;
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
        state = Decel;
        millisInitial = millis();
        lsInitial = ls;
        rsInitial = rs;
        digitalWrite(13, LOW);
      }
    }
    break;
    
    case Decel:
    {
      // decel by subtracting 1 from original speeds every 4 ms
      int16_t ds = (millis() - millisInitial) / 4;
      
      ls = lsInitial - ds;
      if (ls < 0)
        ls = 0;
        
      rs = rsInitial - ds;
      if (rs < 0)
        rs = 0;
        
      driveMotors.setSpeeds(ls, rs);
      
      if (ls == 0 && rs == 0)
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
        state = GoHome;
      }
    }
    break;
    
    case GoHome:
    {
        if (s > 0 || (s == 0 && c < 0))
          driveMotors.setSpeeds(50, -50);
        else if (s < 0)
          driveMotors.setSpeeds(-50, 50);
          
          if (abs(s) < 500)
          {
          driveMotors.setSpeeds(80, 80);
          while(x < -500)
          {
            updateWheelEncoders();
            
          }
          driveMotors.setSpeeds(0, 0);
          while(1);
          }
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
  int8_t lCount = lwEnc.read();
  int8_t rCount = rwEnc.read();
  
  if (lCount != 0)
  {
    int16_t dc = + divide(lCount*s - lCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = - divide(lCount*c + lCount*s/2/StepsPerRadian, StepsPerRadian);
  
    c += dc;
    s += ds;
    x += lCount * c;
    y += lCount * s;
    
    lwEnc.write(0);
  }
  
  if (rCount != 0)
  {
    int16_t dc = - divide(rCount*s + rCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = + divide(rCount*c - rCount*s/2/StepsPerRadian, StepsPerRadian);
    
    c += dc;
    s += ds;
    x += rCount * c;
    y += rCount * s;
  
    rwEnc.write(0);
  }
}

int16_t divide(int16_t a, int16_t b)
{
  return (a + sign(a)*(b/2-1)) / b;
}

boolean onLine()
{
  return (lineSensorValues[0] > 500) || (lineSensorValues[1] > 500) || (lineSensorValues[2] > 500);
}

void followLine()
{
  static uint16_t last_proportional = 0;
  static long integral = 0;
  
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
  int16_t power_difference = proportional/4;// + derivative*6;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  const int16_t max = 255, diff_max = max;
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


