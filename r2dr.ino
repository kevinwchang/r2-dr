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

const uint16_t MaxSpeed = 255;

DRV8835 driveMotors(7, 5, 8, 6);

Encoder lwEnc(18, 19);
Encoder rwEnc(21, 20);

Pushbutton btn(0);

enum { WaitForButton, FindLine, StartFollowLine, FollowLine, Decel, GoHome, Done } state;

int16_t ls, rs;

const uint16_t AngleScale = 20000;
const uint16_t StepsPerRadian = 1025; //TODO: tune

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
{
  //Serial.print(lwEnc.read());Serial.print("\t");Serial.println(rwEnc.read());delay(100);return;
  //static int count=0;count++;updateWheelEncoders();if(count>100){printDebug();count=0;}return;
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
      if (s <= MaxSpeed)
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
        printDebug();
        Serial.println("end of line");
        
        transform();
        printDebug();
        
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
          driveMotors.setSpeeds(0, 0);
          printDebug();
          Serial.println("facing home");
          //btn.waitForButton();
            
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
  static int8_t prevLCount = 0, prevRCount = 0;
  
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
  }
  
  static uint16_t lastPrint = 0;
  if ((uint16_t)(millis() - lastPrint) > 1000)
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
  const int16_t MaxDiff = MaxSpeed;
  if(power_difference > MaxDiff)
    power_difference = MaxDiff;
  if(power_difference < -MaxDiff)
    power_difference = -MaxDiff;

  if(power_difference < 0)
  {
    ls = MaxSpeed + power_difference;
    rs = MaxSpeed;
  }
  else
  {
    ls = MaxSpeed;
    rs = MaxSpeed - power_difference;
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
