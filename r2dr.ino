#include "QTRSensors_teensy3.h"
#include "DRV8835_teensy3.h"
#include <Encoder.h>
#include <Pushbutton.h>

//#define DISABLE_SOUND

const uint8_t NumLineSensors = 3;
const int16_t LineSensorTimeout = 1000;
const int16_t LineSensorMin = 200;
const int16_t LineSensorMax = 1000;

const int16_t DomeSensorMin = 200;
const int16_t DomeSensorMax = 400;

QTRSensorsRC lineSensors((uint8_t[]) {14, 15, 16}, NumLineSensors, LineSensorTimeout);
uint16_t lineSensorValues[NumLineSensors];

QTRSensorsRC domeSensor((uint8_t[]) {17}, 1, LineSensorTimeout);
uint16_t domeSensorValue;

DRV8835 driveMotors(7, 5, 8, 6);
DRV8835 domeMotor(10, 9, 10, 9);
const int16_t MaxSpeed = 255;

int16_t lCount, rCount;
uint8_t lEncErr, rEncErr;

Encoder domeEnc(22, 23);

// ~380:1 gearbox * 12 CPR(motor)
// 4550 is exact; see gear ratio at https://www.pololu.com/product/4790
const int16_t DomeCPR = 4550;

Pushbutton btn(0);

enum { WaitForButton, FindLine, FollowLine, Decel, TurnToHome, GoHome } state;

enum
{
  ALARM7, // do not use - same as startsnd
  ALARM8, // finish
  HUM1,
  HUM2,
  MISC1,
  MISC14,
  MISC17,
  MISC29,
  MISC32,
  MISC33,
  MISC35, // end of line
  PROC7, // start
  RAZZ5,
  SCREAM1,
  STARTSND, // power on
  WHIST1,
  WHIST14,
  WHIST15,
  WHIST16
} sounds;

const uint8_t IdleSounds[] = { HUM1, HUM2, MISC1, MISC17, MISC29, MISC32, MISC33, RAZZ5, WHIST1, WHIST14, WHIST15, WHIST16 };
const uint8_t IdleSoundCount = 12;

int16_t ls, rs;

const uint16_t AngleScale = 20000;
const uint16_t StepsPerRadian = 960; //1075;
const uint8_t LeapTickPerL = 0;
const uint8_t LeapTickPerR = 200;
const int32_t HomeMaxY = 13000000L;
const int16_t HomeMaxS = 15000L;

int16_t c, s;
int32_t x, y;

#define sign(x) ((x) < 0 ? -1 : 1)

void followLine(int16_t accelMaxSpeed = 255);

const uint8_t SomoClk = 3;
const uint8_t SomoData = 4;

void setup()
{
  randomSeed(analogRead(14));
  Serial.begin(115200);
  setSensorCalibration();
  driveMotors.init(20000);
  domeMotor.init(20000);

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(18), tickL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), tickL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), tickR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), tickR, CHANGE);
  lEncErr = 0;
  rEncErr = 0;

  pinMode(13, OUTPUT);

  somoInit();
  somoCmd(STARTSND);

  homeDome();

  state = WaitForButton;
}

uint16_t lastSoundMs = 0;

void loop()
{
  /*Serial.print(lCount);
  Serial.print('\t');
  Serial.print(rCount);
  Serial.print('\t');
  Serial.print(lEncErr);
  Serial.print('\t');
  Serial.print(rEncErr);
  Serial.println();
  delay(10);
  return;*/

  static uint16_t accelStartMs = 0;
  static uint16_t stateStartMs = 0;
  static bool accel = false;
  static int16_t accelMaxSpeed = 0;

  updateDome();
  checkFall();
  idleSounds();

  if (accel)
  {
    accelMaxSpeed = (uint16_t)(millis() - accelStartMs) / 4; // 0 to 255 over ~1 second
    if (accelMaxSpeed >= MaxSpeed)
    {
      accelMaxSpeed = MaxSpeed;
      accel = false;
    }
  }

  switch (state)
  {
    case WaitForButton:

      if (btn.getSingleDebouncedRelease())
      {
        digitalWrite(13, HIGH);
        delay(1000);

        c = AngleScale;
        s = 0;
        x = 0, y = 0;

        state = FindLine;
        accel = true;
        accelStartMs = millis();
        somoCmd(PROC7);
      }
      break;

    case FindLine:

      driveMotors.setSpeeds(accelMaxSpeed, accelMaxSpeed);

      lineSensors.readCalibrated(lineSensorValues);
      if (onLine())
      {
        state = FollowLine;
        stateStartMs = millis();
      }

      break;

    case FollowLine:

      followLine(accelMaxSpeed);

      // to avoid thinking we're done before we're stable on the line,
      // don't look for end of line for 1 second
      if ((uint16_t)(millis() - stateStartMs) > 1000 && !onLine())
      {
        state = Decel;
        stateStartMs = millis();
        digitalWrite(13, LOW);
        somoCmd(MISC35);
      }

      break;

    case Decel:

      accelMaxSpeed = MaxSpeed - (uint16_t)(millis() - stateStartMs) / 2; // 255 to 0 over ~0.5 second
      if (accelMaxSpeed < 0) { accelMaxSpeed = 0; }
      if (ls > accelMaxSpeed) { ls = accelMaxSpeed; }
      if (rs > accelMaxSpeed) { rs = accelMaxSpeed; }
      driveMotors.setSpeeds(ls, rs);

      if (accelMaxSpeed == 0)
      {
        //printDebug();
        transform();
        //printDebug();

        state = TurnToHome;
        digitalWrite(13, HIGH);
      }

      break;

    case TurnToHome:

      turnToHome();

      if (abs(s) < (AngleScale / 18))
      {
        // pointing within 5 degrees of home direction

        state = GoHome;
        accel = true;
        accelStartMs = millis();
        digitalWrite(13, LOW);
      }

      break;

    case GoHome:

      goHome(accelMaxSpeed);

      if (x > -5000000)
      {
        driveMotors.setSpeeds(0, 0);
        state = WaitForButton;
        somoCmd(ALARM8);
      }

      break;
  }
}

void homeDome()
{
  // find + edge:
  domeMotor.setMASpeed(100);

  // wait for black
  while (1)
  {
    domeSensor.readCalibrated(&domeSensorValue);
    if (domeSensorValue > 500) { break; }
  }

  domeMotor.setMASpeed(50);

  // wait for white
  while (1)
  {
    domeSensor.readCalibrated(&domeSensorValue);
    if (domeSensorValue < 500) { break; }
  }

  domeMotor.setMASpeed(0);
  delay(100);
  int16_t posP = domeEnc.read();

  // find - edge:
  domeMotor.setMASpeed(-50);

  // wait for black
  while (1)
  {
    domeSensor.readCalibrated(&domeSensorValue);
    if (domeSensorValue > 500) { break; }
  }

  // wait for white
  while (1)
  {
    domeSensor.readCalibrated(&domeSensorValue);
    if (domeSensorValue < 500) { break; }
  }

  domeMotor.setMASpeed(0);
  delay(100);
  int16_t posM = domeEnc.read();

  // black bar is (posP - posM) wide, and we're now on the - side of it,
  // so our current position should be:
  domeEnc.write(-(posP - posM) / 2);
}

void updateDome()
{
  static int8_t lastCount = 0; // last enc reading
  static int16_t wrappedCount = 0;
  static uint16_t lastDomeMoveMs = 0;

  int8_t count = domeEnc.read();

  wrappedCount += (int8_t)(lastCount - count);
  lastCount = count;

  if (wrappedCount >  (DomeCPR/2)) { wrappedCount -= DomeCPR; }
  if (wrappedCount < -(DomeCPR/2)) { wrappedCount += DomeCPR; }

  const uint8_t MovePeriod = 40; // 40 ms = 25 Hz
  uint16_t now = millis();

  if ((uint16_t)(now - lastDomeMoveMs) >= MovePeriod)
  {
    lastDomeMoveMs += MovePeriod;

    float domeTarget;

    switch (state)
    {
      case FindLine:
      case FollowLine:
      case Decel:
      case TurnToHome:
      case GoHome:
        domeTarget = atan2(s, -c) + atan2(y, x);
        break;

      default:
        domeTarget = 0;
    }
    //domeTarget = ((millis() >> 11) & 1) ? domeTarget : 0;

    int16_t targetCount = domeTarget * DomeCPR / 2 / PI;
    if (targetCount >  DomeCPR/2) { targetCount -= DomeCPR; }
    if (targetCount < -DomeCPR/2) { targetCount += DomeCPR; }

    int16_t countDiff = (wrappedCount - targetCount);
    if (countDiff >  DomeCPR/2) { countDiff -= DomeCPR; }
    if (countDiff < -DomeCPR/2) { countDiff += DomeCPR; }

    int16_t domeSpeed = -countDiff;
    domeSpeed = constrain(domeSpeed, -MaxSpeed, MaxSpeed);

    domeMotor.setMASpeed(domeSpeed);
  }
}

void setSensorCalibration()
{
  lineSensors.calibrate(); // force allocate calibrated values

  for (uint8_t i = 0; i < NumLineSensors; i++)
  {
    lineSensors.calibratedMinimumOn[i] = LineSensorMin;
    lineSensors.calibratedMaximumOn[i] = LineSensorMax;
  }

  domeSensor.calibrate();
  domeSensor.calibratedMinimumOn[0] = DomeSensorMin;
  domeSensor.calibratedMaximumOn[0] = DomeSensorMax;
}

/*
18 B3
19 B2
20 D5
21 D6
*/

void tickL()
{
  static uint8_t lastL1 = 0, lastL2 = 0;

  uint8_t newL1 = ((GPIOB_PDIR & 0x08) != 0);
  uint8_t newL2 = ((GPIOB_PDIR & 0x04) != 0);

  if((lastL1 ^ newL1) & (lastL2 ^ newL2)) { lEncErr++; }

  int8_t dLCount = (lastL1 ^ newL2) - (int)(newL1 ^ lastL2);
  updateLWheelEncoder(dLCount);

  lCount += dLCount;

  lastL1 = newL1;
  lastL2 = newL2;
}

void tickR()
{
  static uint8_t lastR1 = 0, lastR2 = 0;

  uint8_t newR1 = ((GPIOD_PDIR & 0x40) != 0);
  uint8_t newR2 = ((GPIOD_PDIR & 0x20) != 0);

  if((lastR1 ^ newR1) & (lastR2 ^ newR2)) { rEncErr++; }

  int8_t dRCount = (lastR1 ^ newR2) - (int)(newR1 ^ lastR2);
  updateRWheelEncoder(dRCount);

  rCount += dRCount;

  lastR1 = newR1;
  lastR2 = newR2;
}

void updateLWheelEncoder(int8_t dLCount)
{
  static uint8_t leapLCount = 0;

  if (dLCount != 0)
  {
    if (LeapTickPerL > 0)
    {
      leapLCount += dLCount;
      if (leapLCount >= LeapTickPerL)
      {
        dLCount += sign(dLCount);
        leapLCount -= LeapTickPerL;
      }
    }

    int16_t dc = + divide(dLCount*s - dLCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = - divide(dLCount*c + dLCount*s/2/StepsPerRadian, StepsPerRadian);

    c += dc;
    s += ds;
    c = constrain(c, -AngleScale, AngleScale);
    s = constrain(s, -AngleScale, AngleScale);
    x += dLCount * c;
    y += dLCount * s;
  }
}

void updateRWheelEncoder(int8_t dRCount)
{
  static uint8_t leapRCount = 0;

  if (dRCount != 0)
  {
    if (LeapTickPerR > 0)
    {
      leapRCount += dRCount;
      if (leapRCount >= LeapTickPerR)
      {
        dRCount += sign(dRCount);
        leapRCount -= LeapTickPerR;
      }
    }

    int16_t dc = - divide(dRCount*s + dRCount*c/2/StepsPerRadian, StepsPerRadian);
    int16_t ds = + divide(dRCount*c - dRCount*s/2/StepsPerRadian, StepsPerRadian);

    c += dc;
    s += ds;
    c = constrain(c, -AngleScale, AngleScale);
    s = constrain(s, -AngleScale, AngleScale);
    x += dRCount * c;
    y += dRCount * s;
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

void turnToHome()
{
  const int16_t TurnMaxDiff = MaxSpeed / 2;
  int32_t err;

  if (c < 0)
  {
    // pointed backwards
    err = (s > 0 ? TurnMaxDiff : -TurnMaxDiff);
  }
  else
  {
    int32_t target_s = -(y / 10000 * HomeMaxS / (HomeMaxY / 10000));
    target_s = constrain(target_s, -HomeMaxS, HomeMaxS);
    err = (s - target_s) / 20;
    err = constrain(err, -TurnMaxDiff, TurnMaxDiff);
  }

  driveMotors.setSpeeds(err, -err);
}

void goHome(int16_t accelMaxSpeed)
{
  int16_t speed = accelMaxSpeed;
  int32_t err;

  if (x > -20000000)
  {
    speed /= 4;
  }

  /*if (c < 0)
  {
    // pointed backwards
    err = (s > 0 ? speed/2 : -speed/2);
  }
  else*/
  {
    int32_t target_s = -(y / 10000 * HomeMaxS / (HomeMaxY / 10000));
    target_s = constrain(target_s, -HomeMaxS, HomeMaxS);
    err = (s - target_s) / 100;
    err = constrain(err, -speed, speed);
  }

  if (err > 0)
  {
    driveMotors.setSpeeds(speed, speed - err);
  }
  else
  {
    driveMotors.setSpeeds(speed + err, speed);
  }
}

boolean onLine()
{
  return (lineSensorValues[0] > 500) || (lineSensorValues[1] > 500) || (lineSensorValues[2] > 500);
}

void followLine(int16_t accelMaxSpeed)
{
  static uint16_t lastProportional = 0;
  static long integral = 0;
  const int16_t LfMaxSpeed = accelMaxSpeed, LfMaxDiff = accelMaxSpeed;

  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  uint16_t position = lineSensors.readLine(lineSensorValues);

  // The "proportional" term should be 0 when we are on the line.
  int16_t proportional = ((int)position) - 1000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int16_t derivative = proportional - lastProportional;
  integral += proportional;

  // Remember the last position.
  lastProportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.
  int16_t power_difference = proportional/4;// + derivative*6;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if (power_difference >  LfMaxDiff) { power_difference =  LfMaxDiff; }
  if (power_difference < -LfMaxDiff) { power_difference = -LfMaxDiff; }

  if (power_difference < 0)
  {
    ls = LfMaxSpeed + power_difference;
    rs = LfMaxSpeed;
  }
  else
  {
    ls = LfMaxSpeed;
    rs = LfMaxSpeed - power_difference;
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

void somoInit()
{
  pinMode(SomoClk, OUTPUT);
  digitalWrite(SomoClk, HIGH);
  pinMode(SomoData, OUTPUT);
  digitalWrite(SomoData, HIGH);

  while (millis() < 1200) {}
}

void somoCmd(uint16_t cmd)
{
#ifndef DISABLE_SOUND
  lastSoundMs = millis();
  digitalWrite(SomoClk, LOW);
  delay(2);

  for (int8_t b = 15; b >= 0; b--)
  {
      digitalWrite(SomoClk, LOW);
      digitalWrite(SomoData, (cmd >> b) & 1);
      delayMicroseconds(90);
      digitalWrite(SomoClk, HIGH);
      delayMicroseconds(90);
  }
  delay(2);
#endif
}

void idleSounds()
{
  static uint16_t timeToNextSound = random(7000, 20000);

  if ((uint16_t)(millis() - lastSoundMs) >= timeToNextSound)
  {
    somoCmd(IdleSounds[random(0, IdleSoundCount)]);
    timeToNextSound = random(7000, 20000);
  }
}

void checkFall()
{
  static uint16_t lastCheckMs = millis();
  static uint8_t count = 0;

  uint16_t now = millis();

  if (state != WaitForButton && (uint16_t)(now - lastCheckMs) >= 100)
  {
    lineSensors.readCalibrated(lineSensorValues);
    if (lineSensorValues[0] > 900 &&
        lineSensorValues[1] > 900 &&
        lineSensorValues[2] > 900)
    {
      count++;
      if (count == 3)
      {
        driveMotors.setSpeeds(0, 0);
        domeMotor.setMASpeed(0);
        somoCmd(SCREAM1);
        while (1) {}
      }
    }
    else
    {
      count = 0;
    }
    lastCheckMs = now;
  }
}
