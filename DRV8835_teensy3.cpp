#include "DRV8835_teensy3.h"
#include <Arduino.h>

void DRV8835::init(long frequency)
{
  pinMode(_DIRA, OUTPUT);
  pinMode(_DIRB, OUTPUT);
  
  analogWriteFrequency(_PWMA, frequency);
  analogWriteFrequency(_PWMB, frequency);
}

void DRV8835::setMASpeed(short speed)
{
  byte reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;
    reverse = 1;
  }
  if (speed > 255)
    speed = 255;
    
  analogWrite(_PWMA, speed);
  digitalWrite(_DIRA, reverse);
}

void DRV8835::setMBSpeed(short speed)
{
  byte reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;
    reverse = 1;
  }
  if (speed > 255)
    speed = 255;
    
  analogWrite(_PWMB, speed);
  digitalWrite(_DIRB, reverse);
}

void DRV8835::setSpeeds(short mASpeed, short mBSpeed)
{
  setMASpeed(mASpeed);
  setMBSpeed(mBSpeed);
}
