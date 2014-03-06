#include "DRV8835_teensy3.h"

void DRV8835::init(uint32_t frequency)
{
  pinMode(_DIRA, OUTPUT);
  pinMode(_DIRB, OUTPUT);
  
  analogWriteFrequency(_PWMA, frequency);
  analogWriteFrequency(_PWMB, frequency);
}

void DRV8835::setMASpeed(int16_t speed)
{
  uint8_t reverse = 0;
  
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

void DRV8835::setMBSpeed(int16_t speed)
{
  uint8_t reverse = 0;
  
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

void DRV8835::setSpeeds(int16_t mASpeed, int16_t mBSpeed)
{
  setMASpeed(mASpeed);
  setMBSpeed(mBSpeed);
}
