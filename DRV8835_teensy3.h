#ifndef DRV8835_teensy3_h
#define DRV8835_teensy3_h

#include <Arduino.h>

class DRV8835
{
  public:
    DRV8835(unsigned char DIRA, unsigned char PWMA, unsigned char DIRB, unsigned char PWMB) : _DIRA(DIRA), _PWMA(PWMA), _DIRB(DIRB), _PWMB(PWMB) { }
    
    void init(uint32_t frequency);
    void setMASpeed(int16_t speed);
    void setMBSpeed(int16_t speed);
    void setSpeeds(int16_t mASpeed, int16_t mBSpeed);
    
  private:
    unsigned char _DIRA;
    unsigned char _PWMA;
    unsigned char _DIRB;
    unsigned char _PWMB;
};

#endif // DRV8835_teensy3_h
