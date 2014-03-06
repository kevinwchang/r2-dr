#ifndef DRV8835_teensy3_h
#define DRV8835_teensy3_h

class DRV8835
{
  public:
    DRV8835(unsigned char DIRA, unsigned char PWMA, unsigned char DIRB, unsigned char PWMB) : _DIRA(DIRA), _PWMA(PWMA), _DIRB(DIRB), _PWMB(PWMB) { }
    
    void init(long frequency);
    void setMASpeed(short speed);
    void setMBSpeed(short speed);
    void setSpeeds(short mASpeed, short mBSpeed);
    
  private:
    unsigned char _DIRA;
    unsigned char _PWMA;
    unsigned char _DIRB;
    unsigned char _PWMB;
};

#endif // DRV8835_teensy3_h
