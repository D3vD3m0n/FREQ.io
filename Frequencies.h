

#include <stdlib.h>
//#if ARDUINO >= 100
#include <Arduino.h>
//#else
//#include <WProgram.h>
//#include <wiring.h>
//#endif
#include <math.h>

#ifndef Frequencies_h
#define Frequencies_h

class Frequencies {
  public:
    Frequencies();
    Frequencies(uint8_t pinYuksekPWM , uint8_t pinAlcakPWM , double HzFrekan);
    void run();
    void init();
  	void init(uint8_t pin, uint8_t hz, uint16_t coolDown);
    void end();
    void printData();
    void setAlarm(void (*action)(), uint16_t thresh, uint16_t hold);
    void checkAlarm();
    int16_t level();
    void alternatePin(bool enabled);
    void throwError(byte errorNum);
    void tone(int frequency, byte volume);
    void fadeOut(int duration);
    void noTone();
    unsigned long millis();
    unsigned long micros();
    void delay(unsigned long d);
    void delayMicroseconds(unsigned long du);
    void setMasterVolume(float mv);
	
  private:
    double modifeMap(double x, double in_min, double in_max, double out_min, double out_max);
    uint8_t _pin[2];
    unsigned long _zamanAraligi;
    unsigned long _sonZamanAraligi;
    unsigned long _ilkZaman;
    double alfa;
    unsigned long zaman;
    long gen;
};

#endif