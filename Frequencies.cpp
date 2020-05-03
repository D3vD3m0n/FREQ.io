//Released under the GPLv3 license.

#include "Frequencies.h"

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <Ticker.h>
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#include <Arduino.h>
#endif
uint16_t b_reads[16] = {};       // Takes in raw ADC readings for phase cancellation
uint16_t b_averages[32] = {};    // Smoothes the b_roughMotion output of b_reads[]
uint16_t b_sensor = 0;           // ADC pin to use
uint8_t b_phaseShift = 0;        // Offset in index of b_reads[] used for phase cancellation
uint32_t b_buzzStart = 0;        // Used to timestamp beginning of usage for b_buzzWait
int16_t b_motionLevel = 0;       // Most current reading into b_record[]
uint16_t b_buzzThreshold = 20;   // Minimum amount of motion required to trigger alarm
uint16_t b_buzzHold = 1000;      // Minimum amount of time between alarms
uint32_t b_buzzWait = 0;         // Timestamp millis() must pass before another alarm can trigger
uint32_t b_lastAlarm = 0;        // Timestamp when last alarm triggered
bool b_buzzReady = false;        // False before millis() reaches time in b_buzzWait to stop alarms
bool b_alarm = false;            // Alarm flag, set to true by ISR, checked by checkAlarm()
int16_t b_alarmLine = -100;      // Used by printData() to produce "vertical" lines in the plotter
static void (*b_alarmFunc)();    // The function to be called in an alarm state, defined by setAlarm()
int16_t b_buzzRead = 0;          // Difference between most recent read ond the one 128ms ago,used to derive current motion measurement
int16_t b_record[64] = {};       // Used to compare most current reading with one from 128ms agoto produce b_buzzRead value
bool _toneEnable = false;        //Vol Control
bool _toneState = false;
byte _toneVol = 0;
unsigned int _freq = 0;
float _masterVol = 1.00;
bool _fadeOut = false;
float _fadeVol = 1.00;
float _fadeAmount = 0.01;
byte _fadeCounter = 0;

#if defined(ESP8266)
Ticker timer;
volatile int interrupts;
byte _SPEAKER = 4;
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
byte _SPEAKER = 4;
#endif
#if defined(__AVR_ATmega32U4__)
byte _SPEAKER = 9;
#endif
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
byte _SPEAKER = 5;
#endif

Frequencies::Frequencies(uint8_t pinYuksekPWM , uint8_t pinAlcakPWM , double HzFrekans){
  _pin[0] = pinYuksekPWM;
  _pin[1] = pinAlcakPWM;
  pinMode(_pin[0],OUTPUT);
  pinMode(_pin[1],OUTPUT);
  digitalWrite(_pin[0],0);
  digitalWrite(_pin[1],0);

  _zamanAraligi = 1000000 / HzFrekans; 
}

Frequencies::Frequencies() {
	uint8_t YuksekPWM = 3; 
	uint8_t AlcakPWM = 5; 
	double Frekans = 1 ;
	_pin[0] = YuksekPWM;
	_pin[1] = AlcakPWM;
  pinMode(_pin[0],OUTPUT);
  pinMode(_pin[1],OUTPUT);
  digitalWrite(_pin[0],0);
  digitalWrite(_pin[1],0);
	
  _zamanAraligi = 1000000 / Frekans; 
}

#if defined(ESP8266)
// ISR to Fire when Timer is triggered
void ICACHE_RAM_ATTR onTime() {
	interrupts++;
	Serial.print("Total Ticks:");
	Serial.println(interrupts);
	// Re-Arm the timer as using TIM_SINGLE
	timer1_write(2500000);//12us
}
#endif

void Frequencies::run(){
  zaman = micros();
  _sonZamanAraligi= (zaman-_ilkZaman) % _zamanAraligi;
  alfa = modifeMap(_sonZamanAraligi,0,_zamanAraligi,0,2*PI);
  gen = sin(alfa)*255; // -255 - +255 

  if (alfa < PI){
    digitalWrite(_pin[1],0);
    analogWrite(_pin[0],gen);  
  }else{
    digitalWrite(_pin[0],0);
    analogWrite(_pin[1],-gen); 
  }
  
  pinMode(_SPEAKER, OUTPUT);
}

void Frequencies::init(){
  _ilkZaman=micros();
  _sonZamanAraligi=0;
  alfa=0;
}

// pin = ADC pin to use for measurement
// hz = AC electricity frequency for your region used for phase cancellation
// coolDown = Amount of time to "cool down" - ADC is too sensitive at sketch start and needs to stabilize

void Frequencies::init(uint8_t pin, uint8_t hz, uint16_t coolDown) {
  _ilkZaman=micros();
  _sonZamanAraligi=0;
  alfa=0;
  b_buzzStart = ::millis();
  b_buzzWait = b_buzzStart + coolDown;
  b_sensor = pin;
  
  if (hz == 60) {b_phaseShift = 2;}
  else if (hz == 50) {b_phaseShift = 1;}
  for (byte i = 0; i < 64; i++) {b_record[i] = 0;}
  for (byte i = 0; i < 16; i++) {b_reads[i] = 0;}
  for (byte i = 0; i < 32; i++) {b_averages[i] = 0;}

  #if defined(ESP8266)
  //Initialize Ticker every 0.5s
	timer1_attachInterrupt(onTime); // Add ISR Function
	timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
	/* Dividers:
		TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
		TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
		TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
	Reloads:
		TIM_SINGLE	0 //on interrupt routine you need to write a new value to start the timer again
		TIM_LOOP	1 //on interrupt the counter will start with the same value again
	*/ 
  //Arm the Timer for our 0.5s Interval
	timer1_write(2500000); // 2500000 / 5 ticks per us from TIM_DIV16 == 500,000 us interval 
  #endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
   cli(); // TIMER 1 for interrupt frequency 500 Hz: stop interrupts
   TCCR1A = 0; // set entire TCCR1A register to 0
   TCCR1B = 0; // same for TCCR1B
   TCNT1  = 0; // initialize counter value to 0 set compare match register for 500 Hz increments
   OCR1A = 31999; // = 16000000 / (1 * 500) - 1 (must be <65536)  turn on CTC mode
   TCCR1B |= (1 << WGM12);// Set CS12, CS11 and CS10 bits for 1 prescaler  
   TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
   TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
   sei(); // allow interrupts
  #endif
}


double Frequencies::modifeMap(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Frequencies::end() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  TIMSK1 &= (0 << OCIE1A); // Disable timer1 ISR
  TCCR0B = (TCCR0B & 0b11111000) | 0x03;
  #endif
  #if defined(ESP8266)
  
  #endif
  noTone();
}

void Frequencies::printData() {
  if (Serial) { // If user has started Serial
    if (b_buzzReady == true) { // If the ADC is ready, display chart - if not, display a single line
      Serial.print(F("100\t"));
      Serial.print(F("-100\t"));
      Serial.print(b_buzzThreshold);
      Serial.print(F("\t"));
      Serial.print(int(b_buzzThreshold) * -1);
      Serial.print(F("\t"));
      Serial.print(b_alarmLine);
      Serial.print(F("\t"));
      Serial.println(b_buzzRead);
    }
    else {
      Serial.println("0\t0\t0\t0\t0\t0\t");
    }
  }
}

void Frequencies::setAlarm(void (*action)(), uint16_t thresh, uint16_t hold) {
  // action() = user-defined function to call during alarm state - can be any function
  // thresh = sets value of buzzThreshold
  // hold = sets value of buzzHold

  b_alarmFunc = action;
  b_buzzThreshold = thresh;
  b_buzzHold = hold;
}

void Frequencies::checkAlarm() {
  if (b_alarm == true) { // If alarm flag has been set in the ISR:
    b_alarmFunc();             // Call the alarm function
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    TIMSK1 &= (0 << OCIE1A); // Disable timer1 ISR:
    TIMSK1 |= (1 << OCIE1A); // Enable timer1 ISR:
    #endif
  #if defined(ESP8266)  

  #endif
    b_alarm = false;           // Reset alarm flag

    // show "vertical" line in printData()
    if (b_alarmLine == -100) {
      b_alarmLine = 100;
    }
    else if (b_alarmLine == 100) {
      b_alarmLine = -100;
    }
  }
}

int16_t Frequencies::level() { // Returns value of buzzRead
  return b_buzzRead;
}


//Vol Control

void Frequencies::alternatePin(bool enabled) {
  if (enabled == true) {
	  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    _SPEAKER = 13;
#endif

#if defined(__AVR_ATmega32U4__)
    _SPEAKER = 10;
#endif

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
    _SPEAKER = 6;
#endif

  }
}

void Frequencies::throwError(byte _errorNum) {
  end();
  while (true) {
    byte _beeps = _errorNum;
    while (_beeps > 0) {
      ::tone(_SPEAKER, 500, 100);
      ::delay(200);
      _beeps--;
    }
    ::delay(1000);
  }
}

void Frequencies::fadeOut(int duration){
  _fadeOut = true;
  _fadeAmount = (1/float(_freq*2*(duration/1000.0)))*10;
  _fadeCounter = 0;
}

void Frequencies::tone(int frequency, byte volume)
{
  _freq = frequency;
  _fadeOut = false;
  _fadeVol = 1.00;
  _toneEnable = true;
  long _clk = F_CPU / (1 * frequency * 2) - 1;
  if (_clk >= 65536) {
    _clk = 65536 - 1;
  }

  cli(); // stop interrupts
  //OCR1A = _clk;
  sei(); // allow interrupts

  _toneVol = volume;
  return;
}
void Frequencies::noTone()
{
  _toneEnable = false;
  _toneState = false;
  _toneVol = 0;
  return;
}
void Frequencies::delay(unsigned long d) {
  ::delay(d * 64);
  return;
}
unsigned long Frequencies::millis() {
  return ::millis() / 64;
}
unsigned long Frequencies::micros() {
  return ::micros() / 64;
}
void Frequencies::delayMicroseconds(unsigned long du) {
  ::delayMicroseconds(du * 64);
  return;
}
void Frequencies::setMasterVolume(float mv) {
  _masterVol = mv;
  return;
}


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
// Interrupt Service Routine called at 500 Hz
ISR(TIMER1_COMPA_vect) {
	 if (_toneEnable == true) {
    if (_toneState == false) {
      _toneState = !_toneState;
      analogWrite(_SPEAKER, _toneVol * _masterVol * _fadeVol);
    }
    else if (_toneState == true) {
      _toneState = !_toneState;
      analogWrite(_SPEAKER, 0);
    }
	if(_fadeOut == true){
		_fadeCounter++;
		if(_fadeCounter >= 10){
			_fadeCounter = 0;
			if(_fadeVol > 0){
				_fadeVol-=_fadeAmount;
				if(_fadeVol < 0){
					_fadeVol = 0;
				}
			}
			else{
				_fadeOut = false;
				_toneEnable = false;
				_toneState = false;
				_toneVol = 0;
			}
		}
	}
  }
	
  // Check if buzzWait is over:
  if (millis() >= b_buzzWait && b_buzzReady == false) {
    b_buzzReady = true;
  }

  uint16_t b_reading = analogRead(b_sensor); // Take reading

  // keep moving average of b_reading in reads[]
  for (byte i = 0; i < 15; i++) {
    b_reads[i] = b_reads[i + 1];
  }
  b_reads[15] = b_reading;

  // phase cancellation to remove AC sine effects
  // by comparing current value to an older one 180
  // degrees out of phase
  uint16_t b_roughMotion = b_reads[b_phaseShift] + b_reading;

  // keep moving average of b_roughMotion in b_averages[];
  for (byte i = 0; i < 31; i++) {
    b_averages[i] = b_averages[i + 1];
  }
  b_averages[31] = b_roughMotion;

  // b_motionLevel is the average of all in the b_averages[] array
  uint16_t b_sum = 0;
  for (byte i = 0; i < 32; i++) {
    b_sum += b_averages[i];
  }
  b_motionLevel = abs(b_sum / 32);

  // Add b_motionLevel to b_record[] for comparing current motion data
  // against past data to calculate the shift (amount of change = b_buzzRead)
  for (byte i = 0; i < 63; i++) {
    b_record[i] = b_record[i + 1];
  }
  b_record[63] = b_motionLevel;
  b_buzzRead = (b_motionLevel - b_record[0])*-1; // inverted so that movement towards antenna
                                                 // is a positive shift in the log

  // If buzzRead is >= buzzThreshold, we're past the buzzWait, and
  //we haven't had another alarm too recently, set the alarm flag to true.
  if (abs(b_buzzRead) >= b_buzzThreshold && b_buzzReady == true && millis() >= b_lastAlarm + b_buzzHold) {
    b_lastAlarm = millis();
    b_alarm = true;
  }
}
ISR(TIMER1_COMPB_vect){
  TCCR1B = 0;  //stop timer here
}
#endif