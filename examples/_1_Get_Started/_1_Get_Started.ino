
#define OUT 9          // OUT PIN

void setup()
{
  Serial.begin(9600);
  pinMode(OUT, OUTPUT);
}

void loop()
{
  if(Serial.available()) 
  {
    long frequency = Serial.parseInt();
    Serial.print(Set_frequency(frequency));
    Serial.println(" Hz");
    delay(3000);
  }
}


long Set_frequency(long freq)
{
  TCCR1A = 0b01000000; 
  uint16_t ocr;
  if(freq < 134) 
  {
    TCCR1B = 0b00001010; 
    ocr = 1000000UL/freq - 1;
    freq = 1000000UL/(ocr+1);
  }
  else 
  {
    TCCR1B = 0b00001001;
    ocr = 8000000UL/freq - 1;
    freq = 8000000UL/(ocr+1);
  }
  OCR1AH = highByte(ocr);
  OCR1AL = lowByte(ocr);
  TIMSK1 = 0;
  return freq;
} 