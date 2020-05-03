 /*
 *
 */
 
#include <Arduino.h>
#include "Frequencies.h"

Frequencies Freq(3,5,0.2);

void setup() {
  Freq.init();
}

void loop() {
  Freq.run();
}