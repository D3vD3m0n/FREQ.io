 /*
 * connect a wire/jumper (6-12") to pin A0, and open the Arduino IDE's Serial Plotter to see the current motion value! 
 * Try waving your hand near the antenna, or walking past it.
 */
#include <Arduino.h>
#include "Frequencies.h"

Frequencies Freq(3,5,0.2);

void setup() {
  Serial.begin(115200);
  Freq.init(A0,60,3000);
}

void loop() {
  Freq.run();
  Serial.println(Freq.level());
  delay(1);
}

 /*Functions
 *
 *Buzz buzz;
 *This initializes the Buzz library after import. "Freq" can be any word you want, as long as it's reflected in the rest of your code.
 *
 *Freq.begin(byte pin, byte hz, unsigned int coolDown);
 *
 *This sets up a Timer Compare Interrupt on Timer1 for logging motion changes. It watches the ADC input defined by pin, does phase cancellation for hz AC to remove sine-wave artifacts from the data, and waits for coolDown milliseconds for the ADC to stabilize before triggering any alarms.
 *
 *Freq.end();
 *
 *This clears the Timer1 ISR that checks motion, essentially stopping all Buzz execution.
 *
 *Freq.level();
 *
 *Returns the current motion level as a signed integer, with a minor motion giving a value of ~10, and more major motions returning ~100 or more.
 *
 *Freq.level() can be both positive or negative depending on if the motion was towards the antenna or away from it! Use abs(buzz.level()) to get all values as positive.
 *
 *Freq.setAlarm(void action, unsigned int threshold, unsigned int hold);
 *
 *Used to set the user-provided function as the callback for an alarm trigger. If motion is >= to threshold, and an alarm hasn't been triggered for hold milliseconds, the function action will be called. If the function you wrote is this helloWorld() demo:
 *
 *void helloWorld(){
 *  Serial.println("Hello world!");
 *}
 *
 *You would write setAlarm() like this:
 *Freq.setAlarm(helloWorld, 20, 500);
 *A current limitation is that arguments/parameters cannot be passed to the alarm function, and the function can't return data either - though a workaround is to set those values in global variables and read them wherever else your need to, inside or outside the alarm function. See Limitations.
 *
 *Freq.checkAlarm();
 *
 *This is used to see if the alarm flag has been set by Buzz. This function should be called as often as possible, and your code should avoid blocking functions like delay(). If the flag has been set true by motion exceding your custom threshold, the function defined in setAlarm() will be called.
 *
 *Freq.printData()
 *
 *This renders a graph to the Arduino IDE Serial Plotter containing current motion levels, your threshold for alarm, and marks when alarms were triggered. Nothing will appear in the plotter until the coolDown from buzz.begin() has passed.
 *Limitations
 *
 *Unfortunately, a solution this simple has it's caveats:
 *
 *Freq is susceptable to false positives
 *
 *Because we're relying on AC and static electricity for our readings, it's trivial to cause interference to the input by turning on power appliances nearby, or picking up local lighting strikes. (Though the latter is a cool use as well!)
 *
 *Freq loves cats
 *
 *While a human like yourself always has enough static charge to cause a measurable shift in ADC voltage, cats are covered in electron-loving fur. It's much more sensitive to them! A solution is to make your cat wear an anti-static soldering strap at all times, but so far I've been very unsuccessful in implementing this.
 *
 *Phase cancellation has only been tested with 60Hz AC
 *
 *While it shouldn't make a huge difference without it, the phase cancellation has been tested for 60Hz AC, and calculated for 50Hz AC. 50Hz users may want to report results back to me if the defaults need changes.
 *
 *Freq disables PWM on Pin 9 and 10
 *
 *Freq uses Hardware Timer 1 to watch motion data "in the background" while your standard code runs. Timer1 is also responsible for PWM on 9 and 10, and it can't do both. If you desperately need to, call buzz.end(), run your PWM, then buzz.begin() again.
 *
 *Freq will only work with direct grounding
 *
 *The phenomenon this technique relies on requires the Arduino/ATmega to be fully connected to ground. This means that while it will work on a computer's USB port, it will not work directly on a 5V phone charger/wall adapter due to galvanic isolation. This is solved by tying one of the microcontroller's GND pins to a true ground like your house's grounding, or to the common of an appliance. (E.g. the back of a fridge, or the outside of a metal lamp.
 */