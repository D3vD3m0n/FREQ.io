# FREQ.io
FREQ.io is a Arduino Libery to easy work with Frequencies und Waves
<p><img src="https://images.squarespace-cdn.com/content/v1/5ad8aa849f8770bce530b0b1/1565971518592-WJAXRNJ0C7Z22CIKOH63/ke17ZwdGBToddI8pDm48kI87z90ccK9kyToEOB_z21MUqsxRUqqbr1mOJYKfIPR7LoDQ9mXPOjoJoqy81S2I8N_N4V1vUb5AoIIIbLZhVYwL8IeDg6_3B-BRuF4nNrNcQkVuAT7tdErd0wQFEGFSnARKyyS8zp7KtsInv_69uSsBrIz-MrCD9I0nwvL6gU0veR3T4Np9pFaMKEmkolIwAg/NFB+Banner.jpeg" alt="foo"  /></p>
/*Frequencies.io library after import. 
 *Frequencies.begin(byte pin, byte hz, unsigned int coolDown);
 *This sets up a Timer Compare Interrupt on Timer1 for logging motion changes. It watches the ADC input defined by pin, does phase cancellation for hz AC to remove sine-wave artifacts from the data, and waits for coolDown milliseconds for the ADC to stabilize before triggering any alarms.
 *Frequencies.end();
 *This clears the Timer1 ISR that checks motion, essentially stopping all Frequencies.io execution.
 *Frequencies.level();
 *Returns the current motion level as a signed integer, with a minor motion giving a value of ~10, and more major motions returning ~100 or more.
 *Frequencies.level() can be both positive or negative depending on if the motion was towards the antenna or away from it! Use abs(Frequencies.io.level()) to get all values as positive.
 *Frequencies.setAlarm(void action, unsigned int threshold, unsigned int hold);
 *Used to set the user-provided function as the callback for an alarm trigger. If motion is >= to threshold, and an alarm hasn't been triggered for hold milliseconds, the function action will be called. If the function you wrote is this helloWorld() demo:
 *void helloWorld(){Serial.println("Hello world!");}
 *You would write setAlarm() like this:
 *Frequencies.setAlarm(helloWorld, 20, 500);
 *A current limitation is that arguments/parameters cannot be passed to the alarm function, and the function can't return data either - though a workaround is to set those values in global variables and read them wherever else your need to, inside or outside the alarm function. See Limitations.
 *Frequencies.checkAlarm();
 *This is used to see if the alarm flag has been set by Frequencies.io. This function should be called as often as possible, and your code should avoid blocking functions like delay(). If the flag has been set true by motion exceding your custom threshold, the function defined in setAlarm() will be called.
 *Frequencies.printData()
 *This renders a graph to the Arduino IDE Serial Plotter containing current motion levels, your threshold for alarm, and marks when alarms were triggered. Nothing will appear in the plotter until the coolDown from Frequencies.io.begin() has passed.
 *Limitations
 *Unfortunately, a solution this simple has it's caveats:
 *Freq is susceptable to false positives
 *Because we're relying on AC and static electricity for our readings, it's trivial to cause interference to the input by turning on power appliances nearby, or picking up local lighting strikes. (Though the latter is a cool use as well!)
 *Freq loves cats
 *While a human like yourself always has enough static charge to cause a measurable shift in ADC voltage, cats are covered in electron-loving fur. It's much more sensitive to them! A solution is to make your cat wear an anti-static soldering strap at all times, but so far I've been very unsuccessful in implementing this.
 *Phase cancellation has only been tested with 60Hz AC
 *While it shouldn't make a huge difference without it, the phase cancellation has been tested for 60Hz AC, and calculated for 50Hz AC. 50Hz users may want to report results back to me if the defaults need changes.
 *Freq disables PWM on Pin 9 and 10
 *Freq uses Hardware Timer 1 to watch motion data "in the background" while your standard code runs. Timer1 is also responsible for PWM on 9 and 10, and it can't do both. If you desperately need to, call Frequencies.io.end(), run your PWM, then Frequencies.io.begin() again.
 *Freq will only work with direct grounding
 *The phenomenon this technique relies on requires the Arduino/ATmega to be fully connected to ground. This means that while it will work on a computer's USB port, it will not work directly on a 5V phone charger/wall adapter due to galvanic isolation. This is solved by tying one of the microcontroller's GND pins to a true ground like your house's grounding, or to the common of an appliance. (E.g. the back of a fridge, or the outside of a metal lamp.
 */
<p><img src="https://img.wavescdn.com/1lib/images/blog/banners/eq-tips-boost-or-cut-the-frequencies/1.jpg" alt="foo"  /></p>

