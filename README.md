# RobotSuit
###Firmware for Robot Kids Costume (Circuit Playground Express + CRICKIT).

*3D printed parts on [Thingiverse](https://www.thingiverse.com/thing:4145990).*

Here is the firmware I used to create a carnival robot custume for my kid.

The "brains" of the robot consist of a Adafruit Crickit connected to an Adafruit Circuit Playground Express (Cortex ARM version).

The Crickit is placed in a case that can take holder for 4xAA batteries (use 1.2V ones), or, if thats too heavy (as my kid said after all was finished), you can of course fit a lithium cell with a 5V boost converter instead (saved about 80g).

The lid of the case comes in two versions, where the one I used for the robot helmet has four lids and can be screwed together so it doesn't fall out of the cardboard box. The one without the ledges could come handy for other projects.

A kind of helmet can be printed that can be attached to the hexagonal case to help the kid keep the cardboard box on its head (the first version didn't have that and the helmet fell over and crashed into the ground, breaking stuff).

The Crickit drives two servos that are taped (duck tape) via a servo holder onto the cardboard box that move "antennas" back and forth. The antennas are actually drinking straws, attached via a servo adapter and a LED holder adapter on the top end, where two big 8mm LEDs are placed. The resistor and the cables for the LEDs go inside the straw and through the servo adapter.

The Crickit drives two gear box motors that are attached to the side of the helmet as kind of radar ears.

A strip of neopixels does some blinkenlights / Knight Rider / Cylone stuff on the front of the helmet. More blinkenlights are available via the built-in 10 neopixels on the Circuit Playground itself.

There's an 8 ohms speaker attached to the front via the Crickit, though the buzzer on the Circuit Playground works too. There's an artsy object to protect the speaker from being touched.

There are two touch pads connected to allow the kid to control the motors and/or toggle the sound effects.

About the used firmware: The ears react to sounds and tilting the head, the antenna LEDs blink in various patterns, the antennas move back and forth on a complex sine pattern, the front Neopixel stripe reacts to tilting the head, and the speaker outputs random robot sounds (10 bit 22.5 KHz).

My son wore it once to a theme birthday party but wants to go as a pirate on carnival instead. Sigh.