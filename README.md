# Arduino Sketch for DotStar Based Microscope Ring Light

This Arduino sketch is for a two-knob intensity/color controller for a microscope or photographic ring light
composed of a strip of DotStar RGB LEDs.

Only two GPIO outputs and two analog inputs are needed on the Arduino, and the code needs less than 500 bytes of dynamic
memory, so this sketch should run on virtually any Arduino whatsoever.

A full description of the project, with printable STL files for a microscope ring light housing and a link to the CAD files are over on
Thingiverse in [this Thing](https://www.thingiverse.com/thing:5150894)

Filtering is done on the analog inputs to tamp down noise and prevent the brightness and color
from changing unexpectedly.  The ring-buffer based filtering actually accounts for most of the code.

## Dependencies

*  FastLED library
