// Arduino sketch for Adafruit Metro Mini (UNO board type) driving DotStar high density
// LED strip (max 60 mA per RGB pixel) for microscope ringlight.
//
// Copyright by Dave Cook caveduck17@gmail.com, Nov 2021.
//
// Features
//    Uses only 2 discrete outputs and 2 analog inputs, should run on virtually any Arduino
//    Powered from micro-USB port
//      Output brightness limited by upstream USB 5V supply.  Soft limits are compile time selectable.
//        On USB hub from computer you can only rely on about 500 mA, so must not exceed ~20% white brightness
//        Powered from Adafruit 2.5V wall adapter you should be able to run the 40-LED strip at full power (2.4A nominal).
//    Brightness control knob, far CCW position for LEDs off
//    Hue control knob (white center with deadband?  exact UX is TBD)
//    Smooth control response, low flicker

#include <FastLED.h>

// HW configuration

#define NUM_LEDS 40

#define DATA_PIN 3                  // green wire on DotStar strips
#define CLOCK_PIN 13                // yellow wire on DotStar strips
#define ANALOG_COLOR_PIN      A4    // color knob input pin
#define ANALOG_BRIGHTNESS_PIN A5    // brightness knob input pin

// SW options

#define HSV_LIMIT_V       0          // 1 to clip V value to max_bright when in HSV mode.  Careful with that axe...

// DEBUG slows down refresh rate a lot, to allow some serial output inside the refresh loop
// It also always sets LOW_POWER since you're assumed to be using the computer and likely a USB hub
#define DEBUG 0
#define SHOW_ANALOG_IN 0            // 1 to print analog raw inputs
#define SHOW_ANALOG_FILTERED 0      // 1 to print filtered analog inputs
#define PRINT_COLORS 0              // 1 to print CRGB or CSHV color components when computed

#if DEBUG
#define REFRESH_RATE 20             // slow down to keep serial output reasonable
#define LOW_POWER 1                 // always limit current draw to hub-suitable levels
#else
#define REFRESH_RATE 200            // 200 Hz gives very smooth response
#define LOW_POWER 0                 // still low power for safety while testing often
#endif

// startup delay gives a window to enter the bootloader in case the default config is eating too
// much power and tripping overcurrent protection
#define STARTUP_DELAY 2000          // initialization wait in msec

#define ANALOG_IN_SMOOTHING_SAMPLES     12   // number of analog input samples to average in ring buffer
#define COLOR_INPUT_THRESH              25   // max ADC value seen when color pot is full CCW, interacts with power


// dim white for testing
const CRGB dimwhite = 0x020202;

// brightness upper/lower limits for current draw control
const int min_bright = 0x00;  // for testing, use 0 for production so we can turn it off
#if LOW_POWER
const int max_bright = 0x20;  // OK for typical USB hub at 500 mA of 5V
#else
const int max_bright = 0xFF;  // requires 2.5A USB 5V supply
#endif

const int refresh_delay = 1000 / REFRESH_RATE;

// --------------------------------------------------------
// Buffer and low-pass filter class for analog inputs
// --------------------------------------------------------
// Provides reading, storing and moving-average LPF functions for analog inputs.
// Uses the Arduino built-in function analogRead() for reading the inputs.
// The LPF is currently a simple trailing-N-samples average.
// Sampling is triggered by the caller; there is no async timed sampling logic in this class.

#define ANALOG_RINGBUF_MAX_SIZE ANALOG_IN_SMOOTHING_SAMPLES
#define SHOW_RING_SAMPLES   0

class AnalogRingbuf {
  int ringptr;
  int ringsize;
  int numvalid;
  int curval;
  int pin;
  char *tag;
  int samples[ANALOG_RINGBUF_MAX_SIZE];

  public:
  AnalogRingbuf(char *tag, int size, int pin) {
    this->ringptr = 0;
    this->numvalid = 0;
    this->tag = tag;
    this->ringsize = size;
    this->pin = pin;
    for (int i = 0; i < size; i++)
      this->samples[i] = 0;
    this->curval = 0;
  }

  int computeAvg() {
    long sum = 0;      
    for (int i = 0; i < this->numvalid; i++)
      sum += this->samples[i];
    this->curval = (sum + this->numvalid/2) / this->numvalid;
    return this->curval;
  }

  void readSample() {
    int val = analogRead(this->pin);
    #if SHOW_ANALOG_IN
    Serial.print("Analog in ("); Serial.print(this->tag), Serial.print(") ");Serial.println(val, HEX);
    #endif    
    this->addSample(val);   
  }

  // addSample could be used if you are obtaining samples from somewhere else than the Arduino analog inputs.
  // Design decision was to recompute the filtered value here instead of on-demand in the accessor,
  // since it will make CPU usage more predictable
  void addSample(int sample) {
    this->samples[(this->ringptr)++] = sample;
    if (this->ringptr >= this->ringsize)
      this->ringptr = 0;
    if (this->numvalid < this->ringsize)
      this->numvalid++;    
    this->computeAvg();
    #if SHOW_RING_SAMPLES
    Serial.print("Vals ("); Serial.print(this->tag); Serial.print(") Ringsize: "); Serial.println(this->ringsize);
    for (int i = 0; i < this->ringsize; i++) {
      Serial.println(this->samples[i], HEX);
    }
    #endif
    #if SHOW_ANALOG_FILTERED
    Serial.print("Filt ("); Serial.print(this->tag); Serial.print(") "); Serial.println(this->curval, HEX);
    #endif
  }

  int getFilteredValue() {
    return this->curval;
  }

};

typedef enum opMode {
  RGB_MODE,
  HSV_MODE
};

opMode g_opmode = RGB_MODE;

CRGB leds[NUM_LEDS];          // LEDs array for the FastLED library

// Ringbuffer/filters for analog inputs
AnalogRingbuf* g_brt_ringbuf = new AnalogRingbuf("BRT", ANALOG_IN_SMOOTHING_SAMPLES, ANALOG_BRIGHTNESS_PIN);
AnalogRingbuf* g_color_ringbuf = new AnalogRingbuf("COL", ANALOG_IN_SMOOTHING_SAMPLES, ANALOG_COLOR_PIN);

// init routine - name is reserved
void setup() {
    // maximal debug output speed
    Serial.begin(115200);

  	// startup delay - allows reprogramming if accidently overcurrent at boot
   	delay(STARTUP_DELAY);

    // Adafruit DotStar full-size strips are SK9822, with clock (yellow) and data (green) lines
    // Library CRGB object always uses RGB order, we need BGR output order specified in the FastLED init
    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);  // BGR ordering
}

// main loop

// primitives for loading the pixel array and writing it out to the LEDs

void loadLedsRGB(CRGB lcolor) {
  for(int led = 0; led < NUM_LEDS; led += 1) {
    leds[led] = lcolor;
  }
}

void loadLedsHSV(CHSV lcolor) {
  for(int led = 0; led < NUM_LEDS; led += 1) {
    leds[led] = lcolor;
  }
}

void showPixels() {
  FastLED.show();  
}

// Original example - almost the simplest possible moving-pixel updater
void doMovingPixels() {
  for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
    // Set current led on, show LEDs, then turn it off again
    leds[whiteLed] = dimwhite;
    FastLED.show();
    delay(refresh_delay);
    leds[whiteLed] = CRGB::Black;
  }
}

CHSV computeColorHSV(int bright_in, int color_in) {
  
  uint8_t hsv_h = map(color_in, COLOR_INPUT_THRESH, 1023, 0, 127);     // hue goes in a circle, so we only need half the domain
  uint8_t hsv_s = map(bright_in, 0, 1023, 127, 255);  // put a floor on the saturation so the color moves enough
  uint8_t hsv_v = map(bright_in, 0, 1023, 0, 255);

#if HSV_LIMIT_V
  // clamping v is not the same as doing it in RGB mode, too drastic 
  if (hsv_v > max_bright)
    hsv_v = max_bright;
  if (hsv_v < min_bright)
    hsv_v = min_bright;
#endif    

  CHSV color = CHSV(hsv_h, hsv_s, hsv_v);

  #if PRINT_COLORS
  Serial.print("Hue "); Serial.print(color.h, HEX); Serial.print(" s "); Serial.print(color.s, HEX); 
  Serial.print(" v "); Serial.print(color.v, HEX); Serial.println();
  #endif

  return color;
  
}

// computes pixel color value from brightness and color input knob values
CRGB computeColorRGB(int bright, int usercolor) {

  // white brightness generation
  int brightval = map(bright, 0, 1023, 0, max_bright);

  int red = brightval;
  int green = brightval;
  int blue = brightval;

  // TBD: tilt the colors, letting nothing exceed brightval.
  
  // safety dance
  // *** check to see if FastLED has a clamping function ***
  // could just clamp brightval if we don't implement color changes here
  red = (red > max_bright) ? max_bright : red;
  red = (red < min_bright) ? min_bright : red;
  
  green = (green > max_bright) ? max_bright : green;
  green = (green < min_bright) ? min_bright : green;
  
  blue = (blue > max_bright) ? max_bright : blue;
  blue = (blue < min_bright) ? min_bright : blue;
  
  CRGB retv;
  retv.setRGB(red, green, blue);
  
  #if PRINT_COLORS
  Serial.print("R "); Serial.print(retv.r, HEX);
  Serial.print(" G "); Serial.print(retv.g, HEX);
  Serial.print(" B "); Serial.println(retv.b, HEX);
  #endif
  
  return retv;
}

// Process the analog inputs to buffer and do a simple low-pass smoothing filter
//  We get 10-bit analog readings from 0-1023
// if left floating they read 285-310 or so
// There is significant noise in the low bits of 3+ LSBs, need to check signal with scope
// Possible interaction with serial port activity...measuring interferes with the measurement :)
void read_and_filter_analog_inputs() {
  g_brt_ringbuf->readSample();
  g_color_ringbuf->readSample();
}

// set operating mode based on knob inputs
//   color input == 0 (full CCW):  RGB_MODE
//   color input nonzero: HSV_MODE
//   color input full CW:  HSV fixed hue submode (faux-white)
void set_opmode() {
  // the color input is affected somewhat by the brightness input, wtf
  if (g_color_ringbuf->getFilteredValue() <= COLOR_INPUT_THRESH)
    g_opmode = RGB_MODE;
  else
    g_opmode = HSV_MODE;  
}

// main loop - the name is reserved

void loop() {
  read_and_filter_analog_inputs();
  set_opmode();

  if (g_opmode == HSV_MODE) {
      Serial.print("Opmode: "); Serial.println("HSV");
      CHSV hsvcolor = computeColorHSV(g_brt_ringbuf->getFilteredValue(), g_color_ringbuf->getFilteredValue());
      loadLedsHSV(hsvcolor);
  }
  else if (g_opmode == RGB_MODE) {
      Serial.print("Opmode: "); Serial.println("RGB");
      CRGB rgbcolor = computeColorRGB(g_brt_ringbuf->getFilteredValue(), g_color_ringbuf->getFilteredValue());
      loadLedsRGB(rgbcolor);
  }
  showPixels();
  delay(refresh_delay);
  
  //doMovingPixels();
}
