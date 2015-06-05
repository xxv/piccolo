/*
PICCOLO is a tiny Arduino-based audio visualizer.

Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Bicolor LED Matrix with I2C Backpack (ID: 902)
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Optional: battery for portable use (else power through USB)
Software requirements:
 - elm-chan's ffft library for Arduino

Connections:
 - 3.3V to mic amp+ and Arduino AREF pin <-- important!
 - GND to mic amp-
 - Analog pin 0 to mic amp output
 - +5V, GND, SDA (or analog 4) and SCL (analog 5) to I2C Matrix backpack

Written by Adafruit Industries.  Distributed under the BSD license --
see license.txt for more information.  This paragraph must be included
in any redistribution.

ffft library is provided under its own terms -- see ffft.S for specifics.
*/

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.

#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>

// Number of LEDs in the strip
#define NUMPIXELS 182

// Microphone connects to Analog Pin 0.  Corresponding ADC channel number
// varies among boards...it's ADC0 on Uno and Mega, ADC7 on Leonardo.
// Other boards may require different settings; refer to datasheet.
#ifdef __AVR_ATmega32U4__
 #define ADC_CHANNEL 7
#else
 #define ADC_CHANNEL 0
#endif

int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter
//uint16_t    peak[FFT_N/2];

static PROGMEM const byte
  palette_flame[] = {
    /*lvl  R     G     B */
     0, 0x00, 0x00, 0x00,
    10, 0x33, 0x00, 0x00,
    20, 0xff, 0x23, 0x00,
    30, 0xff, 0xff, 0x00,
    40, 0xff, 0xff, 0x70,
  };

byte
  peak[64];
byte
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 255,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

void setup() {
  uint8_t i, j, binNum;

  memset(peak, 0, sizeof(peak));

  strip.begin();
  strip.show();

  for (i = 0; i < NUMPIXELS/2; i++){
    strip.setPixelColor(NUMPIXELS/2-i+1, 0x0);
    strip.setPixelColor(NUMPIXELS/2-i, 0xff0000);
    strip.setPixelColor(NUMPIXELS/2+i-1, 0x0);
    strip.setPixelColor(NUMPIXELS/2+i, 0x0000ff);
    strip.show();
    delay(8);
  }
  strip.setPixelColor(0, 0);
  strip.show();

  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
}

uint32_t mask_colors(uint8_t b, uint32_t mask) {
  return (b | (uint32_t)b << 8 | (uint32_t)b << 16) & mask;
}

uint32_t read_color(int x) {
  return (uint32_t)pgm_read_byte(&palette_flame[x + 1]) << 16 |
         (uint32_t)pgm_read_byte(&palette_flame[x + 2]) << 8 |
         (uint32_t)pgm_read_byte(&palette_flame[x + 3]);
}

uint32_t palette(uint8_t level) {
  uint8_t level_a, level_b, N;
  uint32_t color_a, color_b;
  float offset;

  N = sizeof(palette_flame);

  for (int i = 0; i < N; i+=4){
    level_b = pgm_read_byte(&palette_flame[i]);
    color_b = read_color(i);

    if (level == level_b || (level >= level_b && i == (N - 4))) {
      return color_b;
    }

    if (i > 0) {
      if (level < level_b) {
        return
               (uint32_t)map(level, level_a, level_b,
                   (color_a >> 16) & 0xff, (color_b >> 16) & 0xff) << 16 |
               (uint32_t)map(level, level_a, level_b,
                   (color_a >> 8) & 0xff, (color_b >> 8) & 0xff) << 8 |
               (uint32_t)map(level, level_a, level_b,
                   color_a & 0xff, color_b & 0xff);
      }
    }

    level_a = level_b;
    color_a = color_b;
  }
}

void flame_peaks(){
  for(uint8_t x=0; x<FFT_N/2; x++) {
    if (peak[x] < 5) {
      strip.setPixelColor(x * 2, 0);
      strip.setPixelColor(x * 2 + 1, 0);
    } else {
      strip.setPixelColor(x * 2, palette(peak[x]));
      strip.setPixelColor(x * 2 + 1, palette(peak[x]));
    }
  }
}

void white_peaks(){
  for(uint8_t x=0; x<FFT_N/2; x++) {
    if (peak[x] < 10) {
      strip.setPixelColor(x * 2, 0);
      strip.setPixelColor(x * 2 + 1, 0);
    } else {
      strip.setPixelColor(x * 2, mask_colors(peak[x], 0xffffff));
      strip.setPixelColor(x * 2 + 1, mask_colors(peak[x], 0xffffff));
    }
  }
}

void white_sparkle(){
  for(uint8_t x=0; x<FFT_N/2; x++) {
    if (spectrum[x] < 10) {
      strip.setPixelColor(x * 2, 0);
      strip.setPixelColor(x * 2 + 1, 0);
    } else {
      strip.setPixelColor(x * 2, mask_colors(spectrum[x], 0xffffff));
      strip.setPixelColor(x * 2 + 1, mask_colors(spectrum[x], 0xffffff));
    }
  }
}

uint8_t test_i;
void loop() {
  uint8_t  i, x, L, binNum, c, ranged;
  uint16_t minLvl, maxLvl;
  int      level, y, sum;

  while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise[x]);
    spectrum[x] = (spectrum[x] <= L) ? 0 :
      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
    ranged = spectrum[x] & 0xff;
    if(ranged > peak[x]) {
      peak[x] = ranged;
    }
    /*if (maxLvl < spectrum[x]){
      maxLvl = spectrum[x];
    }
    */
  }


  flame_peaks();
  //white_sparkle();

  strip.show();
  delay(10);

  // Every third frame, make the peak pixels drop by 1:
  if(++dotCount >= 3) {
    dotCount = 0;
    for(x=0; x<FFT_N/2; x++) {
      if(peak[x] > 0) peak[x]--;
    }
  }

  if(++colCount >= 10) colCount = 0;
  maxLvl = 0;
}

ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023

  capture[samplePos] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}

