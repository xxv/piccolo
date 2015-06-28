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
#include <Adafruit_DotStar.h>
#include <SPI.h>

// Number of LEDs in the strip
#define NUMPIXELS 128
#define CHASE_LEN 128

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

#define MIN_LEVEL 5
#define MIN_SPECTRUM_BIN 8
#define END_PALETTE 255


static PROGMEM const byte
  palette_test[] = {
     0, 0x00, 0x00, 0x00,
    10, 0x00, 0x00, 0xff,
    20, 0x00, 0xff, 0x00,
    30, 0xff, 0x00, 0x00,
    END_PALETTE
  },
  palette_flame[] = {
    /*lvl  R     G     B */
     0, 0x00, 0x00, 0x00,
    10, 0x33, 0x00, 0x00,
    20, 0xff, 0x23, 0x00,
    30, 0xff, 0xff, 0x00,
    40, 0xff, 0xff, 0x70,
    END_PALETTE
  },
  palette_matrix[] = {
     0, 0x00, 0x00, 0x00,
    10, 0x00, 0xff, 0x00,
    40, 0xff, 0x00, 0x00,
    END_PALETTE
  },
  palette_rainbow[] = {
     0, 0x00, 0x00, 0x00,
     5, 0xff, 0x00, 0x00, // R
    10, 0xff, 0x92, 0x00, // O
    15, 0xff, 0xff, 0x00, // Y
    20, 0x00, 0xff, 0x00, // G
    30, 0x00, 0x00, 0xff, // B
    40, 0x50, 0x00, 0xff, // V
    END_PALETTE
  },
  palette_teal[] = {
    /*lvl  R     G     B */
     0, 0x00, 0x00, 0x00,
    10, 0x00, 0x9b, 0x9b,
    35, 0x00, 0xff, 0xff,
    40, 0xaf, 0xff, 0xff,
    END_PALETTE
  },
  palette_red[] = {
     0, 0x00, 0x00, 0x00,
    40, 0xff, 0x00, 0x00,
    END_PALETTE
  },
  palette_purple[] = {
     0, 0x00, 0x00, 0x00,
    40, 0xff, 0x00, 0xff,
    END_PALETTE
  },
  palette_blue[] = {
     0, 0x00, 0x00, 0x00,
    40, 0x00, 0x00, 0xff,
    END_PALETTE
  },
  palette_white[] = {
     0, 0x00, 0x00, 0x00,
    40, 0xff, 0xff, 0xff,
    END_PALETTE
  },
  palette_sky[] = {
     0, 0x00, 0x00, 0xaa,
    20, 0x00, 0x00, 0xaa,
    30, 0xaa, 0xaa, 0xff,
    END_PALETTE
  };


static const byte* palettes[] = {
  palette_flame,
  palette_rainbow,
  palette_sky,
  palette_red,
  palette_matrix,
  palette_teal,
  palette_blue,
  palette_purple,
  palette_white,
  0
};

#define DECAY 1
byte peak[64];

byte chase[CHASE_LEN];
uint8_t chase_offset = 0, chase_slowdown = 0;

byte
  dotCount = 0; // Frame counter for delaying dot-falling speed

#define BUTTON_COUNT 5
#define DEBOUNCE_TIME 3

unsigned long button_down_time[BUTTON_COUNT] = {0,0,0,0,0};
bool button_last_state[BUTTON_COUNT] = {0,0,0,0,0};
bool button_state[BUTTON_COUNT] = {1,1,1,1,1};
uint8_t buttons[] = {
  A1,
  A2,
  A3,
  A4,
  A5,
  };

#define BTN_PALETTE 0
#define BTN_MODE 1
#define BTN_RESET 4

#define MODE_COUNT 4
uint8_t mode = 0;

uint8_t palette = 0;


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
    255, 255,100,100,100,100,100, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

void setup() {
  uint8_t i, j;

  memset(peak, 0, sizeof(peak));
  memset(chase, 0, sizeof(chase));

  // Using A7 as a ground for buttons
  pinMode(A7, OUTPUT);
  digitalWrite(A7, 0);

  // A1-A5 are button inputs
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

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

  digitalWrite(A7, 0);
  sei(); // Enable interrupts
}

uint32_t mask_colors(uint8_t b, uint32_t mask) {
  return (b | (uint32_t)b << 8 | (uint32_t)b << 16) & mask;
}

uint32_t read_color(const byte* pal) {
  return (uint32_t)pgm_read_byte(&pal[1]) << 16 |
         (uint32_t)pgm_read_byte(&pal[2]) << 8 |
         (uint32_t)pgm_read_byte(&pal[3]);
}

uint32_t flat_palette(const byte* pal, uint8_t level) {
  uint8_t level_a, level_b, N;
  uint32_t color_a, color_b;
  float offset;

  for (int i = 0; i < 255; i+=4){
    level_b = pgm_read_byte(&pal[i]);
    if (level_b == END_PALETTE) {
      break;
    }

    color_b = read_color(&pal[i]);

    if (level == level_b || (level >= level_b && i == (N - 4))) {
      return color_b;
    }

    if (i > 0) {
      if (level < level_b) {
        return color_a;
      }
    }

    level_a = level_b;
    color_a = color_b;
  }
}


uint32_t smooth_palette(const byte* pal, uint8_t level) {
  uint8_t level_a, level_b;
  uint32_t color_a, color_b, color = 0;
  float offset;

  for (int i = 0; i < 255; i+=4) {
    level_b = pgm_read_byte(&pal[i]);
    if (level_b == END_PALETTE) {
      color = color_b;
      break;
    }

    color_b = read_color(&pal[i]);

    if (level == level_b) {
      color = color_b;
    } else if (i > 0) {
      if (level < level_b) {
        color =
               (uint32_t)map(level, level_a, level_b,
                   (color_a >> 16) & 0xff, (color_b >> 16) & 0xff) << 16 |
               (uint32_t)map(level, level_a, level_b,
                   (color_a >> 8) & 0xff, (color_b >> 8) & 0xff) << 8 |
               (uint32_t)map(level, level_a, level_b,
                   color_a & 0xff, color_b & 0xff);
        break;
      }
    }

    level_a = level_b;
    color_a = color_b;
  }

  return color;
}

void single_bar(const byte* pal) {
  uint16_t level = get_representative_level();

  for (uint8_t x=0; x < NUMPIXELS; x++) {
    if (x <= level) {
      strip.setPixelColor(x, smooth_palette(pal, x/3));
    } else {
      strip.setPixelColor(x, 0);
    }
  }
}

void smooth_peaks(const byte* pal){
  for(uint8_t x=0; x<FFT_N/2; x++) {
    if (peak[x] < MIN_LEVEL) {
      strip.setPixelColor(x * 2, 0);
      strip.setPixelColor(x * 2 + 1, 0);
    } else {
      strip.setPixelColor(x * 2, smooth_palette(pal, peak[x]));
      strip.setPixelColor(x * 2 + 1, smooth_palette(pal, peak[x]));
    }
  }
}

void sparkle(const byte* pal) {
  for(uint8_t x=0; x<FFT_N/2; x++) {
    if (spectrum[x] < 5) {
      strip.setPixelColor(x * 2, 0);
      strip.setPixelColor(x * 2 + 1, 0);
    } else {
      strip.setPixelColor(x * 2, smooth_palette(pal, spectrum[x]));
      strip.setPixelColor(x * 2 + 1, smooth_palette(pal, spectrum[x]));
    }
  }
}

uint8_t get_peak_bin() {
  uint8_t peak_bin, x;

  for(x=0; x<FFT_N/2; x++) {
    if (spectrum[x] > spectrum[peak_bin]) {
      peak_bin = x;
    }
  }

  return peak_bin;
}

uint8_t get_representative_level() {
  uint16_t average;
  uint8_t x, col_count = 0;


  for(x=0; x<FFT_N/2; x++) {
    if (spectrum[x] >= MIN_SPECTRUM_BIN) {
      average += spectrum[x];
      col_count++;
    }
  }

  if (col_count == 0) {
    return 0;
  } else {
    return average/col_count;
  }
}


void chase_pgm(const byte* pal) {
  uint8_t peak_bin, y, x;

  if (chase_slowdown == 0){
    peak_bin = get_peak_bin();

    if (peak_bin == 0 || spectrum[peak_bin] < 10) {
      chase[chase_offset] = 0;
    } else {
      //chase[chase_offset] = spectrum[peak_bin] & 0xff;
      chase[chase_offset] = peak_bin;
    }

    for (x=0; x<CHASE_LEN; x++) {
      y = (chase_offset + CHASE_LEN - x) % CHASE_LEN;
      strip.setPixelColor(x, smooth_palette(pal, chase[y]));
    }

    chase_offset = (chase_offset + 1) % CHASE_LEN;
  }

  chase_slowdown = (chase_slowdown + 1) % 1;
}

void reset_leds() {
  for (uint8_t x=0; x < NUMPIXELS; x++) {
    strip.setPixelColor(x, 0);
  }
}

void on_button_change(uint8_t button, bool value) {

  if (value == LOW) {
    switch (button) {
      case BTN_MODE:
        reset_leds();
        mode = (mode + 1) % MODE_COUNT;
        break;

      case BTN_PALETTE:
        palette++;
        if (palettes[palette] == 0) {
          palette = 0;
        }
        break;

      case BTN_RESET:
        mode = 0;
        palette = 0;
        reset_leds();
        break;
    }
  }
}

void update_buttons() {
  static int counter = 0;
  counter++;

  for (uint8_t i=0; i < BUTTON_COUNT; i++) {
    bool raw = digitalRead(buttons[i]);

    if (raw != button_last_state[i]) {
      button_down_time[i] = counter;
    }

    if ((counter - button_down_time[i]) > DEBOUNCE_TIME) {

      // State change
      if (raw != button_state[i]) {
        button_state[i] = raw;

        on_button_change(i, raw);
      }
    }
    button_last_state[i] = raw;
  }
}

uint8_t test_i;
void loop() {
  uint8_t  i, x, L, c, ranged;
  int      level, y;

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
  }

  update_buttons();

  switch (mode) {
    case 0:
      smooth_peaks(palettes[palette]);
      break;
    case 1:
      sparkle(palettes[palette]);
      break;
    case 2:
      chase_pgm(palettes[palette]);
      break;
    case 3:
      single_bar(palettes[palette]);
      break;
    }

  strip.show();

  // Every third frame, make the peak pixels drop by 1:
  if(++dotCount >= 3) {
    dotCount = 0;
    for(x=0; x<FFT_N/2; x++) {
      if(peak[x] >= DECAY) {
        peak[x] -= DECAY;
      }
    }
  }
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

