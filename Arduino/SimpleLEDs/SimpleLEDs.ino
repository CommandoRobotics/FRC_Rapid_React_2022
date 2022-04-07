#include <FastLED.h>
#define ALLIANCE_COLOR_PIN 12
#define SHOOTER_PIN 9
#define LED_PIN     6
#define NUM_LEDS    30

#define BRIGHTNESS  50

CRGB leds[NUM_LEDS];

void setup() {
  delay(3000); // sanity delay

  pinMode(SHOOTER_PIN, INPUT_PULLUP);
  pinMode(ALLIANCE_COLOR_PIN, INPUT_PULLUP);

  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  if(digitalRead(SHOOTER_PIN) == LOW) {
    set_led_colors(CRGB::Green);
  } else if(digitalRead(ALLIANCE_COLOR_PIN) == LOW) {
    set_led_colors(CRGB::Red);
  } else {
    set_led_colors(CRGB::Blue);
  }
  
  FastLED.show(); // display this frame
}

void set_led_colors(CRGB color) {
  for (size_t i = 0; i < NUM_LEDS; ++i) {
    leds[i] = color;
  }
}
