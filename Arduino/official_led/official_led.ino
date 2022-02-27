#include <Adafruit_NeoPixel.h>
#define LEFT_SHOOTER_PIN 4
#define RIGHT_SHOOTER_PIN 5
#define LEFT_INDEX_PIN 6
#define RIGHT_INDEX_PIN 7
#define RED_ALLIANCE_PIN 8
#define AUTO_AIM_LOCKED_PIN 9
#define NUM_PIXELS 60
#define FLASH_DELAY 250

Adafruit_NeoPixel leftShooter(NUM_PIXELS, LEFT_SHOOTER_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rightShooter(NUM_PIXELS, RIGHT_SHOOTER_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leftIndex(NUM_PIXELS, LEFT_INDEX_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rightIndex(NUM_PIXELS, RIGHT_INDEX_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  leftShooter.begin();
}

void loop() {
  if(digitalRead(RED_ALLIANCE_PIN) == LOW) {
    // Blue Alliance
    if(digitalRead(AUTO_AIM_LOCKED_PIN) == HIGH) {
      // Auto aim is locked
      leftShooter.clear();
      leftShooter.show();
      delay(FLASH_DELAY);
      for(int i = 0; i < NUM_PIXELS; i++) {
        leftShooter.setPixelColor(i, leftShooter.Color(0,0,255));
      }
      leftShooter.show();
      delay(FLASH_DELAY);
    } else {
      for(int i = 0; i < NUM_PIXELS; i++) {
        leftShooter.setPixelColor(i, leftShooter.Color(0,0,255));
      }
      leftShooter.show();
    }
  } else {
    if(digitalRead(AUTO_AIM_LOCKED_PIN) == HIGH) {
      // Auto aim is locked
      leftShooter.clear();
      leftShooter.show();
      delay(FLASH_DELAY);
      for(int i = 0; i < NUM_PIXELS; i++) {
        leftShooter.setPixelColor(i, leftShooter.Color(255,0,0));
      }
      leftShooter.show();
      delay(FLASH_DELAY);
    } else {
      for(int i = 0; i < NUM_PIXELS; i++) {
        leftShooter.setPixelColor(i, leftShooter.Color(255,0,0));
      }
      leftShooter.show();
  }
}
}
