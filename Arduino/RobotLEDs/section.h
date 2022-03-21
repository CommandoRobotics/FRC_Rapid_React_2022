// Defines a part of the LED strand that lights up a particular area (i.e. side) of the robot
#ifndef SECTION_H
#define SECTION_H

#include <FastLED.h>

enum class section_names {
  no_section, // For satisfying compiler issues
  left_bumper,
  right_bumper,
  left_front,
  left_top,
  left_back,
  right_back,
  right_top,
  right_front
};

struct section_pointer {
  CRGB* start_point;
  size_t length;

  section_pointer(CRGB* pointer, size_t pixels) :
    start_point(pointer),
    length(pixels)
    { }

  section_pointer() :
    start_point(nullptr),
    length(0)
  { }
};

struct section {
  section_names name; // Name of this section
  size_t number_of_leds; // How many LEDs are in this section

  section(section_names section_name, size_t section_led_count) :
    name(section_name),
    number_of_leds(section_led_count)
   { }

  section() :
    name(section_names::no_section),
    number_of_leds(0)
   { }
};

#endif
