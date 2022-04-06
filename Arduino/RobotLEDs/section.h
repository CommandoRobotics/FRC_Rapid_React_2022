// Defines a part of the LED strand that lights up a particular area (i.e. side) of the robot
#ifndef SECTION_H
#define SECTION_H

#include <FastLED.h>

enum class section_names {
  no_section, // For satisfying compiler issues
  left_bumper,
  right_bumper,
  left_vertical,
  left_rear_bottom_stripe,
  left_rear_top_stripe,
  left_gear,
  left_front_top_stripe,
  left_front_bottom_stripe,
  left_void,
  right_void,
  right_front_bottom_stripe,
  right_front_top_stripe,
  right_gear,
  right_rear_top_stripe,
  right_rear_bottom_stripe,
  right_vertical
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
