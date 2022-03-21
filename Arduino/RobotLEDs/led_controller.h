// Encapsulates all leds on the robot and determines how to light them
#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <FastLED.h>
#include <Vector.h>

#include "communications.h"
#include "fire2012.h"
#include "section.h"
#include "waves.h"

class led_controller {
public:

  led_controller();

  ~led_controller();

  // Call this every pass so LEDs update
  void loop(communications& the_rio);

private:
  static const size_t led_pin = 6; // Pin number where the LED signal is connected
  size_t total_leds; // Total number of LEDs to control
  CRGB* leds; // Pointer to the array of LEDs to control
  static const size_t max_number_of_sections = 20; // Arduino doesn't have reall c++ std::vector, so allocate this many objects for their "Vector" class.
  section vector_storage[max_number_of_sections]; // Arduino doesn't have reall c++ std::vector, so allocate this many objects for their "Vector" class.
  Vector<section> sections; // Arduino doesn't have reall c++ std::vector, so allocate this many objects for their "Vector" class.

  connection_state connected;
  alliance_color alliance;
  match_state match;
  shooter_state shooter;
  cargo_hound_state cargo_hound;

  waves wave_effect;
  // Use seperate fire effects for each section, so flames are the correct length
  fire2012 tower_fire_effect;
  fire2012 bumper_fire_effect;

  // Returns the count of all LEDs in all sections
  size_t count_all_leds();

  // Gets the section_pointer object for the section, by name
  section_pointer get_section_pointer(section_names section_name);

  // Gets the total number of leds in a section, by name
  size_t get_section_length(section_names section_name);

  // Finds the section and returns the index number of the first pixel
  size_t get_section_led_index(section_names section_name);

  // Interfaces with Rio to get current robot state
  void get_states(communications the_rio);

  // Applies the appropriate style for the entire robot
  void apply_base_pattern();

  // Sets any status-indicating LEDs
  void apply_overlays();

  // Overlays a color on top of the existing pattern
  void overlay(const CRGB& color, size_t total_leds_in_section, CRGB* led_section, size_t spacing, size_t offset = 0);

  void overlay_by_name(const CRGB& color, section_names section_name, size_t spacing, size_t offset = 0);

  // Copies the LED colors from the first section to the second
  void copy(section_pointer destination, section_pointer source);

  // Copies the LED colors from the first section to the second, but in reverse
  void reverse_copy(section_pointer destination, section_pointer source);

  void cargo_hound_overlay(bool acquired);

  void shooter_overlay(bool ready);

  // Set all lights in the section to the passed color.
  void set_section_color(section_names section_name, CRGB color);

  // Set all lights to the passed color.
  void set_solid_color(CRGB color);

  void blue_waves();

  void fire();

};

#endif
