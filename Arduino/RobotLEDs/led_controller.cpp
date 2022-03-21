#include "led_controller.h"

led_controller::led_controller() :
    total_leds(0),
    leds(nullptr),
    sections(vector_storage),
    connected(connection_state::no_connection),
    alliance(alliance_color::blue),
    match(match_state::not_active),
    shooter(shooter_state::not_ready_to_fire),
    cargo_hound(cargo_hound_state::not_acquired),
    tower_fire_effect(20),
    bumper_fire_effect(32)    
{
    // Bumpers Sections
    // Shooter is front, intake is back
    //        FRONT (shoot this way)
    //    --------------
    // L  |            |  R
    // E  |            |  I
    // F  |            |  G
    // T  |            |  H
    //    |   Intake   |  T
    //    -----    -----
    //        BACK
    sections.push_back(section(section_names::left_bumper, 32)); // Section under left bumper
    sections.push_back(section(section_names::right_bumper, 32)); // Section under right bumper
    // Tower Sections
    //         ______
    //        /      |
    //       /       |
    //      /        |
    //     /         |
    //    |     top  |  (top of tower points up toward shooter)
    //    ============
    // F  |          | B  (sides of tower point in)
    // R  |          | A
    // O  |          | C
    // N  |          | K
    // T  |          |
    sections.push_back(section(section_names::left_back, 19));
    sections.push_back(section(section_names::left_top, 14));
    sections.push_back(section(section_names::left_front, 19));
    sections.push_back(section(section_names::right_back, 19));
    sections.push_back(section(section_names::right_top, 14));
    sections.push_back(section(section_names::right_front, 19));

    total_leds = count_all_leds();
    leds = new CRGB[total_leds];
    
    FastLED.addLeds<WS2811, led_pin>(leds, total_leds);
    FastLED.setBrightness(100); // Turn the LEDs on
  }

led_controller::~led_controller() {
    delete[] leds;
}

// Call this every pass so LEDs update
void led_controller::loop(communications& the_rio) {
  get_states(the_rio);
  apply_base_pattern();
  apply_overlays();
  FastLED.show();
}

size_t led_controller::count_all_leds() {
  size_t total = 0;
  for (auto s : sections) {
    total += s.number_of_leds;
  }
  return total;
}

section_pointer led_controller::get_section_pointer(section_names section_name) {
  size_t pixel_count = 0;
  for (auto current_section : sections) {
    if (current_section.name == section_name) {
      return section_pointer(&leds[pixel_count], current_section.number_of_leds);
    }
    pixel_count += current_section.number_of_leds;
  }
  // Couldn't find the section.
  return section_pointer();
}

size_t led_controller::get_section_length(section_names section_name) {
  for (auto current_section : sections) {
    if (current_section.name == section_name) {
      return current_section.number_of_leds;
    }
  }
  // If it wasn't found, just return none
  return 0;
}

size_t led_controller::get_section_led_index(section_names section_name) {
  size_t pixel_count = 0;
  for (auto current_section : sections) {
    if (current_section.name == section_name) {
      return pixel_count;
    }
    pixel_count += current_section.number_of_leds;
  }
  // It wasn't found
  return 0;
}

void led_controller::get_states(communications the_rio) {
  connected = the_rio.connected();
  alliance = the_rio.color();
  match = the_rio.match();
  shooter = the_rio.shooter();
  cargo_hound = the_rio.cargo_hound();
}

void led_controller::apply_base_pattern() {
  if (connected == connection_state::no_connection) {
    // No Rio, so maybe power just turned on, or we are displaying for judging/outreach
    // Just look pretty
    set_solid_color(CRGB::Blue);
  }
  // Before the match, show a pretty effect. During the match, just use the alliance color.
  if (alliance == alliance_color::blue) {
    if (match == match_state::not_active) {
      blue_waves();
    } else {
      set_solid_color(CRGB::Blue);
    }
  } else {
    if (match == match_state::not_active) {
      fire();
    } else {
      set_solid_color(CRGB::Red);
    }
  }
}

void led_controller::apply_overlays() {
  if (connected == connection_state::no_connection) {
    // No useful information. Don't do anything.
    return;
  }
  cargo_hound_overlay(cargo_hound == cargo_hound_state::acquired);
  shooter_overlay(shooter == shooter_state::ready_to_fire);
}

void led_controller::overlay(const CRGB& color, size_t total_leds_in_section, CRGB* led_section, size_t spacing, size_t offset = 0) {
  for (size_t i = offset; i < total_leds_in_section; i+= spacing) {
    led_section[i] = color;
  }
}

void led_controller::overlay_by_name(const CRGB& color, section_names section_name, size_t spacing, size_t offset = 0) {
  size_t start_index = get_section_led_index(section_name);
  CRGB* led_array = &leds[start_index];
  size_t total_leds_in_section = get_section_length(section_name);
  overlay(color, total_leds_in_section, led_array, spacing, offset);
}

void led_controller::copy(section_pointer destination, section_pointer source) {
  memcpy(destination.start_point, source.start_point, sizeof(CRGB) * destination.length);
}

void led_controller::reverse_copy(section_pointer destination, section_pointer source) {
  for (size_t i = 0; i < destination.length; ++i) {
    size_t destination_index = destination.length - 1 - i; // Subtract one since zero-indexed
    destination.start_point[destination_index] = source.start_point[i];
  }
}

void led_controller::cargo_hound_overlay(bool acquired) {
  CRGB overlay_color = CRGB::White; // Default to not acquired
  if (acquired) {
    overlay_color = CRGB::Purple;
    return; // Leave the animation colors as they are
  }

  overlay_by_name(CRGB::White, section_names::left_top, 4, 0);
  overlay_by_name(CRGB::White, section_names::right_top, 4, 0);
}

void led_controller::shooter_overlay(bool ready) {
  CRGB overlay_color = CRGB::White; // Default to not ready
  if (ready) {
    overlay_color = CRGB::Green;
  }

  overlay_by_name(overlay_color, section_names::left_top, 4, 2);
  overlay_by_name(overlay_color, section_names::right_top, 4, 2);
}


void led_controller::set_section_color(section_names section_name, CRGB color) {
  auto the_section = get_section_pointer(section_name);
  for (size_t i = 0; i < the_section.length; ++i) {
    the_section.start_point[i] = color;
  }
}

void led_controller::set_solid_color(CRGB color) {
  // Dont' worry about sections, just change all leds to blue
  for (size_t i = 0; i < count_all_leds(); ++i) {
    leds[i] = color;
  }
}

void led_controller::blue_waves() {
  wave_effect.loop();
  // Bottom of the robot
  auto right_bottom = get_section_pointer(section_names::right_bumper);
  wave_effect.apply(right_bottom);
  auto left_bottom = get_section_pointer(section_names::left_bumper); 
  // Symmetrically copy to the left side (so they mirror from the center of the "front")
  reverse_copy(left_bottom, right_bottom);

  // Tower - All vertical strings will be the same
  auto left_front = get_section_pointer(section_names::left_front);
  wave_effect.apply(left_front);
  // Copy to the left side
  auto right_front = get_section_pointer(section_names::right_front);
  copy(left_front, right_front);
  // The back strings should be in reverse order of the front ones
  auto left_back = get_section_pointer(section_names::left_back);
  reverse_copy(left_back, left_front);
  auto right_back = get_section_pointer(section_names::right_back);
  reverse_copy(right_back, right_front);
  
  // Shooter
  auto left_top = get_section_pointer(section_names::left_top);
  wave_effect.apply(left_top);
  auto right_top = get_section_pointer(section_names::right_top);
  reverse_copy(right_top, left_top);
}

void led_controller::fire() {
  // Bottom of the robot
  bumper_fire_effect.loop();
  auto right_bottom = get_section_pointer(section_names::right_bumper);
  bumper_fire_effect.apply(right_bottom);
  auto left_bottom = get_section_pointer(section_names::left_bumper); 
  // Symmetrically copy to the left side (so they mirror from the center of the "front")
  reverse_copy(left_bottom, right_bottom);

  // Tower - All vertical strings will be the same
  tower_fire_effect.loop();
  auto left_front = get_section_pointer(section_names::left_front);
  tower_fire_effect.apply(left_front);
  // Copy to the left side
  auto right_front = get_section_pointer(section_names::right_front);
  copy(left_front, right_front);
  // The back strings should be in reverse order of the front ones
  auto left_back = get_section_pointer(section_names::left_back);
  reverse_copy(left_back, left_front);
  auto right_back = get_section_pointer(section_names::right_back);
  reverse_copy(right_back, right_front);

  // Shooter - Horizontal flames here look weird, just make it red.
  set_section_color(section_names::left_top, CRGB::Red);
  set_section_color(section_names::right_top, CRGB::Red);
}
