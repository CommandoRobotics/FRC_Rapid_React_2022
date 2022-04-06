#include "led_controller.h"

led_controller::led_controller() :
    total_leds(0),
    leds(nullptr),
    sections(vector_storage),
    connected(connection_state::no_connection),
    alliance(alliance_color::blue),
    match(match_state::not_active),
    shooter(shooter_state::not_ready_to_fire),
    cargo_hound(cargo_hound_state::not_acquired)
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
    sections.push_back(section(section_names::left_bumper, 2)); // Section under left bumper
    sections.push_back(section(section_names::right_bumper, 2)); // Section under right bumper
//    sections.push_back(section(section_names::left_bumper, 32)); // Section under left bumper
//    sections.push_back(section(section_names::right_bumper, 32)); // Section under right bumper
    // Tower Sections
    //         ______
    //        /      |
    //       /       |
    //      /== C == |
    //     /         |
    //    |     top  |  (top of tower points up toward shooter)
    //    ============
    // F  |          | B  (sides of tower point in)
    // R  |          | A
    // O  |          | C
    // N  |          | K
    // T  |          |
    sections.push_back(section(section_names::left_vertical, 5));
    sections.push_back(section(section_names::left_rear_bottom_stripe, 2));
    sections.push_back(section(section_names::left_rear_top_stripe, 2));
    sections.push_back(section(section_names::left_gear, 11));
    sections.push_back(section(section_names::left_front_top_stripe, 2));
    sections.push_back(section(section_names::left_front_bottom_stripe, 2));
    sections.push_back(section(section_names::left_void, 2));
    sections.push_back(section(section_names::right_void, 2));
    sections.push_back(section(section_names::right_front_bottom_stripe, 2));
    sections.push_back(section(section_names::right_front_top_stripe, 2));
    sections.push_back(section(section_names::right_gear, 11));
    sections.push_back(section(section_names::right_rear_top_stripe, 2));
    sections.push_back(section(section_names::right_rear_bottom_stripe, 2));
    sections.push_back(section(section_names::right_vertical, 5));

    total_leds = count_all_leds();
    leds = new CRGB[total_leds];
    
    FastLED.addLeds<WS2811, led_pin, GRB>(leds, total_leds);
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
    team_livery();
    return;
  }
  
  // Before the match, show a pretty effect. During the match, just use the alliance color.
  if (alliance == alliance_color::blue) {
    if (match == match_state::not_active) {
      blue_animation();
    } else {
      set_solid_color(CRGB::Blue);
    }
  } else {
    if (match == match_state::not_active) {
      red_animation();
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
  }

  set_section_color(section_names::left_rear_bottom_stripe, overlay_color);
  set_section_color(section_names::left_rear_top_stripe, overlay_color);
  set_section_color(section_names::right_rear_bottom_stripe, overlay_color);
  set_section_color(section_names::right_rear_top_stripe, overlay_color);
}

void led_controller::shooter_overlay(bool ready) {
  CRGB overlay_color = CRGB::White; // Default to not ready
  if (ready) {
    overlay_color = CRGB::Green;
  }

  set_section_color(section_names::left_gear, overlay_color);
  set_section_color(section_names::right_gear, overlay_color);
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

void led_controller::calibrate() {
  set_section_color(section_names::left_bumper, CRGB::White);
  set_section_color(section_names::right_bumper, CRGB::Red);
  set_section_color(section_names::left_vertical, CRGB::Orange);
  set_section_color(section_names::left_rear_bottom_stripe, CRGB::Yellow);
  set_section_color(section_names::left_rear_top_stripe, CRGB::Green);
  set_section_color(section_names::left_gear, CRGB::Blue);
  set_section_color(section_names::left_front_top_stripe, CRGB::Indigo);
  set_section_color(section_names::left_front_bottom_stripe, CRGB::Purple);
  set_section_color(section_names::left_void, CRGB::White);
  set_section_color(section_names::right_void, CRGB::Red);
  set_section_color(section_names::right_front_bottom_stripe, CRGB::Orange);
  set_section_color(section_names::right_front_top_stripe, CRGB::Yellow);
  set_section_color(section_names::right_gear, CRGB::Green);
  set_section_color(section_names::right_rear_top_stripe, CRGB::Blue);
  set_section_color(section_names::right_rear_bottom_stripe, CRGB::Indigo);
  set_section_color(section_names::right_vertical, CRGB::Purple);
}

void led_controller::team_livery() {
  // Gear is white, stripes are red, and everything else blue.
  set_section_color(section_names::left_bumper, CRGB::Navy);
  set_section_color(section_names::right_bumper, CRGB::Navy);
  set_section_color(section_names::left_vertical, CRGB::Navy);
  set_section_color(section_names::left_rear_bottom_stripe, CRGB::Red);
  set_section_color(section_names::left_rear_top_stripe, CRGB::Red);
  set_section_color(section_names::left_gear, CRGB::White);
  set_section_color(section_names::left_front_top_stripe, CRGB::Red);
  set_section_color(section_names::left_front_bottom_stripe, CRGB::Red);
  set_section_color(section_names::left_void, CRGB::Navy);
  set_section_color(section_names::right_void, CRGB::Navy);
  set_section_color(section_names::right_front_bottom_stripe, CRGB::Red);
  set_section_color(section_names::right_front_top_stripe, CRGB::Red);
  set_section_color(section_names::right_gear, CRGB::White);
  set_section_color(section_names::right_rear_top_stripe, CRGB::Red);
  set_section_color(section_names::right_rear_bottom_stripe, CRGB::Red);
  set_section_color(section_names::right_vertical, CRGB::Navy);
}

void led_controller::blue_animation() {
  for (size_t i = 0; i < count_all_leds(); ++i) {
    leds[i] = CRGB::Blue;
  }
}

void led_controller::red_animation() {
  for (size_t i = 0; i < count_all_leds(); ++i) {
    leds[i] = CRGB::Red;
  }
}
