// Data communicated with the RoboRio
#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <stdlib.h>

enum class connection_state {
  alive, // High signal from RoboRio
  no_connection // Ground or floating
};

enum class alliance_color {
  blue,
  red
};

enum class match_state {
  active, // Autonomous or teleop
  not_active // Pre- or Post-match
};

enum class cargo_hound_state {
  not_acquired,
  acquired
};

enum class shooter_state {
  not_ready_to_fire,
  ready_to_fire
};

class communications {
public:
    // Constructor sets up communicartions pins
    communications();

    connection_state connected();

    alliance_color color();

    match_state match();

    cargo_hound_state cargo_hound();

    shooter_state shooter();

private:
    const size_t alive_pin = 13;
    const size_t color_pin = 12;
    const size_t time_pin = 11;
    const size_t cargo_hound_pin = 10;
    const size_t shooter_pin = 9;
};

#endif
