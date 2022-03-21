// Blue Waves effect
// Commando Robotics
#ifndef WAVES_H
#define WAVES_H

#include <FastLED.h>

#include "section.h"

// Encapsulates the Blue Waves effect.
// Sinusoidal intensity "moves" along the pixel strand.
class waves {
public:
    waves();    

    // Call to advance the effect. Call once per pass.
    void loop();

    // Sets the LED array with the effect. Call this for each section.
    void apply(section_pointer section);
private:
    static const size_t pattern_width_in_pixels = 10; // How many pixels before the pattern repeats
    double intensity[pattern_width_in_pixels]; // Array used to store the "intensity" of blue (0-1.0)
    unsigned long period_in_milliseconds = 2000; // How quickly the wave moves
    unsigned long last_pass; // Time loop() was last called.
    double pattern_start_location; // "Pixel" (or partial pixel) where the pattern currently begins
};

#endif
