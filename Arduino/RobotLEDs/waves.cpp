#include "waves.h"

#include <math.h>

waves::waves() :
    pattern_start_location(0.0),
    last_pass(millis())
{ }

void waves::loop() {
    auto current_time = millis();
    auto time_passed = current_time - last_pass;
    // Ensure math is done using doubles, or the value will be rounded to 0.
    double percentage_to_advance = (time_passed * 1.0) / period_in_milliseconds;
    pattern_start_location += percentage_to_advance * pattern_width_in_pixels;
    while (pattern_start_location > pattern_width_in_pixels) {
        // Starting location is passed the end of the "repeated length", so bring it back in.
        pattern_start_location -= pattern_width_in_pixels;
        // This is repeated because so much time may have passed that we are many lengths be behind.
    }
    // Calculate the intensity of the sinusoidal wave at each pixel.
    for (size_t i = 0; i < pattern_width_in_pixels; ++i) {
        double pixels_from_pattern_start = static_cast<double>(i) - pattern_start_location;
        const double pi = 3.14159;
        double radians_from_pattern_start = (pixels_from_pattern_start / pattern_width_in_pixels) * 2 * pi;
        // Sine returns a number -1.0 to 1.0, so adjust to a 0 to 1.0 scale.
        intensity[i] = (sin(radians_from_pattern_start) + 1.0) / 2.0;
    }
}

void waves::apply(section_pointer section) {
    size_t pattern_pixel = 0;
    for (size_t i = 0; i < section.length; ++i) {
        section.start_point[i].setRGB(0, 0, static_cast<int>(intensity[pattern_pixel] * 255));
        ++pattern_pixel;
        if (pattern_pixel >= pattern_width_in_pixels) {
            // We've reached the end of the pattern, repeat from the beginning.
            pattern_pixel -= pattern_width_in_pixels;
        }
    }
}
