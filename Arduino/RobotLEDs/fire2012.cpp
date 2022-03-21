// Fire 2012 Implementation

#include "fire2012.h"

fire2012::fire2012(size_t total_leds_in_this_section) :
    number_of_leds(total_leds_in_this_section)
{
    uint8_t heat = new uint8_t[number_of_leds];
}

fire2012::~fire2012() {
    delete[] heat;
}

void fire2012::loop() {
    // Step 1.  Cool down every cell a little
    for( int i = 0; i < number_of_leds; i++) {
        heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / number_of_leds) + 2));
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (int k = number_of_leds - 1; k >= 2; k--) {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
        int y = random8(7);
        heat[y] = qadd8( heat[y], random8(160,255) );
    }
}

void fire2012::apply(section_pointer the_section) {
    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < number_of_leds; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber = j;
      the_section.start_point[pixelnumber] = color;
    }
}
