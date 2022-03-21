#include "Arduino.h"

#include "communications.h"

communications::communications() {
    // Set up all pins as read and default low
    pinMode(alive_pin, INPUT);
    pinMode(color_pin, INPUT);
    pinMode(time_pin, INPUT);
    pinMode(cargo_hound_pin, INPUT);
    pinMode(shooter_pin, INPUT);
}

connection_state communications::connected() {
    if (digitalRead(alive_pin) == HIGH) {
        return connection_state::alive;
    } else {
        return connection_state::no_connection;
    }
}

alliance_color communications::color() {
        if (digitalRead(color_pin) == HIGH) {
        return alliance_color::red;
    } else {
        return alliance_color::blue;
    }       
}

match_state communications::match() {
        if (digitalRead(time_pin) == HIGH) {
        return match_state::active;
    } else {
        return match_state::not_active;
    }         
}

cargo_hound_state communications::cargo_hound() {
    if (digitalRead(cargo_hound_pin) == HIGH) {
        return cargo_hound_state::acquired;
    } else {
        return cargo_hound_state::not_acquired;
    }        
}

shooter_state communications::shooter() {
    if (digitalRead(shooter_pin) == HIGH) {
        return shooter_state::ready_to_fire;
    } else {
        return shooter_state::not_ready_to_fire;
    }
}
