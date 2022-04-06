#include "Arduino.h"

#include "communications.h"

communications::communications() {
    // Set up all pins as read and default high (ground to trigger)
    pinMode(alive_pin, INPUT_PULLUP);
    pinMode(color_pin, INPUT_PULLUP);
    pinMode(time_pin, INPUT_PULLUP);
    pinMode(cargo_hound_pin, INPUT_PULLUP);
    pinMode(shooter_pin, INPUT_PULLUP);
}

connection_state communications::connected() {
    if (digitalRead(alive_pin) == LOW) {
        Serial.println("Connected");
        return connection_state::alive;
    } else {
        Serial.println("NOT Connected");
        return connection_state::no_connection;
    }
}

alliance_color communications::color() {
        if (digitalRead(color_pin) == LOW) {
        Serial.println("Alliance: Red");
        return alliance_color::red;
    } else {
        Serial.println("Alliance: Blue");
        return alliance_color::blue;
    }       
}

match_state communications::match() {
        if (digitalRead(time_pin) == LOW) {
        Serial.println("Match: Active");
        return match_state::active;
    } else {
        Serial.println("Match: NOT Active");
        return match_state::not_active;
    }         
}

cargo_hound_state communications::cargo_hound() {
    if (digitalRead(cargo_hound_pin) == LOW) {
        Serial.println("Cargo: Acquired");
        return cargo_hound_state::acquired;
    } else {
        Serial.println("Cargo: NOT Acquired");
        return cargo_hound_state::not_acquired;
    }        
}

shooter_state communications::shooter() {
    if (digitalRead(shooter_pin) == LOW) {
        Serial.println("Shooter: Ready");
        return shooter_state::ready_to_fire;
    } else {
        Serial.println("Shooter: NOT Ready");
        return shooter_state::not_ready_to_fire;
    }
}
