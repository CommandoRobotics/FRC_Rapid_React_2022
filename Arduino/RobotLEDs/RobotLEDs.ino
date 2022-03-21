// Code for an Arduino to control lights on FiaDuo (2022 Rapid React)
// Commando Robotics - FRC 5889

#include "communications.h"
#include "led_controller.h"

led_controller the_controller;
communications robo_rio;

void setup() {

}

void loop() {
  the_controller.loop(robo_rio);
}
