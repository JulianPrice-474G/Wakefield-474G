#pragma once

#include "EZ-Template/api.hpp"
#include "api.h" 

extern Drive chassis;


extern pros::Motor intake;
extern pros::Motor lever;
extern pros::adi::DigitalOut Hood;
extern pros::adi::DigitalOut Matchloader;
extern pros::adi::DigitalOut DESCORE_MECH;
extern pros::adi::DigitalOut RobotUp;
extern pros::adi::DigitalIn limit_switch;
extern void shootLeverTask(void*);



// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');