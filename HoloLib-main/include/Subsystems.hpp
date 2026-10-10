#pragma once
#include "main.h" // or whichever PROS header you need

// Declarations ONLY
extern pros::Controller master; // defined in main.cpp
extern pros::MotorGroup lift;
extern pros::Motor intake;
extern pros::Motor claw_pivot;
extern pros::adi::DigitalOut VPI_piston;
extern pros::adi::DigitalOut Claw;