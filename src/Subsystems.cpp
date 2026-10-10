#include "Subsystems.hpp"

// Definitions (Initializations) go here, exactly ONCE
pros::MotorGroup lift({15, -19});
pros::Motor intake(18);
pros::Motor claw_pivot(11);
pros::adi::DigitalOut VPI_piston('A');
pros::adi::DigitalOut Claw('B');