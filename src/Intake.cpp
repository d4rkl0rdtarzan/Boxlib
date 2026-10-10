#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "intake.hpp"

void Intake(){

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    intake.move(127);
}

else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    intake.move(-127);
}

else {
    intake.move(0);
}
}