#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "lift.hpp"

void Lift(){

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    lift.move(127);
}

else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
    lift.move(-127);
}

else {
    lift.move(0);
}
}