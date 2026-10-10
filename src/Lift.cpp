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
    // Stop using the brake mode (hold, set in initialize) so the lift keeps its
    // height instead of dropping when the buttons are released
    lift.brake();
}
}