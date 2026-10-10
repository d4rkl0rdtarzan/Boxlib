#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "Claw_Pivot.hpp"

// Claw Pivot: X pivots one way, Y the other, holds position when released

void claw_pivot_control(){

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
    claw_pivot.move(127);
}

else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
    claw_pivot.move(-127);
}

else {
    claw_pivot.brake();
}
}
