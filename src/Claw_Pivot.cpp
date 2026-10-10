#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "Claw_Pivot.hpp"

// Claw Pivot: D-pad up pivots one way, D-pad down the other, holds position when
// released. (X is the color sort button.)

void claw_pivot_control(){

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    claw_pivot.move(127);
}

else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    claw_pivot.move(-127);
}

else {
    claw_pivot.brake();
}
}
