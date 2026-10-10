#include "hololib/config.hpp"
#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "Claw.hpp"

//Claw

void Claw_Control(){
    static bool Clawstate = true;
    static bool lastButtonState = false;

    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    if (currentButtonState && !lastButtonState)
        Clawstate = !Clawstate;
    lastButtonState = currentButtonState;
    Claw.set_value(Clawstate);

}