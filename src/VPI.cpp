#include "hololib/config.hpp"
#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "VPI.hpp"

//VPI

void VPI_control(){
    static bool VPIstate = true;
    static bool lastButtonState = false;

    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    if (currentButtonState && !lastButtonState)
        VPIstate = !VPIstate;
    lastButtonState = currentButtonState;
    VPI_piston.set_value(VPIstate);

}