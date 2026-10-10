#include "hololib/config.hpp"
#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "VPI.hpp"

//VPI

// Matches the hardware at boot: DigitalOut starts low
static bool VPIstate = false;

void setVPI(bool state) {
    VPIstate = state;
    VPI_piston.set_value(VPIstate);
}

void VPI_control(){
    static bool lastButtonState = false;

    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    // Only write on a press, so the first driver loop doesn't override whatever
    // autonomous left the piston at
    if (currentButtonState && !lastButtonState)
        setVPI(!VPIstate);
    lastButtonState = currentButtonState;

}
