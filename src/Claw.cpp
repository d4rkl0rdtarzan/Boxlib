#include "hololib/config.hpp"
#include "main.h"
#include "pros/misc.h"
#include "Subsystems.hpp"
#include "Claw.hpp"

//Claw

// Matches the hardware at boot: DigitalOut starts low
static bool Clawstate = false;

void setClaw(bool state) {
    Clawstate = state;
    Claw.set_value(Clawstate);
}

void Claw_Control(){
    static bool lastButtonState = false;

    bool currentButtonState = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    // Only write on a press, so the first driver loop doesn't override whatever
    // autonomous left the claw at
    if (currentButtonState && !lastButtonState)
        setClaw(!Clawstate);
    lastButtonState = currentButtonState;

}
