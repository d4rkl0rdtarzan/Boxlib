#include "main.h"
#include "color_sort.hpp"

// ============================================================
// Settings
// ============================================================

// Smart port the color (optical) sensor is plugged into
static const int OPTICAL_PORT = 5;

// Smart port(s) of the intake motor(s). Use a negative number for a motor that
// has to spin the other way to pull objects in, and list every intake motor,
// for example {2, -3}.
static pros::MotorGroup intake_motors({18});

// Intake speed while the sort button is held, 0 to 127
static const int SORT_INTAKE_SPEED = 127;

// Controller button that runs the intake until the chosen color arrives
static const pros::controller_digital_e_t SORT_BUTTON = pros::E_CONTROLLER_DIGITAL_X;

// The color the intake stops on
enum class SortColor { RED, BLUE };
static const SortColor STOP_ON = SortColor::RED;

// Hue ranges, 0 to 360. Red wraps around zero, so its minimum is larger than
// its maximum. Check these against the "Color sensor" page on the brain screen.
static const double RED_HUE_MIN = 340.0;
static const double RED_HUE_MAX = 25.0;
static const double BLUE_HUE_MIN = 190.0;
static const double BLUE_HUE_MAX = 250.0;

// An object only counts once it is this close (0 = far, 255 = touching)
static const int PROXIMITY_MIN = 120;

// ============================================================

extern pros::Controller master; // defined in main.cpp

static pros::Optical optical(OPTICAL_PORT);

void colorSortInit() {
    optical.set_led_pwm(100);         // sensor light on full, so colors read the same in any lighting
    optical.set_integration_time(20); // fast readings, so a moving object is not missed
}

double colorSortHue() { return optical.get_hue(); }

int colorSortProximity() { return optical.get_proximity(); }

const char* colorSortTargetName() { return STOP_ON == SortColor::RED ? "RED" : "BLUE"; }

bool colorSortSeesTarget() {
    if (optical.get_proximity() < PROXIMITY_MIN) return false;

    const double hue = optical.get_hue();
    const double min = STOP_ON == SortColor::RED ? RED_HUE_MIN : BLUE_HUE_MIN;
    const double max = STOP_ON == SortColor::RED ? RED_HUE_MAX : BLUE_HUE_MAX;
    if (min <= max) return hue >= min && hue <= max;
    return hue >= min || hue <= max; // range that wraps around zero (red)
}

ColorSortCommand colorSortUpdate() {
    static bool stopped = false;

    // Releasing the button clears the stop, ready for the next object
    if (!master.get_digital(SORT_BUTTON)) {
        stopped = false;
        return ColorSortCommand::NONE;
    }

    if (!stopped && colorSortSeesTarget()) stopped = true;
    return stopped ? ColorSortCommand::STOP : ColorSortCommand::RUN;
}

bool colorSortRunIntake() {
    switch (colorSortUpdate()) {
        case ColorSortCommand::RUN:
            intake_motors.move(SORT_INTAKE_SPEED);
            return true;
        case ColorSortCommand::STOP:
            intake_motors.brake();
            return true;
        default:
            return false; // button not held: the normal intake code is in charge
    }
}
