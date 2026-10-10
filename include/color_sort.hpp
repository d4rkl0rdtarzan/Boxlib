#pragma once

// Color sensor on the intake. Settings (sensor port, button, color) are at the
// top of src/color_sort.cpp.
//
// While the sort button is held, the intake runs inward until the color sensor
// sees the chosen color, then stops and stays stopped until the button is
// released. The normal intake buttons are not affected.

// What the intake should do this loop
enum class ColorSortCommand {
    NONE, // sort button not held: run the normal intake code
    RUN,  // sort button held, chosen color not seen yet: run the intake inward
    STOP  // sort button held and the chosen color was seen: hold the intake still
};

void colorSortInit();               // call once in initialize()

// Call once per driver loop. While the sort button is held this runs or stops
// the intake motor itself and returns true, which tells the driver loop to
// skip the normal intake code. Returns false when the button is not held.
bool colorSortRunIntake();

ColorSortCommand colorSortUpdate(); // the decision alone, without touching the motor

// Readouts for the brain screen (used for tuning the settings)
double colorSortHue();              // color the sensor sees, 0 to 360
int colorSortProximity();           // how close the object is, 0 (far) to 255 (touching)
bool colorSortSeesTarget();         // true while the chosen color is in front of the sensor
const char* colorSortTargetName();  // "RED" or "BLUE"
