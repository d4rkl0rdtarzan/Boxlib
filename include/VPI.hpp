#pragma once
#include "main.h"

// VPI Control Function
void VPI_control();

// Sets the VPI piston and remembers it, so driver control toggles from here
// instead of snapping back to a default. Use this (not VPI_piston.set_value) in autonomous.
void setVPI(bool state);
