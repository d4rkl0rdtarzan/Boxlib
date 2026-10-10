#pragma once
#include "main.h"

//Claw Control Function
void Claw_Control();

// Sets the claw piston and remembers it, so driver control toggles from here
// instead of snapping back to a default. Use this (not Claw.set_value) in autonomous.
void setClaw(bool state);
