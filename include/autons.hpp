#pragma once

// Autonomous routines and the autonomous selector live in src/autons.cpp.
// main.cpp lists the routines in initialize() and runs the selected one
// in autonomous().

#include "api.h"
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motion_handler.hpp"
#include "hololib/motions/motions.hpp"

#include <string>
#include <vector>

// Defined in main.cpp
extern volatile bool auton_expired; // true once the autonomous time limit has passed
extern volatile int diag_auton;     // which autonomous step was last queued (brain screen line 7)

// Queues one autonomous move, unless the time limit has already passed
#define autonMove(f)          \
    do {                      \
        if (!auton_expired) { \
            chassisAsync(f)   \
        }                     \
    } while (0)

// ---- Routines ----
void diagnosticAuton(); // drive, turn, strafe, and return-to-start test
void matchAuton();      // empty template for the real match routine

// ---- Selector (same functions and behavior as EZ-Template's auton selector) ----
// Each routine gets its own page on brain screen lines 0 to 5: "Page N" on
// line 0, then the routine's text. The left and right screen buttons change
// pages. After the last routine come the "blank pages", which the rest of the
// program can draw on (main.cpp puts the position readout on blank page 0).
// The routine page that was open last is the one autonomous runs, and with an
// SD card in the brain it is remembered between runs.

struct Auton {
    std::string text; // "Name\n\nDescription", up to 5 screen lines
    void (*run)();    // the routine to run
};

class AutonSelector {
  public:
    void autons_add(std::vector<Auton> list); // add routines; each one becomes a page
    void initialize();           // load the saved page and set up the screen buttons
    void limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit,
                                     pros::adi::DigitalIn* left_limit = nullptr); // page with limit switches
    void shutdown();             // turn the selector and the screen off
    bool enabled() const { return is_enabled; } // true after initialize()

    void page_up();              // next page (right screen button)
    void page_down();            // previous page (left screen button)
    void selected_auton_print(); // redraw lines 0 to 5 for the current page
    void selected_auton_call();  // run the selected routine

    bool page_blank_is_on(int page); // true while that blank page is open (creates it if needed)
    void page_blank_remove_all();    // remove every blank page

    void update();                   // redraws after a page change; called by the screen task in main.cpp
    int auton_page_current() const { return auton_page; } // selected routine, 0 = first

  private:
    std::vector<Auton> autons;
    volatile int current_page = 0;  // page on screen: routines first, then blank pages
    volatile int auton_page = 0;    // routine page that was open last
    volatile int blank_count = 0;   // how many blank pages exist
    volatile bool needs_redraw = false;
    volatile bool is_enabled = false;
    void page_set(int page);
};

extern AutonSelector auton_selector;
