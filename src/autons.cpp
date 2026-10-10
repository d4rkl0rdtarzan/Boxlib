#include "main.h"
#include "autons.hpp"

#include <cstdio>

// How to write a routine:
//   1. Set the starting pose with odom.setPose(x, y, heading).
//   2. Queue each move with autonMove(...). Moves run in the order written.
//   3. Set diag_auton before each move so the brain screen (line 7, "A")
//      shows which step the robot was on if something goes wrong.
// To add a new routine, write it at the bottom of this file, declare it in
// include/autons.hpp, and add it to auton_selector.autons_add(...) in
// initialize() in main.cpp.

// ============================================================
// Selector (same functions and behavior as EZ-Template's auton selector)
// ============================================================

AutonSelector auton_selector;

static const char* SAVE_FILE = "/usd/auto.txt"; // remembered routine page, on the SD card
static const int SELECTOR_LAST_LINE = 5;        // the selector owns screen lines 0 to 5

static pros::adi::DigitalIn* limit_right = nullptr;
static pros::adi::DigitalIn* limit_left = nullptr;

static void onLeftButton() { auton_selector.page_down(); }
static void onRightButton() { auton_selector.page_up(); }

void AutonSelector::autons_add(std::vector<Auton> list) {
    autons.insert(autons.end(), list.begin(), list.end());
    needs_redraw = true;
}

void AutonSelector::initialize() {
    // Pick up the routine page saved on the SD card, if there is one
    if (pros::usd::is_installed()) {
        FILE* file = std::fopen(SAVE_FILE, "r");
        if (file != nullptr) {
            int saved = 0;
            if (std::fscanf(file, "%d", &saved) == 1 && saved >= 0 && saved < (int)autons.size()) {
                auton_page = saved;
                current_page = saved;
            }
            std::fclose(file);
        }
    }

    pros::lcd::register_btn0_cb(onLeftButton);
    pros::lcd::register_btn2_cb(onRightButton);
    is_enabled = true;
    needs_redraw = true;
}

void AutonSelector::limit_switch_lcd_initialize(pros::adi::DigitalIn* right_limit,
                                                pros::adi::DigitalIn* left_limit) {
    if (right_limit == nullptr && left_limit == nullptr) return;
    const bool alreadyRunning = limit_right != nullptr || limit_left != nullptr;
    limit_right = right_limit;
    limit_left = left_limit;
    if (alreadyRunning) return;

    // Checks the limit switches in the background
    pros::Task limit_task([] {
        while (true) {
            if (limit_right != nullptr && limit_right->get_new_press()) {
                auton_selector.page_up();
            } else if (limit_left != nullptr && limit_left->get_new_press()) {
                auton_selector.page_down();
            }
            pros::delay(50);
        }
    });
}

void AutonSelector::shutdown() {
    is_enabled = false;
    pros::lcd::shutdown();
}

void AutonSelector::page_up() { page_set(current_page + 1); }
void AutonSelector::page_down() { page_set(current_page - 1); }

void AutonSelector::page_set(int page) {
    const int routineCount = (int)autons.size();
    const int total = routineCount + blank_count;
    if (total == 0) return;
    if (page < 0) page = total - 1; // wrap around at either end
    if (page >= total) page = 0;
    current_page = page;
    needs_redraw = true;

    // Landing on a routine page selects that routine and remembers it
    if (page < routineCount) {
        auton_page = page;
        if (pros::usd::is_installed()) {
            FILE* file = std::fopen(SAVE_FILE, "w");
            if (file != nullptr) {
                std::fprintf(file, "%d", page);
                std::fclose(file);
            }
        }
    }
}

void AutonSelector::selected_auton_print() {
    if (!is_enabled) return;
    for (int i = 0; i <= SELECTOR_LAST_LINE; i++) pros::lcd::clear_line(i);

    const int routineCount = (int)autons.size();
    const int page = current_page;
    if (routineCount == 0 && blank_count == 0) {
        pros::lcd::set_text(0, "No Autons!");
        return;
    }
    if (page >= routineCount) return; // a blank page: whoever owns it draws it

    pros::lcd::print(0, "Page %d", page + 1);

    // Each "\n" in the text starts a new screen line
    const std::string& text = autons[page].text;
    size_t start = 0;
    for (int i = 1; i <= SELECTOR_LAST_LINE && start <= text.size(); i++) {
        size_t end = text.find('\n', start);
        if (end == std::string::npos) end = text.size();
        pros::lcd::set_text(i, text.substr(start, end - start));
        start = end + 1;
    }
}

void AutonSelector::selected_auton_call() {
    if (autons.empty()) return;
    int page = auton_page;
    if (page < 0 || page >= (int)autons.size()) page = 0;
    autons[page].run();
}

bool AutonSelector::page_blank_is_on(int page) {
    if (page < 0) return false;
    if (page >= blank_count) blank_count = page + 1; // create it on first use
    return is_enabled && current_page == (int)autons.size() + page;
}

void AutonSelector::page_blank_remove_all() {
    blank_count = 0;
    if (current_page >= (int)autons.size()) current_page = auton_page;
    needs_redraw = true;
}

// All drawing happens here, in the task that calls update(), so the button
// presses (which arrive from the screen's own task) never draw over it.
void AutonSelector::update() {
    if (!is_enabled || !needs_redraw) return;
    needs_redraw = false;
    selected_auton_print();
}

// ============================================================
// Routines
// ============================================================

void diagnosticAuton() {
    odom.setPose(0.0f, 0.0f, 0.0f);

    // Gains aren't tuned yet for the 0-127 output scale, so the tests run capped at
    // about half power. Every move also gets its own timeout so one that can't settle
    // doesn't eat the whole 15 s period. Raise the caps once the gains are tuned.
    static constexpr hololib::MoveParams safe{.maxTranslationSpeed = 60.0f, .maxRotationSpeed = 60.0f, .timeout = 2500};

    // Test 1: Linear Translation
    diag_auton = 1;
    autonMove(hololib::moveToPoint(0.0f, 24.0f, safe));

    // Test 2: Point Turn
    diag_auton = 2;
    autonMove(hololib::turnToPoint(12.0f, 24.0f, safe));

    // Test 3: Holonomic Strafing
    diag_auton = 3;
    autonMove(hololib::strafeDistance(10.0f, true, safe));

    // Test 4: Motion Chaining with Early Exit
    diag_auton = 4;
    autonMove(hololib::moveToPoint(0.0f, 0.0f,
                                   {.maxTranslationSpeed = 60.0f, .maxRotationSpeed = 60.0f, .minSpeed = 35.0f,
                                    .earlyExitRange = 7.0f, .timeout = 2000}));
    diag_auton = 5;
    autonMove(hololib::moveToPose(0.0f, 0.0f, 0.0f, safe));
}

void matchAuton() {
    // Starting position on the field: x, y in inches, heading in degrees
    odom.setPose(0.0f, 0.0f, 0.0f);

    // Step 1
    // diag_auton = 1;
    // autonMove(hololib::moveToPoint(0.0f, 24.0f));

    // Step 2
    // diag_auton = 2;
    // autonMove(hololib::turnToPoint(12.0f, 24.0f));
}
