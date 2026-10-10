#include "main.h"
#include "Eigen/Core" 
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/ApriltagLocalization.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motion_handler.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/modular_lift.hpp"
#include "hololib/util/replay.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"

#include "Subsystems.hpp"
#include "Claw.hpp"
#include "VPI.hpp"
#include "intake.hpp"
#include "lift.hpp"

#include <cstdio>
#include <cstdlib>
#include <print>

// 1. Hardware Port Definitions
int frontLPort = 1;
int frontRPort = 20;
int backLPort = 10;
int backRPort = 9;

int imuPort = 7;
int8_t leftOdomPort = 14;
int8_t rightOdomPort = 16;
int8_t horizontalOdomPort = 13;

std::vector<LiftMotorConfig> lift_motor_configs = {
    {15, pros::MotorGear::green},
    {19, pros::MotorGear::green}
};

// 2. CONSTANTS
Eigen::Matrix3f cameraMatrix = (Eigen::Matrix3f() << 383.57019326565296f * 0.5f, 0.0f, 322.50382974986405f * 0.5f,
                                0.0f, 385.93627187500545f * 0.5f, 230.00229561364546f * 0.5f,
                                0.0f, 0.0f, 1.0f).finished();

Eigen::Vector<float, 5> distCoeffs = (Eigen::Vector<float, 5>() << -0.047271311979687029f, 0.19589273312693006f,
                                      -0.0052232338824990303f, 0.00058677141664816013f, -0.14778998923627218f).finished();

// 3. GLOBAL OBJECT INSTANTIATIONS (Must match config.hpp exactly)
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor frontLeft(frontLPort, pros::MotorGear::blue);
pros::Motor frontRight(frontRPort, pros::MotorGear::blue);
pros::Motor backLeft(backLPort, pros::MotorGear::blue);
pros::Motor backRight(backRPort, pros::MotorGear::blue);
pros::Imu imu(imuPort);
pros::AIVision visionSensor(8);

hololib::ChassisConfig chassis_config = {
    .drivetrainWidth = 17.1f, .drivetrainLength = 17.4f, .wheelDiameter = 2.75f, .gearRatio = 1.0f
};

hololib::EncoderEKFOdometry odom(frontLeft, frontRight, backLeft, backRight, imu, chassis_config);
const std::function<hololib::Pose(bool)> poseGetter = [](bool radians) { return odom.getPose(radians); };

hololib::Chassis chassis(frontLeft, frontRight, backLeft, backRight, imu, odom);
hololib::GainScheduler xSched;
hololib::GainScheduler ySched;
hololib::GainScheduler thetaSched;
hololib::ApriltagLocalization tagOdom(visionSensor, imu, poseGetter, cameraMatrix, distCoeffs);
hololib::ObstacleManager obstacles;

LiftConfig my_lift_config = {
    .gear_ratio = 12.0f / 84.0f, .arm_length = 15.0f, .arm_mass_kg = 2.0f, .payload_mass_kg = 0.0f,
    .kG_base = 1750.0f / (2.0f * 9.81f), .tolerance = 5.0f,
    .K = Eigen::Matrix<float, 1, 2>{2.9331f, 1.4557f}, .spool_radius = 1.5f
};

ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE, my_lift_config);

// ---- On-screen diagnostics (brain screen lines 6 and 7) ----
#define BUILD_TAG "B8"            // shows which build is on the robot
volatile const char* diag_mode = "INIT"; // CAL, INIT, AUTO, AEND, OFF, DRV
volatile uint32_t diag_loops = 0;  // counts driver-control loops
volatile int diag_stage = 0;       // which step of the driver loop is running
volatile int diag_auton = 0;       // which autonomous move was last queued
volatile int diag_ly = 0, diag_ry = 0;

void initialize() {
    pros::lcd::initialize();

    // 6. TELEMETRY LCD TASK
    pros::Task screen_task([&]() {
        while (true) {
            // Printed first so they keep updating even if the pose readout below stalls
            pros::lcd::print(6, "%s %s L%lu S%d", BUILD_TAG, (const char*)diag_mode,
                             (unsigned long)diag_loops, (int)diag_stage);
            pros::lcd::print(7, "LY%d RY%d M%d A%d", (int)diag_ly, (int)diag_ry,
                             (int)hololib::motion_handler::isMoving(), (int)diag_auton);

            hololib::Pose pose = odom.getPose(false); 
            pros::lcd::print(0, "X: %.3f", pose.x);
            pros::lcd::print(1, "Y: %.3f", pose.y);
            pros::lcd::print(2, "Theta: %.3f", pose.theta);
            pros::lcd::print(3, "X Vel: %.3f", pose.velocity.vx);
            pros::lcd::print(4, "Y Vel: %.3f", pose.velocity.vy);
            pros::lcd::print(5, "W Vel: %.3f", pose.velocity.w);
            pros::delay(50);
        }
    });

    // Gyro calibration with a time limit. The library's chassis.calibrate()
    // waits forever if the gyro reports an error, which leaves the screen
    // blank and the robot dead. This version gives up after 3.5 seconds.
    diag_mode = "CAL";
    imu.reset(false);
    pros::delay(200);
    uint32_t calStart = pros::millis();
    while (imu.is_calibrating() && pros::millis() - calStart < 3500) {
        pros::delay(10);
    }
    diag_mode = (pros::millis() - calStart >= 3500) ? "CALFAIL" : "INIT";
    master.rumble(".");

    // 4. INJECT ALL 3 TRACKING WHEELS
    hololib::TrackingWheelConfig leftConfig = {
        .port = leftOdomPort,
        .orientation = hololib::TrackingWheelOrientation::VERTICAL,
        .xOffset = -5.125f,
        .yOffset = 0.0f,
        .wheelDiameter = 2.0f,
        .gearRatio = 1.0f
    };
    hololib::TrackingWheelConfig rightConfig = {
        .port = rightOdomPort,
        .orientation = hololib::TrackingWheelOrientation::VERTICAL,
        .xOffset = 5.125f,
        .yOffset = 0.0f,
        .wheelDiameter = 2.0f,
        .gearRatio = 1.0f
    };
    hololib::TrackingWheelConfig horizontalConfig = {
        .port = horizontalOdomPort,
        .orientation = hololib::TrackingWheelOrientation::HORIZONTAL,
        .xOffset = 0.0f,
        .yOffset = 0.0f, // Distance behind the center of rotation
        .wheelDiameter = 2.0f,
        .gearRatio = 1.0f
    };

    odom.addTrackingWheel(leftConfig);
    odom.addTrackingWheel(rightConfig);
    odom.addTrackingWheel(horizontalConfig);
    
    odom.startTask();
    // Camera (AprilTag) task turned off: nothing in this program uses its result,
    // and it runs heavy math and prints 10 messages a second in the background.
    // tagOdom.startTask(100);

    // 5. EXPLICIT FLOAT PID GAINS (4 values per array)
    xSched.setGains({
        {36.0f, {15.0f, 0.0f, 2.4f, 0.0f}},
        {0.0f,  {25.0f, 0.0f, 0.5f, 0.0f}}
    });
    ySched.setGains({
        {36.0f, {15.0f, 0.0f, 1.6f, 0.0f}},
        {0.0f,  {20.0f, 0.0f, 1.5f, 0.0f}}
    });
    thetaSched.setGains({
        {90.0f, {2.764f, 0.011f, 0.038f, 0.0f}},
        {0.0f,  {3.000f, 0.000f, 0.040f, 0.0f}}
    });

    odom.setVelocityCalculations(true);
}

void disabled() {
    // Intentionally empty. PROS kills this task the instant the robot is
    // re-enabled. If that lands while a sensor or motor call is in progress
    // (as the old setPose / lift calls here were), the lock it holds is never
    // released and driver control freezes for the rest of the run.
    diag_mode = "OFF";
}

void competition_initialize() {}
void simulation() {}

// ---- Autonomous time limit ----
const uint32_t AUTON_TIME_LIMIT_MS = 15000; // autonomous stops itself after this long
volatile bool auton_expired = false;        // true once the limit has passed
volatile int auton_run_id = 0;              // tells an old timer apart from a new run

// Queues one autonomous move, unless the time limit has already passed
#define autonMove(f)          \
    do {                      \
        if (!auton_expired) { \
            chassisAsync(f)   \
        }                     \
    } while (0)

void autonomous() {
    diag_mode = "AUTO";

    // Timer that ends autonomous after AUTON_TIME_LIMIT_MS, counted from this point
    auton_expired = false;
    auton_run_id = auton_run_id + 1;
    const int thisRun = auton_run_id;
    pros::Task auton_timer([thisRun] {
        pros::delay(AUTON_TIME_LIMIT_MS);
        if (auton_run_id != thisRun) return; // autonomous was restarted; a newer timer owns it
        auton_expired = true;
        diag_mode = "AEND";
        // Stop the move in progress, and any move that was queued just before the limit
        for (int i = 0; i < 100; i++) {
            if (hololib::motion_handler::isMoving()) {
                hololib::motion_handler::cancel();
            }
            pros::delay(10);
        }
        if (pros::competition::is_autonomous()) {
            chassis.brake();
        }
    });
    // Autonomous moves should stop firmly at their targets, not roll past them
    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    odom.setPose(0.0f, 0.0f, 0.0f);

    // Diagnostic Test 1: Linear Translation
    diag_auton = 1;
    autonMove(hololib::moveToPoint(0.0f, 24.0f));

    // Diagnostic Test 2: Point Turn
    diag_auton = 2;
    autonMove(hololib::turnToPoint(12.0f, 24.0f));

    // Diagnostic Test 3: Holonomic Strafing
    diag_auton = 3;
    autonMove(hololib::strafeDistance(10.0f));

    // Diagnostic Test 4: Motion Chaining with Early Exit
    diag_auton = 4;
    autonMove(hololib::moveToPoint(0.0f, 0.0f, {.minSpeed = 35.0f, .earlyExitRange = 7.0f, .timeout = 2000}));
    diag_auton = 5;
    autonMove(hololib::moveToPose(0.0f, 0.0f, 0.0f));
    diag_auton = 9; // every move has been queued
}

void opcontrol() {
    diag_mode = "DRV";
    // Coast in driver control: motors freewheel when the sticks are released
    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    const int STICK_DEADZONE = 5; // ignore tiny stick movement near center
    const int X_AXIS_DEADZONE = 25; // 20% of full stick travel (127): sideways push below this is ignored

    while (true) {
        // If an autonomous motion is still running (it can slip through when
        // autonomous is cut off by the timer), stop it so it cannot fight the driver.
        diag_loops = diag_loops + 1;
        diag_stage = 1;
        if (hololib::motion_handler::isMoving()) {
            hololib::motion_handler::cancel();
        }

        // Tank drive: left stick Y runs the left side, right stick Y runs the right side.
        // Both sticks together = straight, opposite = spin in place.
        // Pushing the sticks sideways strafes (average of the two X axes).
        int leftY = master.get_analog(ANALOG_LEFT_Y);
        int rightY = master.get_analog(ANALOG_RIGHT_Y);
        int leftX = master.get_analog(ANALOG_LEFT_X);
        int rightX = master.get_analog(ANALOG_RIGHT_X);

        // Ignore sideways stick movement until it passes 20%, so driving
        // forward with slightly off-center sticks does not strafe.
        if (std::abs(leftX) <= X_AXIS_DEADZONE) leftX = 0;
        if (std::abs(rightX) <= X_AXIS_DEADZONE) rightX = 0;

        int forward = (leftY + rightY) / 2;
        int rotation = (leftY - rightY) / 2;
        int sideways = (leftX + rightX) / 2;
        diag_ly = leftY;
        diag_ry = rightY;
        diag_stage = 2;

        diag_stage = 3;
        bool sticksReleased = std::abs(forward) < STICK_DEADZONE && std::abs(sideways) < STICK_DEADZONE &&
                              std::abs(rotation) < STICK_DEADZONE;

        if (sticksReleased) {
            // Stops driving the motors so they follow the brake mode (coast).
            chassis.brake();
        } else {
            // Sends the sticks straight to the X-drive mixer. This deliberately
            // skips the library's driveControl(), which waits on the position
            // tracker every loop; if the tracker stalls, driving would stall with it.
            if (std::abs(forward) < STICK_DEADZONE) forward = 0;
            if (std::abs(rotation) < STICK_DEADZONE) rotation = 0;
            chassis.drive(sideways, forward, rotation, 0);
        }

        // Subsystems
        diag_stage = 4;
        Intake();
        diag_stage = 5;
        Lift();
        diag_stage = 6;
        Claw_Control();
        diag_stage = 7;
        VPI_control();
        diag_stage = 0;

        pros::delay(20); 
    }
}