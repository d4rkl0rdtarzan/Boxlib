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

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

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
    tagOdom.startTask(100);

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

    // 6. TELEMETRY LCD TASK
    pros::Task screen_task([&]() {
        while (true) {
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
}

void disabled() {
    odom.setPose(0.0f, 0.0f, 0.0f);
    my_lift.cancel();
}

void competition_initialize() {}
void simulation() {}

void autonomous() {
    // Autonomous moves should stop firmly at their targets, not roll past them
    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    odom.setPose(0.0f, 0.0f, 0.0f);

    // Diagnostic Test 1: Linear Translation
    chassisAsync(hololib::moveToPoint(0.0f, 24.0f));

    // Diagnostic Test 2: Point Turn
    chassisAsync(hololib::turnToPoint(12.0f, 24.0f));

    // Diagnostic Test 3: Holonomic Strafing
    chassisAsync(hololib::strafeDistance(10.0f));

    // Diagnostic Test 4: Motion Chaining with Early Exit
    chassisAsync(hololib::moveToPoint(0.0f, 0.0f, {.minSpeed = 35.0f, .earlyExitRange = 7.0f, .timeout = 2000}));
    chassisAsync(hololib::moveToPose(0.0f, 0.0f, 0.0f));
}

void opcontrol() {
    hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01f, .deadzone = 5.0f, .minimum_output = 5.0f};
    hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028f, .deadzone = 5.0f, .minimum_output = 5.0f};

    // Coast in driver control: motors freewheel when the sticks are released
    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    const int STICK_DEADZONE = 5; // same as the drive curve deadzones below

    while (true) {
        // Tank drive: left stick Y runs the left side, right stick Y runs the right side.
        // Both sticks together = straight, opposite = spin in place.
        // Pushing the sticks sideways strafes (average of the two X axes).
        int leftY = master.get_analog(ANALOG_LEFT_Y);
        int rightY = master.get_analog(ANALOG_RIGHT_Y);
        int leftX = master.get_analog(ANALOG_LEFT_X);
        int rightX = master.get_analog(ANALOG_RIGHT_X);

        int forward = (leftY + rightY) / 2;
        int rotation = (leftY - rightY) / 2;
        int sideways = (leftX + rightX) / 2;

        if (chassis.detectCollision()) {
            hololib::DriverReplay::getControllerInput(master);
            std::println("Collision Detected!");
        }

        bool sticksReleased = std::abs(forward) < STICK_DEADZONE && std::abs(sideways) < STICK_DEADZONE &&
                              std::abs(rotation) < STICK_DEADZONE;

        if (sticksReleased) {
            // Stops driving the motors so they follow the brake mode (coast).
            // Commanding 0 volts instead makes the motors resist rolling.
            chassis.brake();
        } else {
            chassis.driveControl(
                forward, sideways, rotation,
                {.movement = movement_curve, .rotation = rotation_curve},
                false, 0.0f, 
                {.correctionOn = true, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f}
            );
        }

        // Subsystems
        Intake();
        Lift();
        Claw_Control();
        VPI_control();

        pros::delay(20); 
    }
}