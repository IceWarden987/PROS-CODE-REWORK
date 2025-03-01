#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#define DIGITAL_SENSOR_PORT 'D'

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left({-18, -19, -20}, pros::MotorCartridge::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right({11, 13, 14}, pros::MotorCartridge::blue);
pros::Motor intake(2, pros::MotorCartridge::blue);
pros::adi::DigitalOut mogo(DIGITAL_SENSOR_PORT);
pros::Imu imu(1);
lemlib::Drivetrain drivetrain(
	&left, // left motor group
	&right, // right motor group
	12, // 12 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	360, // drivetrain rpm is 360
	2 // horizontal drift is 2 (for now)
);
// create a v5 rotation sensor on port 1
pros::Rotation horizontal_encoder(4);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);
lemlib::OdomSensors sensors(
	nullptr, // vertical tracking wheel 1, set to null
	nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
	nullptr, // horizontal tracking wheel 1
	nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
	&imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
	10, // proportional gain (kP)
	0, // integral gain (kI)
	3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
	20 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angular_controller(
	3, // proportional gain (kP)
	0, // integral gain (kI)
	11, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
	lateral_controller, // lateral PID settings
	angular_controller, // angular PID settings
	sensors // odometry sensors
);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Tank control scheme
		int left_volt = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int right_volt = controller.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick
		left.move(left_volt);                      // Sets left motor voltage
		right.move(right_volt);                     // Sets right motor voltage
		if (controller.get_digital(DIGITAL_R1)) { 				// intake motor
			intake.move_velocity(500);
		}
		else if (controller.get_digital(DIGITAL_R2)) {
			intake.move_velocity(-500);
		}
		else {
			intake.move_velocity(0);
		}

		if (controller.get_digital(DIGITAL_L1)) {
			mogo.set_value(true);
		}
		if (controller.get_digital(DIGITAL_L2)) {
			mogo.set_value(false);
		}
		if (controller.get_digital(DIGITAL_A)) {
			// set position to x:0, y:0, heading:0
			chassis.setPose(0, 0, 0);
			intake.move_velocity(500);
			mogo.set_value(false);
			pros::delay(1500);
			intake.move_velocity(0);
			chassis.moveToPoint(0, 15, 1500);
			chassis.turnToHeading(90, 1500);
			chassis.moveToPoint(-34, 15, 6500, {.forwards = false});
			mogo.set_value(true);
			pros::delay(1500);
			mogo.set_value(false);
			pros::delay(2000);
			chassis.turnToHeading(-90, 1500);
			pros::delay(1500);
			intake.move_velocity(500);
			chassis.moveToPoint(-80, 15, 1500);
			pros::delay(3500);
			intake.move_velocity(0);
			chassis.turnToHeading(0, 1500);
			pros::delay(1500);
			chassis.moveToPoint(-96, 0, 1500, {.forwards = false});
			pros::delay(1500);
			mogo.set_value(true);

			


		}
		pros::delay(20);                               // Run for 20 ms then update
	}
}