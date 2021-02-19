#include "main.h"
#include "control/autoFunc.hpp"
#include "control/autoRoutines.hpp"
#include "control/driverControl.hpp"
#include "control/lcd.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

using namespace pros;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//lcd::initialize();

	//vision sensor monitoring task
	//Task visionMonitor(vMonitor, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "visionMonitor");

	//filters inertial sensor data
	Task kalmanMonitor(filterHeading, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "kalmanMonitor");
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

void autonomous() {
	skills1();
}

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
	liftMotor1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	liftMotor2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	leftIntake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	rightIntake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	leftDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	rightDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);

  Task drive(manualChassis, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "drive");
  Task intake(lifting, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake");
}
