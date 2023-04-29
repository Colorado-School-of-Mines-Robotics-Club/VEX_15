#include "main.h"

#include <stdlib.h>
#include <float.h>

#include "ports.h"

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::Imu imu(IMU_PORT);
pros::ADIDigitalIn catapult_switch(CATAPULT_LIMIT_PORT);

pros::Motor_Group left_drive_group{ pros::Motor(FRONT_LEFT_MTR_PRT),
				    pros::Motor(MIDDLE_LEFT_MTR_PRT),
				    pros::Motor(BACK_LEFT_MTR_PRT) };
pros::Motor_Group right_drive_group{ pros::Motor(FRONT_RIGHT_MTR_PRT),
				     pros::Motor(MIDDLE_RIGHT_MTR_PRT),
				     pros::Motor(BACK_RIGHT_MTR_PRT) };
pros::Motor_Group intake_group{ pros::Motor(INTAKE_A_MTR_PRT),
				pros::Motor(INTAKE_B_MTR_PRT) };
pros::Motor_Group catapult_group{ pros::Motor(CATAPULT_A_MTR_PRT),
				  pros::Motor(CATAPULT_B_MTR_PRT) };
pros::Motor endgame_motor(ENDGAME_MTR_PTR);

/**
 * A callback function for LLEMU's center button.
 */
void on_center_button()
{
	left_drive_group = 0;
	right_drive_group = 0;
	intake_group = 0;
	catapult_group = 0;
}

void initCommon()
{
	pros::lcd::initialize();

	left_drive_group.set_gearing(
		pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	left_drive_group.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	right_drive_group.set_gearing(
		pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	right_drive_group.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);

	catapult_group.set_brake_modes(
		pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
	catapult_group.set_gearing(pros::motor_gearset_e_t::E_MOTOR_GEAR_RED);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	imu.reset(false);
	endgame_motor = -25;

	initCommon();

	pros::delay(1000);
	endgame_motor.set_zero_position(0);

	while (imu.is_calibrating()) {
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	left_drive_group = 0;
	right_drive_group = 0;
	intake_group = 0;
	catapult_group = 0;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

#define VOLTAGE_MAX 127
#define IN_TO_EN_MULT 80.0

double dAbs(double i)
{
	if (i < 0.0) {
		return -i;
	}
	return i;
}

double limitValue(double d, double max, double min)
{
	if (d > max)
		return max;
	else if (d < min)
		return min;
	return d;
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start
 * it from where it left off.
 */
void autonomous()
{
	initCommon();
	imu.reset(true);

	auto turnToAngle = [&](double maxVelocity, double minVelocity,
			       double targetDegrees, double targetError = 2.0,
			       double slowDownVelocity = 10.0) {
		while (true) {
			double heading = imu.get_heading();
			double error = dAbs(targetDegrees - heading);

			double velocity = limitValue(
				(double)maxVelocity *
					limitValue(error / slowDownVelocity,
						   1.0, 0.0),
				DBL_MAX, minVelocity);
			left_drive_group.move_velocity(velocity);
			right_drive_group.move_velocity(-velocity);

			if (error < targetError)
				break;
		}

		left_drive_group = 0;
		right_drive_group = 0;

		left_drive_group.brake();
		right_drive_group.brake();
	};

#ifndef N

	// Pull back catapult
	catapult_group.move_velocity(60);
	while (!catapult_switch.get_value()) {
	}
	catapult_group.brake();
	// Drive forward to launch point
	left_drive_group.set_zero_position(0.0);
	right_drive_group.set_zero_position(0.0);
	left_drive_group.move_absolute(10.0 * IN_TO_EN_MULT, 75);
	right_drive_group.move_absolute(10.0 * IN_TO_EN_MULT, 75);
	pros::delay(4500);

	// Turn to target
	turnToAngle(35.0, 10.0, 12.0);
	pros::delay(500);

	// Launch catapult
	catapult_group = VOLTAGE_MAX;
	pros::delay(1250);
	catapult_group = 0;
	pros::delay(1500);

	turnToAngle(-35.0, -10.0, 0.0);
	pros::delay(750);

	// Return to middle
	left_drive_group.move_absolute(5.0 * IN_TO_EN_MULT, 75);
	right_drive_group.move_absolute(5.0 * IN_TO_EN_MULT, 75);
	pros::delay(2500);

	turnToAngle(35.0, 10.0, 90.0);
	pros::delay(500);
	left_drive_group.set_zero_position(0.0);
	right_drive_group.set_zero_position(0.0);
	left_drive_group.move_absolute(4.0 * IN_TO_EN_MULT, 50);
	right_drive_group.move_absolute(4.0 * IN_TO_EN_MULT, 50);
	pros::delay(1500);

	// Move diagonal
	turnToAngle(35.0, 10.0, 135.0);
	pros::delay(500);
	left_drive_group.set_zero_position(0.0);
	right_drive_group.set_zero_position(0.0);
	left_drive_group.move_absolute(5.0 * IN_TO_EN_MULT, 50);
	right_drive_group.move_absolute(5.0 * IN_TO_EN_MULT, 50);
	pros::delay(1500);

	// Turn towards roller
	turnToAngle(35.0, 10.0, 180.0);
	// Drop intake
	intake_group = -50;
	pros::delay(200);
	intake_group = 0;
	pros::delay(200);
	// Move down close to roller
	left_drive_group.set_zero_position(0.0);
	right_drive_group.set_zero_position(0.0);
	left_drive_group.move_absolute(1.0 * IN_TO_EN_MULT, 50);
	right_drive_group.move_absolute(1.0 * IN_TO_EN_MULT, 50);
	pros::delay(1000);

#endif

	// Move against roller
	left_drive_group = 50;
	pros::delay(1500);
	// Spin roller
	intake_group = 100;
	pros::delay(200);

	// All stop
	left_drive_group = 0;
	right_drive_group = 0;
	intake_group = 0;
}

#define INTAKE_SPEED_PERCENT 1.0

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol()
{
	while (true) {
		int l_y = ctrl.get_analog(ANALOG_LEFT_Y);
		int r_y = ctrl.get_analog(ANALOG_RIGHT_Y);
		left_drive_group = l_y;
		right_drive_group = r_y;

		bool l1 = ctrl.get_digital(DIGITAL_L1);
		if (l1)
			intake_group = -VOLTAGE_MAX;
		else if (ctrl.get_digital(DIGITAL_L2) &&
			 catapult_switch.get_value())
			intake_group = (int)((double)VOLTAGE_MAX *
					     INTAKE_SPEED_PERCENT);
		else
			intake_group = 0;

		bool r1 = ctrl.get_digital(DIGITAL_R1);
		bool r2 = ctrl.get_digital(DIGITAL_R2);
		if (r1) {
			catapult_group = -VOLTAGE_MAX * 0.3;
		} else if (!catapult_switch.get_value()) {
			catapult_group.move_velocity(60);
		} else if (!r2 && catapult_switch.get_value()) {
			catapult_group.brake();
		} else if (r2 && catapult_switch.get_value()) {
			catapult_group = VOLTAGE_MAX;
			pros::delay(200);
		} else if (!r2) {
			catapult_group.brake();
		}

		bool ar = ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
			  ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_A);

		if (ar) {
			endgame_motor.move_absolute(400, 127);
		} else {
			endgame_motor.move_absolute(0, 100);
		}

		pros::delay(5);
	}
}
