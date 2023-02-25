#include "main.h"

#include <stdlib.h>

#include "ports.h"

/**
 * A callback function for LLEMU's center button.
 */
void on_center_button() {
	pros::Motor front_left_mtr(FRONT_LEFT_MTR_PRT);
	pros::Motor front_right_mtr(FRONT_RIGHT_MTR_PRT);
	pros::Motor back_left_mtr(BACK_LEFT_MTR_PRT);
	pros::Motor back_right_mtr(BACK_RIGHT_MTR_PRT);
	pros::Motor intake_a_mtr(INTAKE_A_MTR_PRT);
	pros::Motor intake_b_mtr(INTAKE_B_MTR_PRT);
	pros::Motor catapult_a_mtr(CATAPULT_A_MTR_PRT);
	pros::Motor catapult_b_mtr(CATAPULT_B_MTR_PRT);

	front_left_mtr = 0.0;
	front_right_mtr = 0.0;
	back_left_mtr = 0.0;
	back_right_mtr = 0.0;
	intake_a_mtr = 0.0;
	intake_b_mtr = 0.0;
	catapult_a_mtr = 0.0;
	catapult_b_mtr = 0.0;

	// intake_a_mtr.set_zero_position(0.0);
	// intake_b_mtr.set_zero_position(0.0);
	// intake_a_mtr.move_absolute(0.0, 30);
	// intake_b_mtr.move_absolute(0.0, 30);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::Motor front_left_mtr(FRONT_LEFT_MTR_PRT);
	pros::Motor front_right_mtr(FRONT_RIGHT_MTR_PRT);
	pros::Motor back_left_mtr(BACK_LEFT_MTR_PRT);
	pros::Motor back_right_mtr(BACK_RIGHT_MTR_PRT);
	pros::Motor intake_a_mtr(INTAKE_A_MTR_PRT);
	pros::Motor intake_b_mtr(INTAKE_B_MTR_PRT);
	pros::Motor catapult_a_mtr(CATAPULT_A_MTR_PRT);
	pros::Motor catapult_b_mtr(CATAPULT_B_MTR_PRT);

	front_left_mtr = 0.0;
	front_right_mtr = 0.0;
	back_left_mtr = 0.0;
	back_right_mtr = 0.0;
	intake_a_mtr = 0.0;
	intake_b_mtr = 0.0;
	catapult_a_mtr = 0.0;
	catapult_b_mtr = 0.0;

	// intake_a_mtr.set_zero_position(0.0);
	// intake_b_mtr.set_zero_position(0.0);
	// intake_a_mtr.move_absolute(0.0, 30);
	// intake_b_mtr.move_absolute(0.0, 30);

	// front_left_mtr.set

	// front_left_mtr.move_relative(0.0)
	// front_right_mtr.move_relative(0.0)
	// back_left_mtr.move_relative(0.0)
	// back_right_mtr.move_relative(0.0)
	// intake_a_mtr.move_relative(0.0)
	// intake_b_mtr.move_relative(0.0)
	// catapult_a_mtr.move_relative(0.0)
	// catapult_b_mtr.move_relative(0.0)
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
	pros::Motor front_left_mtr(FRONT_LEFT_MTR_PRT);
	pros::Motor front_right_mtr(FRONT_RIGHT_MTR_PRT);
	pros::Motor back_left_mtr(BACK_LEFT_MTR_PRT);
	pros::Motor back_right_mtr(BACK_RIGHT_MTR_PRT);
	pros::Motor intake_a_mtr(INTAKE_A_MTR_PRT);
	pros::Motor intake_b_mtr(INTAKE_B_MTR_PRT);
	pros::Motor catapult_a_mtr(CATAPULT_A_MTR_PRT);
	pros::Motor catapult_b_mtr(CATAPULT_B_MTR_PRT);

	auto driveTime = [&](int32_t leftV, int32_t rightV, uint32_t timeMS) {
		front_left_mtr = leftV;
		front_right_mtr = rightV;
		back_left_mtr = leftV;
		back_right_mtr = rightV;
		pros::delay(timeMS);
		front_left_mtr = 0;
		front_right_mtr = 0;
		back_left_mtr = 0;
		back_right_mtr = 0;
	};

	// Drive forward
	driveTime(50, 50, 500);
	pros::delay(1000);

	// Launch catapult
	catapult_a_mtr = 127;
	catapult_b_mtr = 127;
	pros::delay(2000);
	catapult_a_mtr = 0;
	catapult_b_mtr = 0;
	pros::delay(1500);

	driveTime(-50, -50, 500);
	pros::delay(1000);

	driveTime(50,-50, 825); // First turn, less than 90*
	driveTime(50, 50, 1500); // Forward
	driveTime(50,-50, 950); // Turn 90*
	driveTime(50, 50, 1200); // Move forward against roller

	intake_a_mtr = 30;
	intake_b_mtr = 30;
	pros::delay(300);
	intake_a_mtr = 0;
	intake_b_mtr = 0;

	pros::delay(1500);
	// Move Forward against roller
	front_left_mtr = 50;
	back_left_mtr = 50;
	pros::delay(500);

	// Spin roller
	intake_a_mtr = 100;
	intake_b_mtr = 100;

	pros::delay(200);
	// Stop
	intake_a_mtr = 0.0;
	intake_b_mtr = 0.0;

	front_left_mtr = 0;
	back_left_mtr = 0;
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
	pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
	pros::Motor front_left_mtr(FRONT_LEFT_MTR_PRT);
	pros::Motor front_right_mtr(FRONT_RIGHT_MTR_PRT);
	pros::Motor back_left_mtr(BACK_LEFT_MTR_PRT);
	pros::Motor back_right_mtr(BACK_RIGHT_MTR_PRT);
	pros::Motor intake_a_mtr(INTAKE_A_MTR_PRT);
	pros::Motor intake_b_mtr(INTAKE_B_MTR_PRT);
	pros::Motor catapult_a_mtr(CATAPULT_A_MTR_PRT);
	pros::Motor catapult_b_mtr(CATAPULT_B_MTR_PRT);

	while (true) {
		int l_y = ctrl.get_analog(ANALOG_LEFT_Y);
		int r_y = ctrl.get_analog(ANALOG_RIGHT_Y);
		front_left_mtr = l_y;
		back_left_mtr = l_y;
		front_right_mtr = r_y;
		back_right_mtr = r_y;

#define INTAKE_SPEED 127
#define OUTTAKE_SPEED 127

		bool l1 = ctrl.get_digital(DIGITAL_L1);
		if (l1) {
			intake_a_mtr = -OUTTAKE_SPEED;
			intake_b_mtr = -OUTTAKE_SPEED;
		} else {
			bool l2 = ctrl.get_digital(DIGITAL_L2);
			intake_a_mtr = INTAKE_SPEED * (int)l2;
			intake_b_mtr = INTAKE_SPEED * (int)l2;
		}

#define CATAPULT_SPEED 127
#define CATAPULT_SPEED_REV 75

		bool r1 = ctrl.get_digital(DIGITAL_R1);
		bool r2 = ctrl.get_digital(DIGITAL_R2);
		if (r1) {
			catapult_a_mtr = -CATAPULT_SPEED_REV;
			catapult_b_mtr = -CATAPULT_SPEED_REV;
		} else if (r2) {
			catapult_a_mtr = CATAPULT_SPEED;
			catapult_b_mtr = CATAPULT_SPEED;
		} else {
			catapult_a_mtr = 20;
			catapult_b_mtr = 20;
			catapult_a_mtr.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
			catapult_b_mtr.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
		}

		pros::delay(5);
	}

	while (false) {
		int l_y = ctrl.get_analog(ANALOG_LEFT_Y);
		int r_y = ctrl.get_analog(ANALOG_RIGHT_Y);

		catapult_a_mtr = l_y;
		catapult_b_mtr = l_y;

		intake_a_mtr = r_y;
		intake_b_mtr = r_y;

		pros::delay(5);
	}
}
