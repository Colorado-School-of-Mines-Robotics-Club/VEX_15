#include "main.h"

#include "ports.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "timer.h"

#define MAX_VOLTAGE 127
#define MAX_RPM 200

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left_drive_group(LEFT_DRIVE_PORTS);
pros::Motor_Group right_drive_group(RIGHT_DRIVE_PORTS);
pros::Motor_Group intake_extension_group(INTAKE_EXTENSION_PORTS);
pros::Motor_Group intake_spin_group(INTAKE_SPIN_PORTS);
pros::Motor_Group catapult_group(CATAPULT_DRIVE_PORTS);

pros::Motor catapult_block(CATAPULT_STOPPER_PORT);

enum class CatapultDeployStatus {
	NotDeploying,
	RemoveBlock,
	Home,
	PullBackFirst,
	PlaceBlock,
	PullBackSecond,
};

CatapultDeployStatus catapult_deploy_status =
	CatapultDeployStatus::NotDeploying;
Timer deploy_timer = Timer();

/**
 * A callback function for LLEMU's center button.
 */
void on_center_button()
{
	left_drive_group = 0;
	right_drive_group = 0;
}

void handle_catapult_deploy()
{
	switch (catapult_deploy_status) {
	case CatapultDeployStatus::NotDeploying:
		break;
	case CatapultDeployStatus::RemoveBlock:
		catapult_group.brake();
		catapult_block.move(MAX_VOLTAGE);
		if (deploy_timer.GetElapsedTime().AsMilliseconds() >= 500.0) {
			catapult_deploy_status = CatapultDeployStatus::Home;
			deploy_timer.Restart();
		}
		break;
	case CatapultDeployStatus::Home:
		catapult_block.move(0);
		catapult_group.move(-75);
		if (deploy_timer.GetElapsedTime().AsMilliseconds() < 100)
			break;
		if (catapult_group.get_actual_velocities()[0] > -10.0) {
			catapult_deploy_status =
				CatapultDeployStatus::PullBackFirst;
			catapult_group.brake();
			catapult_group.tare_position();
		}
		break;
	case CatapultDeployStatus::PullBackFirst:
		catapult_group.move_absolute(2600, MAX_RPM);
		if (catapult_group.get_positions()[0] >= 2600) {
			catapult_deploy_status =
				CatapultDeployStatus::PlaceBlock;
			deploy_timer.Restart();
		}
		break;
	case CatapultDeployStatus::PlaceBlock:
		catapult_block.move(-MAX_VOLTAGE);
		if (deploy_timer.GetElapsedTime().AsMilliseconds() >= 500.0 ||
		    deploy_timer.GetElapsedTime().AsSeconds() > 5.0) {
			catapult_deploy_status =
				CatapultDeployStatus::PullBackSecond;
			deploy_timer.Restart();
		}
		break;
	case CatapultDeployStatus::PullBackSecond:
		catapult_block.brake();
		catapult_group.move_absolute(3000, MAX_RPM);
		if (catapult_group.get_positions()[0] >= 3000 ||
		    deploy_timer.GetElapsedTime().AsSeconds() > 2.0) {
			catapult_deploy_status =
				CatapultDeployStatus::NotDeploying;
			catapult_group.brake();
		}
		break;
	}
}

void deploy_catapult()
{
	catapult_deploy_status = CatapultDeployStatus::RemoveBlock;
	deploy_timer.Restart();
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

	intake_extension_group.set_gearing(
		pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	intake_extension_group.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);

	intake_spin_group.set_gearing(
		pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	intake_spin_group.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);

	catapult_group.set_gearing(pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	catapult_group.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	catapult_group.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	catapult_block.set_gearing(pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	catapult_block.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	catapult_block.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	intake_extension_group.move(-50);
	pros::delay(1000);
	intake_extension_group.tare_position();
	pros::delay(10);
	intake_extension_group.move(0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	initCommon();
	deploy_catapult();
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
	intake_extension_group = 0;
	intake_spin_group = 0;
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

double double_abs(double i)
{
	if (i < 0.0) {
		return -i;
	}
	return i;
}

double limit_value(double d, double max, double min)
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

	// All stop
	left_drive_group = 0;
	right_drive_group = 0;
}

std::unique_ptr<Timer> intake_extension_toggle_timer =
	std::make_unique<Timer>();
bool is_intake_extended = false;

bool catapult_button_timer_running = false;
std::unique_ptr<Timer> catapult_button_timer = std::make_unique<Timer>();

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
		handle_catapult_deploy();

		int l_stick_y = ctrl.get_analog(ANALOG_LEFT_Y);
		int r_stick_y = ctrl.get_analog(ANALOG_RIGHT_Y);
		left_drive_group = l_stick_y;
		right_drive_group = r_stick_y;

		int right_trigger_upper = ctrl.get_digital(DIGITAL_R1);
		if (right_trigger_upper &&
		    intake_extension_toggle_timer->GetElapsedTime()
				    .AsMilliseconds() > 200) {
			is_intake_extended = !is_intake_extended;
			intake_extension_toggle_timer->Restart();
		}

		if (is_intake_extended) {
			intake_extension_group.move_absolute(170, MAX_RPM);
		} else {
			intake_extension_group.move_absolute(60, MAX_RPM);
		}

		// bool right_trigger_upper = ctrl.get_digital(DIGITAL_R1);
		bool left_trigger_lower = ctrl.get_digital(DIGITAL_L2);
		bool left_trigger_upper = ctrl.get_digital(DIGITAL_L1);
		if (left_trigger_lower) {
			intake_spin_group.move(MAX_VOLTAGE);
		} else if (left_trigger_upper) {
			intake_spin_group.move(-MAX_VOLTAGE);
		} else {
			intake_spin_group = 0;
		}

		// Catapult controls:
		if (catapult_deploy_status ==
		    CatapultDeployStatus::NotDeploying) {
			bool right_trigger_lower = ctrl.get_digital(DIGITAL_R2);
			bool up_arrow_button = ctrl.get_digital(DIGITAL_UP);
			if (up_arrow_button) {
				catapult_block.move(MAX_VOLTAGE);
				catapult_group.move(MAX_VOLTAGE);
			} else if (right_trigger_lower) {
				catapult_group.move(MAX_VOLTAGE);
				catapult_block.brake();
			} else {
				catapult_group.brake();
				catapult_block.brake();
			}

			bool down_arrow_button = ctrl.get_digital(DIGITAL_DOWN);
			if (down_arrow_button) {
				if (catapult_button_timer_running == false) {
					catapult_button_timer->Restart();
					catapult_button_timer_running = true;
				} else if (catapult_button_timer
						   ->GetElapsedTime()
						   .AsMilliseconds() > 250.0) {
					deploy_catapult();
				}
			} else {
				catapult_button_timer_running = false;
			}
		}

		pros::delay(5);
	}
}
