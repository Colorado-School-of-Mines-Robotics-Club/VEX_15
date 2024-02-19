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

bool catapult_deployed_in_auto = false;
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

void set_deploy_catapult()
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

#define INTAKE_EXTENDED_POSITION 170
#define INTAKE_RETRACTED_POSITION 60

#define DRIVE_UNITS_PER_INCH 27.46290005363848
#define DRIVE_UNITS_PER_DEGREE 3.12

enum class AutoActionType {
	DriveAction,
	IntakeSetExtend,
	IntakeSpin,
	DeployCatapult,
	WaitForCatapultDeploy,
	FireCatapultTime,
	WaitForCatapultEngage,
	WaitForCatapultSlip,
};

enum class MotorAction {
	MoveVoltage,
	MoveAbsolute,
	Brake,
};

struct AutoStep {
	AutoActionType action_type;
	MotorAction left_drive_action;
	double left_drive_target;
	double left_drive_speed;
	MotorAction right_drive_action;
	double right_drive_target;
	double right_drive_speed;

	bool intake_extend;
	double intake_extend_speed;
	MotorAction intake_spin_action;
	double intake_spin_speed;

	MotorAction catapult_fire_action;
	double catapult_fire_speed;

	uint32_t required_num_to_procede = 1;
	double timeout_ms;
};

class AutonomousSequence {
	private:
	std::vector<AutoStep> autonomous_steps;

	public:
	void move_position(double drive_target, double drive_speed,
			   double timeout_ms)
	{
		move_position(drive_target, drive_target, drive_speed,
			      drive_speed, timeout_ms);
	}

	void move_position(
		double left_drive_target, double right_drive_target,
		double left_drive_speed, double right_drive_speed,
		double timeout_ms,
		MotorAction left_drive_action = MotorAction::MoveAbsolute,
		MotorAction right_drive_action = MotorAction::MoveAbsolute)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DriveAction;
		new_action.left_drive_action = left_drive_action;
		new_action.left_drive_target = left_drive_target;
		new_action.left_drive_speed = left_drive_speed;
		new_action.right_drive_action = right_drive_action;
		new_action.right_drive_target = right_drive_target;
		new_action.right_drive_speed = right_drive_speed;

		new_action.required_num_to_procede = 2;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void drive_speed(double drive_speed, double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DriveAction;
		new_action.left_drive_action = MotorAction::MoveVoltage;
		new_action.left_drive_speed = drive_speed;
		new_action.right_drive_action = MotorAction::MoveVoltage;
		new_action.right_drive_speed = drive_speed;

		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void set_intake_extension(bool intake_extend, double rpm,
				  double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::IntakeSetExtend;
		new_action.intake_extend = intake_extend;
		new_action.intake_extend_speed = rpm;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void set_intake_spin(double intake_spin_voltage, double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::IntakeSpin;
		new_action.intake_spin_action = MotorAction::MoveVoltage;
		new_action.intake_spin_speed = intake_spin_voltage;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void deploy_catapult()
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DeployCatapult;
		new_action.timeout_ms = 0;

		autonomous_steps.push_back(new_action);
	}

	void wait_for_catapult_deploy(double timeout_ms = 10000)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::WaitForCatapultDeploy;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void fire_catapult_time(double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::FireCatapultTime;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void wait_for_catapult_engage()
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::WaitForCatapultEngage;
		new_action.required_num_to_procede = 1;
		new_action.timeout_ms = 2500;

		autonomous_steps.push_back(new_action);
	}

	void wait_for_catapult_slip()
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::WaitForCatapultSlip;
		new_action.required_num_to_procede = 1;
		new_action.timeout_ms = 2500;

		autonomous_steps.push_back(new_action);
	}

	void run_auto()
	{
		Timer auto_change_timer;
		for (const auto &step : autonomous_steps) {
			auto_change_timer.Restart();
			left_drive_group.brake();
			right_drive_group.brake();
			left_drive_group.tare_position();
			right_drive_group.tare_position();

			while (true) {
				handle_catapult_deploy();
				uint32_t num_ready_to_procede = 0;
				switch (step.action_type) {
				case AutoActionType::DriveAction:
					switch (step.left_drive_action) {
					case MotorAction::MoveVoltage:
						left_drive_group.move(
							step.left_drive_speed);
						break;
					case MotorAction::MoveAbsolute:
						left_drive_group.move_absolute(
							step.left_drive_target,
							step.left_drive_speed);

						if (double_abs(
							    left_drive_group
								    .get_positions()
									    [0] -
							    step.left_drive_target) <=
						    1.0)
							num_ready_to_procede +=
								1;
						break;
					case MotorAction::Brake:
						left_drive_group.brake();
						break;
					}
					switch (step.right_drive_action) {
					case MotorAction::MoveVoltage:
						right_drive_group.move(
							step.right_drive_speed);
						break;
					case MotorAction::MoveAbsolute:
						right_drive_group.move_absolute(
							step.right_drive_target,
							step.right_drive_speed);

						if (double_abs(
							    right_drive_group
								    .get_positions()
									    [0] -
							    step.right_drive_target) <=
						    1.0)
							num_ready_to_procede +=
								1;
						break;
					case MotorAction::Brake:
						right_drive_group.brake();
						break;
					}
					break;
				case AutoActionType::IntakeSetExtend:
					if (step.intake_extend) {
						intake_extension_group.move_absolute(
							INTAKE_EXTENDED_POSITION,
							step.intake_extend_speed);
					} else {
						intake_extension_group.move_absolute(
							INTAKE_RETRACTED_POSITION,
							step.intake_extend_speed);
					}
					break;
				case AutoActionType::IntakeSpin:
					intake_spin_group.move(
						step.intake_spin_speed);
					break;
				case AutoActionType::DeployCatapult:
					catapult_deployed_in_auto = true;
					set_deploy_catapult();
					break;
				case AutoActionType::WaitForCatapultDeploy:
					if (catapult_deploy_status ==
					    CatapultDeployStatus::NotDeploying) {
						num_ready_to_procede += 1;
					}
					break;
				case AutoActionType::FireCatapultTime:
					catapult_group.move(MAX_VOLTAGE);
					break;
				case AutoActionType::WaitForCatapultEngage:
					catapult_group.move(MAX_VOLTAGE);
					catapult_block.brake();
					if (catapult_group
						    .get_current_draws()[0] >
					    500) {
						num_ready_to_procede += 1;
					}
					break;
				case AutoActionType::WaitForCatapultSlip:
					catapult_group.move(MAX_VOLTAGE);
					catapult_block.brake();
					if (catapult_group
						    .get_current_draws()[0] <
					    300) {
						num_ready_to_procede += 1;
					}
					break;
				}
				pros::delay(5);
				if (num_ready_to_procede >=
					    step.required_num_to_procede ||
				    auto_change_timer.GetElapsedTime()
						    .AsMilliseconds() >
					    step.timeout_ms) {
					switch (step.action_type) {
					case AutoActionType::WaitForCatapultSlip:
						catapult_group.brake();
						break;
					default:
						break;
					}
					break;
				}
			}
		}
	}
};

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

	left_drive_group.tare_position();
	right_drive_group.tare_position();
	left_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	AutonomousSequence auto_sequence;

	// #define SKILLS_AUTO

#ifndef SKILLS_AUTO
	// Grab starting triball
	auto_sequence.move_position(-DRIVE_UNITS_PER_INCH * 1.5, MAX_RPM / 4.0,
				    1500);
	auto_sequence.set_intake_spin(-MAX_VOLTAGE, 0);
	auto_sequence.set_intake_extension(true, MAX_RPM / 2.0, 1000);
	auto_sequence.set_intake_extension(false, MAX_RPM / 2.0, 500);
	auto_sequence.set_intake_spin(0, 0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -2.5, MAX_RPM / 4.0,
				    500);
	// Move towards goal
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * -110,
				    DRIVE_UNITS_PER_DEGREE * 110, MAX_RPM / 4.0,
				    MAX_RPM / 4.0, 1500);
	auto_sequence.set_intake_spin(-MAX_VOLTAGE * 0.5, 0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -13, MAX_RPM / 4.0,
				    2500);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * -125,
				    DRIVE_UNITS_PER_DEGREE * 125, MAX_RPM / 4.0,
				    MAX_RPM / 4.0, 1500);
	auto_sequence.set_intake_spin(-MAX_VOLTAGE, 0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 12, MAX_RPM, 2000);
	// Go back to matchload zone
	auto_sequence.set_intake_spin(0, 0);
	auto_sequence.deploy_catapult();
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -10, MAX_RPM / 4.0,
				    2500);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * -35,
				    DRIVE_UNITS_PER_DEGREE * 35, MAX_RPM / 4.0,
				    MAX_RPM / 4.0, 1000);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -13, MAX_RPM / 4.0,
				    2500);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 45,
				    DRIVE_UNITS_PER_DEGREE * -120,
				    MAX_RPM / 6.0, MAX_RPM / 4.0, 2500);
	auto_sequence.drive_speed(-MAX_VOLTAGE * 0.25, 1200);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 25, 0,
				    MAX_RPM / 4.0, MAX_RPM / 4.0, 500);
	// Fire catapult
	auto_sequence.fire_catapult_time(20000);
	auto_sequence.wait_for_catapult_engage();
	auto_sequence.wait_for_catapult_slip();
	// Home after firing
	auto_sequence.drive_speed(-MAX_VOLTAGE * 0.35, 1200);
	// Contact overhead pipe
	auto_sequence.move_position(300.0, 0, MAX_RPM / 4.0, 0, 1000,
				    MotorAction::MoveAbsolute,
				    MotorAction::Brake);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 20, MAX_RPM / 4.0,
				    4000);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 36, MAX_RPM, 4000);
#else
	auto_sequence.set_intake_extension(true, MAX_RPM, 250);
	auto_sequence.set_intake_extension(false, MAX_RPM, 250);
	auto_sequence.deploy_catapult();
	auto_sequence.wait_for_catapult_deploy();
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 25, 0,
				    MAX_RPM / 4.0, MAX_RPM / 4.0, 500);
	// Fire catapult
	auto_sequence.fire_catapult_time(42000);
	auto_sequence.wait_for_catapult_engage();
	auto_sequence.wait_for_catapult_slip();
	// Home after firing
	auto_sequence.drive_speed(-MAX_VOLTAGE * 0.35, 1200);
	// Go to center
	auto_sequence.move_position(0, DRIVE_UNITS_PER_DEGREE * 25, 0,
				    MAX_RPM / 2.0, 2500);
	auto_sequence.set_intake_spin(MAX_VOLTAGE, 0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 60, MAX_RPM, 1500);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 55,
				    DRIVE_UNITS_PER_DEGREE * -55, MAX_RPM / 2.0,
				    MAX_RPM / 2.0, 750);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 90, MAX_RPM, 5000);
	auto_sequence.set_intake_spin(0, 0);
#endif

	auto_sequence.run_auto();

	left_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
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
	initCommon();

	if (!catapult_deployed_in_auto)
		set_deploy_catapult();

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
			intake_extension_group.move_absolute(
				INTAKE_EXTENDED_POSITION, MAX_RPM);
		} else {
			intake_extension_group.move_absolute(
				INTAKE_RETRACTED_POSITION, MAX_RPM);
		}

		// bool right_trigger_upper = ctrl.get_digital(DIGITAL_R1);
		bool do_intake = ctrl.get_digital(DIGITAL_L2);
		bool do_outtake = ctrl.get_digital(DIGITAL_L1);
		if (do_intake) {
			intake_spin_group.move(MAX_VOLTAGE);
		} else if (do_outtake) {
			intake_spin_group.move(-MAX_VOLTAGE);
		} else {
			intake_spin_group = 0;
		}

		// Catapult controls:
		if (catapult_deploy_status ==
		    CatapultDeployStatus::NotDeploying) {
			bool do_fire_catapult = ctrl.get_digital(DIGITAL_R2);
			// bool do_collapse_catapult = ctrl.get_digital(DIGITAL_B);
			bool do_collapse_catapult = false;
			if (do_collapse_catapult) {
				catapult_block.move(MAX_VOLTAGE);
				catapult_group.move(MAX_VOLTAGE);
			} else if (do_fire_catapult) {
				catapult_group.move(MAX_VOLTAGE);
				catapult_block.brake();
				// pros::lcd::set_text(1, std::to_string(catapult_group.get_current_draws()[0]));
			} else {
				catapult_group.brake();
				catapult_block.brake();
			}

			bool do_deploy_catapult =
				ctrl.get_digital(DIGITAL_DOWN);
			if (do_deploy_catapult) {
				if (catapult_button_timer_running == false) {
					catapult_button_timer->Restart();
					catapult_button_timer_running = true;
				} else if (catapult_button_timer
						   ->GetElapsedTime()
						   .AsMilliseconds() > 250.0) {
					set_deploy_catapult();
				}
			} else {
				catapult_button_timer_running = false;
			}
		}

		pros::delay(5);
	}
}
