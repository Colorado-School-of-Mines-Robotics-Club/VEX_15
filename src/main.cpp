#include "main.h"

#include "ports.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "timer.h"
#include <algorithm>

#define MAX_VOLTAGE 127
#define MAX_RPM 200

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::Imu imu(IMU_PORT);

pros::Motor_Group left_drive_group(LEFT_DRIVE_PORTS);
pros::Motor_Group right_drive_group(RIGHT_DRIVE_PORTS);
pros::Motor_Group intake_extension_group(INTAKE_EXTENSION_PORTS);
pros::Motor_Group intake_spin_group(INTAKE_SPIN_PORTS);
pros::Motor_Group catapult_group(CATAPULT_DRIVE_PORTS);
pros::ADIDigitalOut left_wing(LEFT_WING_PORT);
pros::ADIDigitalOut right_wing(RIGHT_WING_PORT);

pros::Motor catapult_block(CATAPULT_STOPPER_PORT);

pros::Motor climb_motor(CLIMB_MOTOR_PORT);

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
		if (catapult_group.get_actual_velocities()[0] > -10.0 ||
		    deploy_timer.GetElapsedTime().AsSeconds() > 8.0) {
			catapult_deploy_status =
				CatapultDeployStatus::PullBackFirst;
			catapult_group.brake();
			catapult_group.tare_position();
		}
		break;
	case CatapultDeployStatus::PullBackFirst:
		catapult_group.move_absolute(1300, MAX_RPM);
		if (catapult_group.get_positions()[0] >= 1300 ||
		    deploy_timer.GetElapsedTime().AsSeconds() > 10.0) {
			catapult_deploy_status =
				CatapultDeployStatus::PlaceBlock;
			deploy_timer.Restart();
		}
		break;
	case CatapultDeployStatus::PlaceBlock:
		catapult_block.move(-MAX_VOLTAGE);
		if (deploy_timer.GetElapsedTime().AsMilliseconds() >= 500.0) {
			catapult_deploy_status =
				CatapultDeployStatus::PullBackSecond;
			deploy_timer.Restart();
		}
		break;
	case CatapultDeployStatus::PullBackSecond:
		catapult_group.move_absolute(1500, MAX_RPM);
		if (catapult_group.get_positions()[0] >= 1500 ||
		    deploy_timer.GetElapsedTime().AsSeconds() > 2.0) {
			catapult_deploy_status =
				CatapultDeployStatus::NotDeploying;
			catapult_group.brake();
			catapult_block.brake();
		}
		break;
	}
}

void set_deploy_catapult()
{
	catapult_deploy_status = CatapultDeployStatus::RemoveBlock;
	deploy_timer.Restart();
}

bool has_intake_homed = false;
bool has_imu_been_set = false;

void initCommon(bool init_imu)
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

	climb_motor.set_gearing(pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
	climb_motor.set_encoder_units(
		pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	climb_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	climb_motor.brake();

	if (init_imu && !has_imu_been_set) {
		imu.reset();
	}

	if (!has_intake_homed) {
		has_intake_homed = true;
		intake_extension_group.move(-50);
		pros::delay(400);
		intake_extension_group.tare_position();
		pros::delay(10);
		intake_extension_group.move(0);
	}

	if (init_imu && !has_imu_been_set) {
		while (imu.is_calibrating())
			pros::delay(5);
		has_imu_been_set = true;
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
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

#define INTAKE_EXTENDED_POSITION 170
#define INTAKE_RETRACTED_POSITION 80

#define DRIVE_UNITS_PER_INCH 27.46290005363848
#define DRIVE_UNITS_PER_DEGREE 3.12

enum class AutoActionType {
	WaitUntilMatchTime,
	ResetIMU,
	TurnIMUFromStart,
	DriveAction,
	IntakeSetExtend,
	IntakeSpin,
	DeployCatapult,
	WaitForCatapultDeploy,
	FireCatapultTime,
	WaitForCatapultEngage,
	WaitForCatapultSlip,
	RunBlockingLambda,
};

enum class MotorAction {
	MoveVoltage,
	MoveAbsolute,
	Brake,
};

enum class Direction {
	Clockwise,
	CounterClockwise,
};

struct AutoStep {
	AutoActionType action_type;
	uint32_t delay_ms_after_done = 0;

	double imu_degree_target;
	double imu_turn_half_offset;
	double imu_turn_target_range;
	Direction imu_turn_direction;

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

	double wait_until_clock_time;

	std::function<void(Timer &)> lambda;
};

class AutonomousSequence {
	private:
	std::vector<AutoStep> autonomous_steps;
	Timer auto_timer;

	public:
	void start_timer()
	{
		auto_timer.Restart();
	}

	void wait_until_match_time(double time_s)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::WaitUntilMatchTime;
		new_action.wait_until_clock_time = time_s;
		new_action.timeout_ms = 10000000;

		autonomous_steps.push_back(new_action);
	}

	void reset_imu(double timeout_ms, bool blocking = false)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::ResetIMU;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void turn_imu(Direction direction, double degrees, double drive_voltage,
		      double timeout_ms, uint32_t delay_ms_after_done = 5,
		      double turn_target_range = 2.0,
		      double imu_turn_half_offset = 5.0)
	{
		AutoStep new_action;

		new_action.delay_ms_after_done = delay_ms_after_done;

		new_action.action_type = AutoActionType::TurnIMUFromStart;
		new_action.imu_degree_target = degrees;
		new_action.imu_turn_direction = direction;
		new_action.imu_turn_half_offset = imu_turn_half_offset;
		new_action.imu_turn_target_range = turn_target_range;

		new_action.left_drive_action = MotorAction::MoveVoltage;
		new_action.left_drive_speed = drive_voltage;
		new_action.right_drive_action = MotorAction::MoveVoltage;
		new_action.right_drive_speed = drive_voltage;

		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void move_position(double drive_target, double drive_rpm,
			   double timeout_ms)
	{
		move_position(drive_target, drive_target, drive_rpm, drive_rpm,
			      timeout_ms);
	}

	void move_position(
		double left_drive_target, double right_drive_target,
		double left_drive_rpm, double right_drive_rpm,
		double timeout_ms,
		MotorAction left_drive_action = MotorAction::MoveAbsolute,
		MotorAction right_drive_action = MotorAction::MoveAbsolute)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DriveAction;
		new_action.left_drive_action = left_drive_action;
		new_action.left_drive_target = left_drive_target;
		new_action.left_drive_speed = left_drive_rpm;
		new_action.right_drive_action = right_drive_action;
		new_action.right_drive_target = right_drive_target;
		new_action.right_drive_speed = right_drive_rpm;

		new_action.required_num_to_procede = 2;
		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void drive_power(double drive_voltage, double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DriveAction;
		new_action.left_drive_action = MotorAction::MoveVoltage;
		new_action.left_drive_speed = drive_voltage;
		new_action.right_drive_action = MotorAction::MoveVoltage;
		new_action.right_drive_speed = drive_voltage;

		new_action.timeout_ms = timeout_ms;

		autonomous_steps.push_back(new_action);
	}

	void drive_power(double drive_voltage_left, double drive_voltage_right,
			 double timeout_ms)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::DriveAction;
		new_action.left_drive_action = MotorAction::MoveVoltage;
		new_action.left_drive_speed = drive_voltage_left;
		new_action.right_drive_action = MotorAction::MoveVoltage;
		new_action.right_drive_speed = drive_voltage_right;

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

	void fire_catapult_time(double timeout_ms, double voltage = MAX_VOLTAGE)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::FireCatapultTime;
		new_action.timeout_ms = timeout_ms;
		new_action.catapult_fire_speed = voltage;

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

	void run_blocking_lambda(std::function<void(Timer &)> func)
	{
		AutoStep new_action;

		new_action.action_type = AutoActionType::RunBlockingLambda;
		new_action.lambda = func;

		autonomous_steps.push_back(new_action);
	}

	void run_auto()
	{
		double last_imu_rotation = 0.0;

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
				case AutoActionType::WaitUntilMatchTime:
					if (auto_timer.GetElapsedTime()
						    .AsSeconds() >=
					    step.wait_until_clock_time)
						num_ready_to_procede += 1;
				case AutoActionType::ResetIMU:
					imu.reset();

					if (!imu.is_calibrating())
						num_ready_to_procede += 1;
					break;
				case AutoActionType::TurnIMUFromStart: {
					double current_angle =
						imu.get_rotation();
					double mult =
						(step.imu_turn_direction ==
								 Direction::
									 Clockwise ?
							 1.0 :
							 -1.0) *
						(current_angle - step.imu_degree_target <
								 step.imu_turn_half_offset ?
							 0.5 :
							 1.0);
					left_drive_group.move(
						step.left_drive_speed * mult);
					right_drive_group.move(
						-step.left_drive_speed * mult);

					if (std::abs(current_angle -
						     step.imu_degree_target) <
					    step.imu_turn_target_range) {
						num_ready_to_procede += 1;
					}
					break;
				}
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
				case AutoActionType::FireCatapultTime: {
					catapult_group.move(
						step.catapult_fire_speed);
					Timer jam_timer = Timer();
					if (catapult_group
						    .get_current_draws()[0] >
					    1750) {
						bool do_unjam = true;
						while (jam_timer
							       .GetElapsedTime()
							       .AsMilliseconds() <
						       500) {
							if (catapult_group
								    .get_current_draws()
									    [0] <
							    1750) {
								do_unjam =
									false;
								break;
							}
						}
						if (do_unjam) {
							catapult_group.move(
								-MAX_VOLTAGE);
							pros::delay(650);
							catapult_group.move(0);
							pros::delay(1000);
						}
					}
				} break;
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
				case AutoActionType::RunBlockingLambda:
					step.lambda(auto_timer);
					num_ready_to_procede += 1;
					break;
				}
				pros::delay(5);
				if (num_ready_to_procede >=
					    step.required_num_to_procede ||
				    auto_change_timer.GetElapsedTime()
						    .AsMilliseconds() >
					    step.timeout_ms) {
					switch (step.action_type) {
					case AutoActionType::TurnIMUFromStart: {
						left_drive_group.set_brake_modes(
							pros::E_MOTOR_BRAKE_HOLD);
						right_drive_group.set_brake_modes(
							pros::E_MOTOR_BRAKE_HOLD);
						left_drive_group.brake();
						right_drive_group.brake();
						left_drive_group.set_brake_modes(
							pros::E_MOTOR_BRAKE_COAST);
						right_drive_group.set_brake_modes(
							pros::E_MOTOR_BRAKE_COAST);
					}
					case AutoActionType::WaitForCatapultSlip:
						catapult_group.brake();
						break;
					case AutoActionType::FireCatapultTime:
						catapult_group.brake();
					default:
						break;
					}
					if (step.delay_ms_after_done != 0)
						pros::delay(
							step.delay_ms_after_done);
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
// #define SKILLS

	AutonomousSequence auto_sequence;
	auto_sequence.start_timer();
	has_imu_been_set = false;
	has_intake_homed = false;
#ifdef SKILLS
	initCommon(false);
#else
	initCommon(true);
#endif

	left_drive_group.tare_position();
	right_drive_group.tare_position();
	left_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	right_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

#ifdef SKILLS
	auto_sequence.deploy_catapult();
	auto_sequence.drive_power(MAX_VOLTAGE, 500);
	auto_sequence.drive_power(0, 250);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -14,
				    DRIVE_UNITS_PER_INCH * -9.5, MAX_RPM,
				    MAX_RPM / 2.5, 750);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -10, MAX_RPM, 500);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * -1,
				    DRIVE_UNITS_PER_INCH * -18, MAX_RPM / 6.0,
				    MAX_RPM, 1000);

	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 35, -20,
				    MAX_RPM / 4.0, MAX_RPM / 4.0, 500);
	auto_sequence.wait_for_catapult_deploy();
	// Fire catapult
	auto_sequence.run_blocking_lambda([](Timer &auto_timer) {
		catapult_block.brake();

		int step = 1;
		Timer delay_timer;
		while (true) {
			// ctrl.print(0, 0, "%i", step);
			uint32_t slip_angle =
				std::max(
					(uint32_t)0,
					(uint32_t)(catapult_group
							   .get_positions()[0] -
						   1500.0)) %
				1259;
			// ctrl.print(0, 1, "%i", catapult_group.get_current_draws()[0]);
			if (step == 1) {
				catapult_group.move(MAX_VOLTAGE);
				if (slip_angle >= 1100) {
					delay_timer.Restart();
					step += 1;
					continue;
				}
			} else if (step == 2) {
				catapult_group.brake();
				if (delay_timer.GetElapsedTime()
					    .AsMilliseconds() > 150) {
					step += 1;
					continue;
				}
			} else if (step == 3) {
				catapult_group.move(MAX_VOLTAGE);
				if (slip_angle < 100) {
					step = 1;
					continue;
				}
			}

			// Timer starts when auto starts
			if (auto_timer.GetElapsedTime().AsMilliseconds() >=
			    49000)
				break;

			Timer jam_timer = Timer();
			if (catapult_group.get_current_draws()[0] > 1750) {
				bool do_unjam = true;
				while (jam_timer.GetElapsedTime()
					       .AsMilliseconds() < 500) {
					if (catapult_group
						    .get_current_draws()[0] <
					    1750) {
						do_unjam = false;
						break;
					}
				}
				if (do_unjam) {
					catapult_group.move(-MAX_VOLTAGE);
					pros::delay(650);
					catapult_group.move(0);
					pros::delay(500);
				}
			}

			pros::delay(5);
		}
		catapult_group.move(-MAX_VOLTAGE);
		pros::delay(500);
		catapult_group.move(0);
	});
	auto_sequence.drive_power(-MAX_VOLTAGE * 0.35, -MAX_VOLTAGE * 0.25,
				  1200);
	// Go to center
	auto_sequence.move_position(0, DRIVE_UNITS_PER_DEGREE * 30, 0,
				    MAX_RPM / 2.0, 2500);
	auto_sequence.set_intake_spin(MAX_VOLTAGE, 0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 60, MAX_RPM, 1500);
	// Turn towards other side of field
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 75,
				    DRIVE_UNITS_PER_DEGREE * -75, MAX_RPM / 2.0,
				    MAX_RPM / 2.0, 750);
	// Go across
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 90, MAX_RPM, 750);
	auto_sequence.run_blocking_lambda([](Timer &auto_timer) {
		right_wing.set_value(true);
		left_wing.set_value(true);
	});
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 90, MAX_RPM, 3500);
	auto_sequence.run_blocking_lambda([](Timer &auto_timer) {
		right_wing.set_value(false);
		left_wing.set_value(false);
	});
	auto_sequence.drive_power(-MAX_VOLTAGE, 400);
	auto_sequence.set_intake_spin(0, 0);
#else
	// Prep to fire
	auto_sequence.deploy_catapult();
	auto_sequence.drive_power(-MAX_VOLTAGE * 0.35, 300);
	auto_sequence.move_position(DRIVE_UNITS_PER_DEGREE * 40, 0,
				    MAX_RPM / 4.0, MAX_RPM / 4.0, 500);
	auto_sequence.wait_for_catapult_deploy();
	// Fire catapult
	auto_sequence.run_blocking_lambda([](Timer &auto_timer) {
		Timer lambda_timer;
		catapult_block.brake();

		int step = 1;
		Timer delay_timer;
		while (true) {
			// ctrl.print(0, 0, "%i", step);
			uint32_t slip_angle =
				std::max(
					(uint32_t)0,
					(uint32_t)(catapult_group
							   .get_positions()[0] -
						   1500.0)) %
				1259;
			// ctrl.print(0, 1, "%i", catapult_group.get_current_draws()[0]);
			if (step == 1) {
				catapult_group.move(MAX_VOLTAGE);
				if (slip_angle >= 1100) {
					delay_timer.Restart();
					step += 1;
					continue;
				}
			} else if (step == 2) {
				catapult_group.brake();
				if (delay_timer.GetElapsedTime()
					    .AsMilliseconds() > 150) {
					step += 1;
					continue;
				}
			} else if (step == 3) {
				catapult_group.move(MAX_VOLTAGE);
				if (slip_angle < 100) {
					step = 1;
					continue;
				}
			}

			// Timer starts when auto starts
			if (auto_timer.GetElapsedTime().AsMilliseconds() >=
			    35000)
				break;

			Timer jam_timer = Timer();
			if (catapult_group.get_current_draws()[0] > 1750) {
				bool do_unjam = true;
				while (jam_timer.GetElapsedTime()
					       .AsMilliseconds() < 500) {
					if (catapult_group
						    .get_current_draws()[0] <
					    1750) {
						do_unjam = false;
						break;
					}
				}
				if (do_unjam) {
					catapult_group.move(-MAX_VOLTAGE);
					pros::delay(650);
					catapult_group.move(0);
					pros::delay(500);
				}
			}

			pros::delay(5);
		}
		catapult_group.move(-MAX_VOLTAGE);
		pros::delay(500);
		catapult_group.move(0);
	});
	// Home after firing
	auto_sequence.drive_power(-MAX_VOLTAGE * 0.35, -MAX_VOLTAGE * 0.1,
				  1000);
	// Go to post
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 33, MAX_RPM, 2500);
	auto_sequence.turn_imu(Direction::Clockwise, 45, MAX_VOLTAGE / 1.5, 1500);
	auto_sequence.wait_until_match_time(41.0);
	auto_sequence.move_position(DRIVE_UNITS_PER_INCH * 20, MAX_RPM / 2.0, 2500);
#endif

	auto_sequence.run_auto();

	left_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	right_drive_group.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

std::unique_ptr<Timer> intake_extension_toggle_timer =
	std::make_unique<Timer>();
bool is_intake_extended = false;

std::unique_ptr<Timer> left_wing_toggle_timer = std::make_unique<Timer>();
bool left_wing_deployed = false;

std::unique_ptr<Timer> right_wing_toggle_timer = std::make_unique<Timer>();
bool right_wing_deployed = false;

bool catapult_button_timer_running = false;
std::unique_ptr<Timer> catapult_button_timer = std::make_unique<Timer>();

bool climb_arm_deployed = false;
bool climb_trigger_timer_running = false;
std::unique_ptr<Timer> climb_trigger_timer = std::make_unique<Timer>();

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
	initCommon(false);

	if (!catapult_deployed_in_auto)
		set_deploy_catapult();

	while (true) {
		handle_catapult_deploy();

		// ctrl.print(0, 0, "%i", catapult_group.get_current_draws()[0]);
		// ctrl.set_text(0, 0, std::to_string(imu.get_rotation()));

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

		// Left wing control code
		if (ctrl.get_digital(DIGITAL_DOWN) &&
		    left_wing_toggle_timer->GetElapsedTime().AsMilliseconds() >
			    200) {
			left_wing_deployed = !left_wing_deployed;
			left_wing.set_value(left_wing_deployed);
			left_wing_toggle_timer->Restart();
		}

		// Right wing control code
		if (ctrl.get_digital(DIGITAL_B) &&
		    right_wing_toggle_timer->GetElapsedTime().AsMilliseconds() >
			    200) {
			right_wing_deployed = !right_wing_deployed;
			right_wing.set_value(right_wing_deployed);
			right_wing_toggle_timer->Restart();
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
			bool do_reverse_catapult = ctrl.get_digital(DIGITAL_UP);
			if (do_reverse_catapult) {
				catapult_group.move(-MAX_VOLTAGE);
			} else if (do_fire_catapult) {
				catapult_group.move(MAX_VOLTAGE);
				catapult_block.brake();
				// pros::lcd::set_text(1, std::to_string(catapult_group.get_current_draws()[0]));
			} else {
				catapult_group.brake();
				catapult_block.brake();
			}

			bool do_place_block = ctrl.get_digital(DIGITAL_LEFT);
			bool do_remove_block = ctrl.get_digital(DIGITAL_RIGHT);
			if (do_place_block) {
				catapult_block.move(-MAX_VOLTAGE);
			} else if (do_remove_block) {
				catapult_block.move(MAX_VOLTAGE);
			}

			bool do_deploy_catapult = ctrl.get_digital(DIGITAL_X);
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
