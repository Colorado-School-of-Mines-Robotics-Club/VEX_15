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

enum class AutonomousStep {
	MoveBackStart,
	ExtendIntake,
	RetractIntake,
	MoveBackBeforeFirstTurn,
	TurnTowardsGoal1,
	MoveTowardsGoal1,
	TurnTowardsGoal2,
	MoveTowardsGoal2,
	TurnTowardsGoal3,
	MoveTowardsGoal4,
	MoveAwayFromGoal1,
	TurnAwayFromGoal1,
	MoveAwayFromGoal2,
	TurnAwayFromGoal2,
	MoveAwayFromGoal3,
	TurnAwayFromGoal3,
	WaitForCatapultDeploy,
	TurnAwayFromGoal4,
	FireCatapult,
	WaitForCatapultEngage,
	WaitForCatapultSlip,
	MoveBackAgainstPipe,
	Done,
};

#define INTAKE_EXTENDED_POSITION 170
#define INTAKE_RETRACTED_POSITION 60

#define DRIVE_UNITS_PER_INCH 27.46290005363848
#define DRIVE_UNITS_PER_DEGREE 3.1

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

	Timer auto_change_timer;
	AutonomousStep sequence_step = AutonomousStep::MoveBackStart;
	// AutonomousStep sequence_step = AutonomousStep::MoveBackAgainstPipe;
	left_drive_group.tare_position();
	right_drive_group.tare_position();

	bool do_catapult_deploy = false;
	bool running_auto = true;
	while (running_auto) {
		if (do_catapult_deploy)
			handle_catapult_deploy();

		switch (sequence_step) {
		case AutonomousStep::MoveBackStart:
#define MOVE_BACK_START_DISTANCE -DRIVE_UNITS_PER_INCH * 1.5
			left_drive_group.move_absolute(MOVE_BACK_START_DISTANCE,
						       MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_BACK_START_DISTANCE, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
			    MOVE_BACK_START_DISTANCE + 1) {
				auto_change_timer.Restart();
				sequence_step = AutonomousStep::ExtendIntake;
			}
			break;
		case AutonomousStep::ExtendIntake:
			left_drive_group.brake();
			right_drive_group.brake();
			intake_extension_group.move_absolute(
				INTAKE_EXTENDED_POSITION, MAX_RPM / 2);
			intake_spin_group.move(-MAX_VOLTAGE);
			if (auto_change_timer.GetElapsedTime().AsMilliseconds() >
			    1000) {
				auto_change_timer.Restart();
				sequence_step = AutonomousStep::RetractIntake;
			}
			break;
		case AutonomousStep::RetractIntake:
			intake_extension_group.move_absolute(
				INTAKE_RETRACTED_POSITION, MAX_RPM);
			if (auto_change_timer.GetElapsedTime().AsMilliseconds() >
			    500) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveBackBeforeFirstTurn;
				intake_spin_group.move(0);
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveBackBeforeFirstTurn:
#define MOVE_BACK_BEFORE_TURN_TOWARDS_GOAL_DISTANCE -DRIVE_UNITS_PER_INCH * 2.5
			left_drive_group.move_absolute(
				MOVE_BACK_BEFORE_TURN_TOWARDS_GOAL_DISTANCE,
				MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_BACK_BEFORE_TURN_TOWARDS_GOAL_DISTANCE,
				MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
			    MOVE_BACK_BEFORE_TURN_TOWARDS_GOAL_DISTANCE + 1) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnTowardsGoal1;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnTowardsGoal1:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 80, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -80, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] >
			    DRIVE_UNITS_PER_DEGREE * 80 - 2) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveTowardsGoal1;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveTowardsGoal1:
#define MOVE_TOWARDS_GOAL_1_DISTANCE DRIVE_UNITS_PER_INCH * 7
			left_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_1_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_1_DISTANCE, MAX_RPM / 4);
			intake_spin_group.move(-MAX_VOLTAGE * 0.5);
			if (left_drive_group.get_positions()[0] >
			    MOVE_TOWARDS_GOAL_1_DISTANCE - 2) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnTowardsGoal2;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnTowardsGoal2:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 150, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -150, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] >
			    DRIVE_UNITS_PER_DEGREE * 150 - 2) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveTowardsGoal2;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveTowardsGoal2:
#define MOVE_TOWARDS_GOAL_2_DISTANCE DRIVE_UNITS_PER_INCH * -8
			left_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_2_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_2_DISTANCE, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
			    MOVE_TOWARDS_GOAL_2_DISTANCE + 2) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnTowardsGoal3;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnTowardsGoal3:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -85, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 85, MAX_RPM / 4);
			if (right_drive_group.get_positions()[0] >
			    DRIVE_UNITS_PER_DEGREE * 85 - 2) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveTowardsGoal4;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
				intake_spin_group.move(0);
			}
			break;
		case AutonomousStep::MoveTowardsGoal4:
#define MOVE_TOWARDS_GOAL_4_DISTANCE DRIVE_UNITS_PER_INCH * 14
			left_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_4_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_TOWARDS_GOAL_4_DISTANCE, MAX_RPM / 4);
			if (auto_change_timer.GetElapsedTime().AsMilliseconds() >
			    2500) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveAwayFromGoal1;
				left_drive_group.move(0);
				right_drive_group.move(0);
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveAwayFromGoal1:
#define MOVE_AWAY_FROM_GOAL_1_DISTANCE DRIVE_UNITS_PER_INCH * -9
			left_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_1_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_1_DISTANCE, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
			    MOVE_AWAY_FROM_GOAL_1_DISTANCE + 1) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnAwayFromGoal1;
				left_drive_group.move(0);
				right_drive_group.move(0);
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnAwayFromGoal1:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -20, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 20, MAX_RPM / 4);
			if (right_drive_group.get_positions()[0] >
			    DRIVE_UNITS_PER_DEGREE * 20 - 1) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveAwayFromGoal2;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveAwayFromGoal2:
#define MOVE_AWAY_FROM_GOAL_2_DISTANCE DRIVE_UNITS_PER_INCH * -3
			left_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_2_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_2_DISTANCE, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
			    MOVE_AWAY_FROM_GOAL_2_DISTANCE + 1) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnAwayFromGoal2;
				left_drive_group.move(0);
				right_drive_group.move(0);
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnAwayFromGoal2:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -10, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 10, MAX_RPM / 4);
			if (right_drive_group.get_positions()[0] >
			    DRIVE_UNITS_PER_DEGREE * 10 - 1) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::MoveAwayFromGoal3;
				do_catapult_deploy = true;
				deploy_catapult();
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::MoveAwayFromGoal3:
#define MOVE_AWAY_FROM_GOAL_3_DISTANCE DRIVE_UNITS_PER_INCH * -6
			left_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_3_DISTANCE, MAX_RPM / 4);
			right_drive_group.move_absolute(
				MOVE_AWAY_FROM_GOAL_3_DISTANCE, MAX_RPM / 4);
			if (left_drive_group.get_positions()[0] <
				    MOVE_AWAY_FROM_GOAL_3_DISTANCE + 1 ||
			    auto_change_timer.GetElapsedTime().AsMilliseconds() >
				    500) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnAwayFromGoal3;
				left_drive_group.move(0);
				right_drive_group.move(0);
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnAwayFromGoal3:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -10, MAX_RPM / 4);
			right_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * -160, MAX_RPM / 4);
			if (right_drive_group.get_positions()[0] <
				    DRIVE_UNITS_PER_DEGREE * -160 + 1 ||
			    auto_change_timer.GetElapsedTime().AsMilliseconds() >
				    1500) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::WaitForCatapultDeploy;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::WaitForCatapultDeploy:
			if (catapult_deploy_status ==
			    CatapultDeployStatus::NotDeploying) {
				auto_change_timer.Restart();
				sequence_step =
					AutonomousStep::TurnAwayFromGoal4;
				left_drive_group.tare_position();
				right_drive_group.tare_position();
			}
			break;
		case AutonomousStep::TurnAwayFromGoal4:
			left_drive_group.move_absolute(
				DRIVE_UNITS_PER_DEGREE * 10, MAX_RPM);
			right_drive_group.move(-30);
			if (left_drive_group.get_positions()[0] >
				    DRIVE_UNITS_PER_DEGREE * 10 - 2 ||
			    auto_change_timer.GetElapsedTime().AsMilliseconds() >
				    300) {
				auto_change_timer.Restart();
				sequence_step = AutonomousStep::FireCatapult;
				left_drive_group.move(0);
				right_drive_group.move(0);
			}
			break;
		case AutonomousStep::FireCatapult:
			catapult_group.move(MAX_VOLTAGE);
			catapult_block.brake();
			if (auto_change_timer.GetElapsedTime().AsSeconds() >
			    20) {
				sequence_step =
					AutonomousStep::WaitForCatapultEngage;
				catapult_group.move(0);
			}
			break;
		case AutonomousStep::WaitForCatapultEngage:
			catapult_group.move(MAX_VOLTAGE);
			catapult_block.brake();
			if (catapult_group.get_current_draws()[0] > 500) {
				sequence_step =
					AutonomousStep::WaitForCatapultSlip;
				catapult_group.move(0);
			}
			break;
		case AutonomousStep::WaitForCatapultSlip:
			catapult_group.move(MAX_VOLTAGE);
			catapult_block.brake();
			if (catapult_group.get_current_draws()[0] < 300) {
				sequence_step = AutonomousStep::Done;
				catapult_group.move(0);
			}
			break;
		case AutonomousStep::MoveBackAgainstPipe:
			left_drive_group.move(-30);
			right_drive_group.move(-30);
			if (left_drive_group.get_current_draws()[0] > 1000 &&
			    right_drive_group.get_current_draws()[0] > 1000) {
				sequence_step = AutonomousStep::Done;
				catapult_group.move(0);
			}
			break;
		case AutonomousStep::Done:
			running_auto = false;
			left_drive_group = 0;
			right_drive_group = 0;
			catapult_group = 0;
			intake_spin_group = 0;
			break;
		}

		pros::delay(5);
	}
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
			bool do_collapse_catapult = ctrl.get_digital(DIGITAL_B);
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
					deploy_catapult();
				}
			} else {
				catapult_button_timer_running = false;
			}
		}

		pros::delay(5);
	}
}
