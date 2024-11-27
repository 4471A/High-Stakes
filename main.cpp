#include "main.h"
#include "functions.hpp"
#include "timing.hpp"
#include <cmath>
#include <vector>
using namespace pros;

Controller master(E_CONTROLLER_MASTER);
    Motor leftFrontMtr(11);
	Motor leftMidMtr(12);
	Motor leftBackMtr(13);
    Motor rightFrontMtr(14);
	Motor rightMidMtr(15);
    Motor rightBackMtr(16);
	Motor conveyorMtrL(6);
	Motor conveyorMtrR(7);
    Imu inertial_sensor(13);
    //Imu gps_sensor(4);
	ADIDigitalOut armFlapL(8);
    ADIDigitalOut armFlapR(9);
	ADIDigitalOut lock(10);
	Rotation rWE(20);
	Rotation lWE(119);






void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "4471A!");
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
	pros::lcd::initialize();
	inertial_sensor.reset();
	//pros::gps::initialize();
	delay(2000);
	//pusharm.set_value(1);

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
void competition_initialize() {
	inertial_sensor.tare_rotation();
	inertial_sensor.tare_pitch();
	leftFrontMtr.tare_position();
	leftMidMtr.tare_position();
	leftBackMtr.tare_position();
	rightFrontMtr.tare_position();
	rightMidMtr.tare_position();
	rightBackMtr.tare_position();
	conveyorMtrL.tare_position();
	conveyorMtrR.tare_position();
	rWE.set_position(0);
	lWE.set_position(0);

	
}

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
void autonomous(){ 
	inertial_sensor.tare_rotation();
	inertial_sensor.tare_pitch();
	leftFrontMtr.tare_position();
	leftMidMtr.tare_position();
	leftBackMtr.tare_position();
	rightFrontMtr.tare_position();
	rightMidMtr.tare_position();
	rightBackMtr.tare_position();
	conveyorMtrL.tare_position();
	conveyorMtrR.tare_position();
	rWE.set_position(0);
	lWE.set_position(0);

	//skills

	/*
	
	//odometryTracking(70, 31, 8, false, false, 5000);

	odometryTracking(115, 31, 4.5, false, 0, false, 2000);
	//delay(500);
	odometryTracking(126, 6.5, 2.5, false, 0, false, 2000);
	armFlapL.set_value(1);
	spinToWin(29000);
	armFlapL.set_value(0);
	//delay(1500);
	intakeOn(-127);
	odometryTracking(140, 38, 5, false, 0, false, 2000);
	//delay(500);
	odometryTracking(135, 110, 3, false, 0, false, 3000);
	//delay(500);
	odometryTracking(65, 112, 100, true, 1, true, 3000);
	//delay(500);
	//odometryTracking(110, 125, 4, false, 0, false, 2000); //small movement
	//delay(500);
	//odometryTracking(65, 122, 100, false, 0, true, 4000); //small movement
	//delay(500);
	//odometryTracking(121, 130, 5, false, false, 2000);
	//delay(5000);
	//odometryTracking(65, 140, 100, false, true, 3000);
	//delay(5000);
	odometryTracking(103, 130, 3, true, 2, false, 2000); //120, 127
	doFlaps(1);
	//delay(10000);
	odometryTracking(70, 85, 4, false, 0, false, 2000);
	//delay(5000);
	//odometryTracking(66, 86, 3, false, 0, false, 2000);
	doFlaps(1);
	//armFlapR.set_value(1);
	//delay(5000);
	//odometryTracking(75, 135, 100, true, 3, false, 4000);
	//delay(5000);
	//armFlapR.set_value(0);
	odometryTracking(70, 135, 100, false, 0, false, 3000);
	//doFlaps(0);
	//delay(5000);
	odometryTracking(75, 75, 3, false, 0, false, 2000);
	//delay(5000);
	odometryTracking(-10, 75, 5, false, 0, false, 2000);
	doFlaps(1);
	//delay(5000);
	odometryTracking(110, 135, 100, false, 0, false, 3000);
	doFlaps(0);

	odometryTracking(72, 80, 3, false, 0, false, 2000);
	
	

	//match not loading 
	//odometryTracking(65, 112, 100, true, 1, true, 3000);
	*/

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
 * If the rohi bot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */ 


void opcontrol() {


	int intakeState = 0;
 	int revintakeState = 0;
	int armFlapStateL = 0;
	int armFlapStateR = 0;
	int lockState = 0;
	int liftState = 0;
	inertial_sensor.tare_rotation();
	inertial_sensor.tare_pitch();
	leftFrontMtr.tare_position();
	leftMidMtr.tare_position();
	leftBackMtr.tare_position();
	rightFrontMtr.tare_position();
	rightMidMtr.tare_position();
	rightBackMtr.tare_position();
	conveyorMtrL.tare_position();
	conveyorMtrR.tare_position();
	rWE.set_position(0);
	lWE.set_position(0);
	
	/*
	while (true) {
		odometryTracking(15, 15, 5, false, 0 , false, 2000);
	}
	*/

	/*
	while (true) {
		pros::lcd::set_text(2, int(leftArm.get_position()));
		pros::lcd::set_text(3, int(rightArm.get_position()));
		delay(1000);
	}
	*/
 
 	while (true) {

 		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
 		leftFrontMtr = -leftY - turn;
 		leftMidMtr = -leftY - turn;
		leftBackMtr = -leftY - turn;
 		rightFrontMtr = leftY - turn;
		rightMidMtr = leftY - turn;
 		rightBackMtr = leftY - turn;

 		if (master.get_digital_new_press(DIGITAL_R1) == 1) {
 			if (intakeState == 0) { 
 				intakeState = 1;
 				//intakeMtr.move_voltage(10000);
				intakeOn(127);
 			}
 			else if (intakeState == 1) {
 				intakeState = 0;
 				//intakeMtr.move_voltage(0);
				intakeOff();
 			}
 		}

		if (master.get_digital_new_press(DIGITAL_X) == 1) {
 			if (revintakeState == 0) {
 				revintakeState = 1;
				revIntake(127);
 			} 
 			else if (revintakeState == 1) {
 				revintakeState = 0;
 				intakeOff();
 			}
		}

		if (master.get_digital_new_press(DIGITAL_DOWN) == 1) {
			if (lockState == 0) {
				moveLock(1);
				lockState = 1;
			}
			else {
				moveLock(0);
				lockState = 0;
			}
		}
 	}
}