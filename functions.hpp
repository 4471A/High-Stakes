#ifndef _FUNCTIONS_HPP_
#define _FUNCTIONS_HPP_
/*
void flyWheelSpin();
void pushWheelSpin();*/
/*
RWEMEBER
nuclear bomb is the right catapult !!!
twentytwomilimeterantiaircraftgun is the left catapult!!!
*/

void forceMove(double power, double time);
void intakeOn(int power);
void intakeOff();
void revIntake(int power);
void moveLock(int position);
double getRotation();
void odometryTracking(double x, double y, double kP, bool curve, int type, bool reset, double time);
#endif