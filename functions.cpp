#include "functions.hpp"
#include "timing.hpp"
#include "main.h"
#include <cmath>
#include <vector>

using namespace pros;

void forceMove(double power, double time)
{
  startTimer(0);
  while (getTime(0) < time)
  {
    leftFrontMtr = -power;
    leftMidMtr = -power;
    leftBackMtr = -power;
    rightFrontMtr = power;
    rightMidMtr = power;
    rightBackMtr = power;
  }
  leftFrontMtr = 0;
  leftMidMtr = 0;
  leftBackMtr = 0;
  rightFrontMtr = 0;
  rightMidMtr = 0;
  rightBackMtr = 0;
}

void intakeOn(int power)
{
  conveyorMtrL = -power;
  conveyorMtrR = -power;
}

void intakeOff()
{
  conveyorMtrL = 0;
  conveyorMtrR = 0;
}

void revIntake(int power)
{
  conveyorMtrL = power;
  conveyorMtrR = power;
}

void moveLock(int position) {

  if (position == 1) {
    lock.set_value(1);
  }
  if (position == 0) {
    lock.set_value(0);
  }
}

double calculateRotation(double x1, double y1, double x2, double y2, double initialRotation) {

  double a = atan2(y2 - y1, x2 - x1);
  double degree = -(a * (180 / M_PI) - 90);
  return degree - initialRotation;

}
// r for right, l for left, p for previous, d for delta, c for current
// WEV = wheel encoder value in radians
// WTD = wheel travel distance in inches
// T = Theta
// DC = displacement cycle

//(btw call this function a few million times second or as much as the brain allows before exploding)
// read all the when sensors or encoders here
// assign values to variables
// read the motor encoders here
// note only works if encoder values is continuous, and it never resets. if it does, replace "these" encodervalues with their
// respective "previous values"

double xCoord = 137; //136.45
double yCoord = 31; //35
double avgD;
double absT = -90; //-90
double pRWEV = 0;
double pLWEV = 0;

void odometryTracking(double x, double y, double kP, bool useBack, int type, bool reset, double time) {
  
  double rWEV;
  double lWEV;
  double wheelOR = 1.375;
	double wheelSpace = 4;

  double dT = 0;
  double dRWEV = 0;
  double dLWEV = 0;

  double accel = 1;
  double turnAccel = 1;
  double vP = 0;
  double vO = 0;
  double vF = 1;
  double target = sqrt(pow(x - xCoord, 2) + pow(y - yCoord, 2));
  double turnTarget = calculateRotation(xCoord, yCoord, x, y, absT);
  double traveled = 0;
  double turned = 0;
  double error = target - traveled;
  double turnError = turnTarget + turned;
  double power = error * kP;
  double turnPower = turnError * 1.25;
  bool finishedTurning = false;

  if (turnTarget > 180) {
    turnTarget -= 360;
  }
  if (turnTarget <= -180) {
    turnTarget += 360;
  }

  int turnCount = 0;
  int count = 0;

  startTimer(0);

  while (accel != 0 && getTime(0) < time /*true*/) {

    rWEV = rWE.get_position() / 100;
    lWEV = lWE.get_position() / 100;
    // int bWEV = somethings;

    // these:
    dRWEV = rWEV - pRWEV;
    dLWEV = lWEV - pLWEV;

    // how much the wheel traveled is the radius times the angle turned in radians so s = r*theta (radians)
    // so its like radius of wheel times rightwheelencodervalue or leftwheelencodervalue
    double rWTD = rWEV / 360 * 2 * M_PI * wheelOR;
    double lWTD = lWEV / 360 * 2 * M_PI * wheelOR;
    avgD += (lWTD + rWTD) / 2;
    //+= (lWTD + rWTD) / 2;
    // int backWheelTravelInch = deltaBWHeelEncoder * wheelOD;

    // dT = ((rWTD - lWTD) / wheelSpace) + dT;
    dT = (dLWEV - dRWEV) / 3.92;
    absT -= (dLWEV - dRWEV) / 3.92;
    //-= (dLWEV - dRWEV) / 4 * 180 / M_PI;

    if (absT > 180) {
      absT -= 360;
    }
    if (absT <= -180) {
      absT += 360;
    }
    

    double dDC = -(dLWEV + dRWEV) / 2 / 360 * 2 * M_PI * wheelOR;

    if (absT > 0 && absT < 90)
    {
      xCoord += dDC * sin(absT * M_PI / 180);
      yCoord += dDC * cos(absT * M_PI / 180);
      //if (useBack) {
        //xCoord -= 2 * sin(absT * M_PI / 180);
        //yCoord -= 2 * cos(absT * M_PI / 180);
      //}
    }
    else if (absT > 90 && absT < 180)
    {
      xCoord += dDC * cos((absT - 90) * M_PI / 180);
      yCoord -= dDC * sin((absT - 90) * M_PI / 180);
      //if (useBack) {
        //xCoord -= 2 * cos((absT - 90) * M_PI / 180);
        //yCoord += 2 * sin((absT - 90) * M_PI / 180);
      //}
    }
    else if (absT > -90 && absT < 0)
    {
      xCoord -= dDC * sin(-absT * M_PI / 180);
      yCoord += dDC * cos(-absT * M_PI / 180);
      //if (useBack) {
        //xCoord += 2 * sin(-absT * M_PI / 180);
        //yCoord -= 2 * cos(-absT * M_PI / 180);
      //}
    }
    else if (absT > -180 && absT < -90)
    {
      xCoord -= dDC * cos((-absT - 90) * M_PI / 180);
      yCoord -= dDC * sin((-absT - 90) * M_PI / 180);
      //if (useBack) {
        //xCoord += 2 * cos((-absT - 90) * M_PI / 180);
        //yCoord += 2 * sin((-absT - 90) * M_PI / 180);
      //}
    }

    // updating the cordinates ^ (inches)

    //linear movement
    if (power > 30 && error > 0.3) { 

    }
    if (!useBack) {
      if (power < 30 && error > 0.3) {
        power = 30;
      }
      if (power < 30 && error < 0) {
        power = -20;
      }
    }


    //turning movement in degrees
    /*
    if (turnPower < 40) {
      turnPower = 40;
    }
    if (turnPower <= 40 && turnError > 2.5) {
      turnPower = 35;
    }
    if (turnPower <= 40 && turnError < -2.5) {
      turnPower = -35;
    }
    if (turnPower >= 40 && (turnError < 2.5 && turnError > -2.5)) {
      turnPower = 35;
    */

    if (turnPower < 20) {
      turnPower = turnPower * 2;
    }

    //movement
    
    if (turnError < 2.5 && turnError > -2.5 && turnAccel < 10e-2) {
      finishedTurning = true;
      //goto skip;
    }           
    
    if (!finishedTurning && !useBack) {
      leftFrontMtr = -turnPower;
      leftMidMtr = -turnPower;
      leftBackMtr = -turnPower;
      rightFrontMtr = -turnPower;
      rightMidMtr = -turnPower;
      rightBackMtr = -turnPower;
    }
    
    if ((finishedTurning || getTime(0) > 1000) && !useBack) {
      leftFrontMtr = -power - turnPower;
      leftMidMtr = -power - turnPower;
      leftBackMtr = -power - turnPower;
      rightFrontMtr = power - turnPower;
      rightMidMtr = power - turnPower;
      rightBackMtr = power - turnPower;
    }

    /*
    if ((finishedTurning || getTime(0) > 1000) && curve) {
      if (type == 1) {
        leftFrontMtr = -40;
        leftMidMtr = -40;
        leftBackMtr = -40;
        rightFrontMtr = 300;
        rightMidMtr = 300;
        rightBackMtr = 300;
      }
      if (type == 2) {
        leftFrontMtr = 10;
        leftMidMtr = 10;
        leftBackMtr = 10;
        rightFrontMtr = -300;
        rightMidMtr = -300;
        rightBackMtr = -300;
      }
      if (type == 3) {
        leftFrontMtr = -300;
        leftMidMtr = -300;
        leftBackMtr = -300;
        rightFrontMtr = 50;
        rightMidMtr = 50;
        rightBackMtr = 50;
      }
    }
    */

    skip:
    
    //pros::lcd::set_text(1, std::to_string(dT));
    pros::lcd::set_text(1, std::to_string(absT));
    pros::lcd::set_text(2, std::to_string(xCoord));
    pros::lcd::set_text(3, std::to_string(yCoord));
    pros::lcd::set_text(4, std::to_string(turnTarget));
    pros::lcd::set_text(5, std::to_string(turnError));
    pros::lcd::set_text(6, std::to_string(turnPower));
    pros::lcd::set_text(7, std::to_string(turnAccel));
    delay(10);
    
    // update "previous" values
    vP = vO;
    vO = vF;
    if (reset) {
      xCoord = 101;
      yCoord = 134;
      absT = -90;
    }
    else {
      traveled += dDC;
      //if (useBack) {
        //traveled -= 2 * dDC;
      //}
      vF = dDC;
    }
    turned += dT;
    error = target - traveled;
    turnError = turnTarget + turned;
    power = error * kP;
    turnPower = turnError * 1.25;
    pRWEV = rWEV;
    pLWEV = lWEV;

    if (finishedTurning) {
      count++;
    }
    turnCount++;
    if (turnCount > 50) {
      turnAccel = vF - vO - vP/ (getTime(0) + 0.000001);
    }
    if (count > 50) {
      accel = vF - vO - vP/ (getTime(0) + 0.000001);
    }
  }

  for (int i = 0; i < 20; i++) {
    leftFrontMtr = 0;
    leftMidMtr = 0;
    leftBackMtr = 0;
    rightFrontMtr = 0;
    rightMidMtr = 0;
    rightBackMtr = 0;
    delay(10);
  }
}

void odometry(double x, double y, double kP, bool useBack, int type, bool reset, double time) {
}