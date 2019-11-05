#include "config.h"
// Values
double leftDriveVoltage, rightDriveVoltage;
double minControllerDrivePct = 2;
/*
 *
 * Drive function using direct voltage output as units for travel
 *
*/
//     vdrive(Controller.Axis3.value()*100/127.0, Controller.Axis2.value()*100/127.0);
void drive() {
  arm.setStopping(brakeType::hold);
  while(true) {
    double powerReduction = .9;

    leftDriveVoltage = powerReduction * Controller.Axis3.value() * 12.0 / 127.0;
    rightDriveVoltage = powerReduction * Controller.Axis2.value() * 12.0 / 127.0;

    // The controller may have sensitivity issues near the default hold point.  This adjusts for that
    if (leftDriveVoltage < minControllerDrivePct && leftDriveVoltage > -minControllerDrivePct) { leftDriveVoltage = 0; }
    if (rightDriveVoltage < minControllerDrivePct && rightDriveVoltage > -minControllerDrivePct) { rightDriveVoltage = 0; }

    frontRight.spin(vex::directionType::fwd, rightDriveVoltage, vex::voltageUnits::volt);
    backRight.spin(vex::directionType::fwd, rightDriveVoltage, vex::voltageUnits::volt);
    frontLeft.spin(vex::directionType::fwd, leftDriveVoltage, vex::voltageUnits::volt);
    backLeft.spin(vex::directionType::fwd, leftDriveVoltage, vex::voltageUnits::volt);


     vex::task::sleep(10);
  }
}

// Spin the intake flippers 
// This uses the following motors:
//if R1 is pressed, intake intakes at 100 until unpressed
//if R2 pressed, intake outakes at -100 until unpressed
//if R1 and (shiftKey) L2 pressed, intake intakes at 50 until unpressed
//if R2 and (shiftKey) L2 pressed, intake outakes at -50 until unpressed
void spinIntake() {
  int pctMax = 70;
  int pct = 0;
  bool prev = false, current = false;
  while(1) {
    if(Controller.ButtonR1.pressing()){
      if(prev == current) {
        current = !current;
      } 
    } 
    else {
      if(prev != current) {
        prev = current;
      }
    }
    if(current)
      pct = 100;
    else 
      pct = 0;
    
    if(Controller.ButtonR2.pressing()){
      current = false;
      pct = -100;
    } else if(Controller.ButtonB.pressing()){
      current = false;
      pct = 100;
    }

    if (pct != 0) {
      rightIntakeFlipper.spin(vex::directionType::fwd, pct, vex::velocityUnits::pct);
      leftIntakeFlipper.spin(vex::directionType::fwd, pct, vex::velocityUnits::pct);
    } else {
      rightIntakeFlipper.stop(vex::brakeType::brake);
      leftIntakeFlipper.stop(vex::brakeType::brake);
    }
    
    vex::task::sleep(10);
  }
}

void moveArm() {

    int pct = 0;
  while(1) {

    //test stuff
    if (Controller.ButtonUp.pressing()){
      pct = -70;
    } else if (Controller.ButtonDown.pressing()){
      pct = 70;
    } else {
      pct = 0;
    }
    
      arm.spin(vex::directionType::fwd, pct, vex::velocityUnits::pct);
    
    
    
    vex::task::sleep(10);
  }
}


void moveTray() {

    int pct = 0;
  while(1) {

    //test stuff
    if (Controller.ButtonL2.pressing()){
      pct = 95;
    } else if (Controller.ButtonL1.pressing()){
      pct = -95;
    } else {
      pct = 0;
    }

    
    if (pct!= 0) {

      // If the limit switch is pressed and we are trying to drive tray into the robot frame,
      //  then we don't want to actually allow the tray to move
      int trayLimitHit = trayLimit.value();
      if (!trayLimitHit || pct < 0) {
        tray.spin(vex::directionType::fwd, pct, vex::velocityUnits::pct);
      } else {
        tray.stop(vex::brakeType::brake);
      }
    } else {
      tray.stop(vex::brakeType::brake);
    } 
    vex::task::sleep(10);
  }
}

#include "vex.h"
#include <cmath>
#include <ratio>
#include <vector>
using namespace vex;

double squareDistance = 1.79;
double rightTurnDistance = 0.75;
//double fullTurnDistance = 2.0;

double distance1a = 3; //to pick up cubes
double distance1b = 3-1.133; //backward
double turn1a = 0.99; //turn to face zone
double distance1c = 1.91-1.139; //drive to zone
double distance1d = 2; //back away from zone

double distance2a = 1; //distance to  cube
double turn2a = 0.5; //turn toward cube
double distance2b = 2.269-0.7; //drive to cube
double turn2b = 0.5; //turn toward zone
double distance2c = 0.2*squareDistance; //drive to zone
double distance2d = distance2d;//back away from zone

////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////AUTON_FUNCTIONS//////////
void wait(int millis) {
  vex::task::sleep(millis);
}

void setBraking() {
  frontLeft.setStopping(vex::brakeType::brake);
  frontRight.setStopping(vex::brakeType::brake);
  backLeft.setStopping(vex::brakeType::brake);
  backRight.setStopping(vex::brakeType::brake);
}

void setHolding() {
  frontLeft.setStopping(vex::brakeType::hold);
  frontRight.setStopping(vex::brakeType::hold);
  backLeft.setStopping(vex::brakeType::hold);
  backRight.setStopping(vex::brakeType::hold);
}

void setCoasting() {
  frontLeft.setStopping(vex::brakeType::coast);
  frontRight.setStopping(vex::brakeType::coast);
  backLeft.setStopping(vex::brakeType::coast);
  backRight.setStopping(vex::brakeType::coast);
}
void stopAll() {
  frontLeft.stop();
  backLeft.stop();
  frontRight.stop();
  backRight.stop();
}

//drive for a given distance, uses built-in encoder function
//program will wait for the drive to finish if wait == true
void basicEncoderDrive(double pct, double rev, bool wait, double timeLimit) {
  clock_t start;
  start = clock();
  frontLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  backLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  frontRight.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  if (wait) {
      backRight.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
        while (clock() - start / CLOCKS_PER_SEC < timeLimit) {}
    return;
  }
  backRight.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
}

void basicEncoderDrive(double pct, double rev, bool wait) {
  frontLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  backLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  frontRight.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  if (wait) {
      backRight.rotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
    return;
  }
  backRight.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
}

//turn in place for a given distance per wheel, uses built-in encoder function
//program will wait for the turn to finish if wait == true
void basicEncoderTurn(double pct, double rev, bool wait) {
  frontLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  backLeft.startRotateFor(rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  frontRight.startRotateFor(-rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  if (wait) {
    backRight.rotateFor(-rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  } else {
    backRight.startRotateFor(-rev, vex::rotationUnits::rev, pct, vex::velocityUnits::pct);
  }
}


//RedAuto, picks up 4 cubes
void RedAuto(){
  setBraking();
  
  rightIntakeFlipper.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  leftIntakeFlipper.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

  basicEncoderDrive(30,4,true);
  
  basicEncoderDrive(30,-4,true);

  rightIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  leftIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);

  frontLeft.spin(vex::directionType::fwd, 50,vex::velocityUnits::pct);
  backLeft.spin(vex::directionType::fwd, 50,vex::velocityUnits::pct);
  frontRight.spin(vex::directionType::rev, 50,vex::velocityUnits::pct);
  backRight.spin(vex::directionType::rev, 50,vex::velocityUnits::pct);
  wait(500);

  stopAll();
  basicEncoderDrive(30,.35,true, 2);

  tray.spin(vex::directionType::fwd,  50, vex::velocityUnits::pct);

  wait(3000);

  tray.spin(vex::directionType::fwd,  0, vex::velocityUnits::pct);

  basicEncoderDrive(35,-1,true, 2);
}
#pragma endregion