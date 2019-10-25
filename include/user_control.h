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
  while(1) {
    leftDriveVoltage = 0.6 * Controller.Axis3.value() * 12.0 / 127.0;
    rightDriveVoltage = 0.6 * Controller.Axis2.value() * 12.0 / 127.0;

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

  while(1) {

    // Sets the pct speed of the intake
    if (Controller.ButtonR1.pressing()){
      pct = 100;
    }
    else if(Controller.ButtonR2.pressing()){
      pct = -100;
    }
    else if (Controller.ButtonR1.pressing() && Controller.ButtonL2.pressing()){
      pct = 50;
    }
    else if (Controller.ButtonR2.pressing() && Controller.ButtonL2.pressing()){
      pct = -50;
    } else {
      pct = 0;
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
      pct = 70;
    } else if (Controller.ButtonL1.pressing()){
      pct = -70;
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