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
    leftDriveVoltage = Controller.Axis3.value() * 12.0 / 127.0;
    rightDriveVoltage = Controller.Axis2.value() * 12.0 / 127.0;

    // The controller may have sensitivity issues near the default hold point.  This adjusts for that
    if (leftDriveVoltage < minControllerDrivePct && leftDriveVoltage > -minControllerDrivePct) { leftDriveVoltage = 0; }
    if (rightDriveVoltage < minControllerDrivePct && rightDriveVoltage > -minControllerDrivePct) { rightDriveVoltage = 0; }

    frontRight.spin(vex::directionType::fwd, leftDriveVoltage, vex::voltageUnits::volt);
    backRight.spin(vex::directionType::fwd, leftDriveVoltage, vex::voltageUnits::volt);
    frontLeft.spin(vex::directionType::fwd, rightDriveVoltage, vex::voltageUnits::volt);
    backLeft.spin(vex::directionType::fwd, rightDriveVoltage, vex::voltageUnits::volt);
  }
}