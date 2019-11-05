// Include VEX globals

#include "vex.h"

#include <cmath>
#include <ratio>
#include <vector>

using namespace vex;

// Create items 
vex::brain Brain;
vex::competition Competition;
vex::controller Controller;

//List all motors
vex::motor frontRight (vex::PORT10, true); 
vex::motor frontLeft (vex::PORT9, false);
vex::motor backRight (vex::PORT7, true);
vex::motor backLeft (vex::PORT11, false); 

vex::motor arm (vex::PORT4, false);
vex::motor tray (vex::PORT19, true);

vex::motor rightIntakeFlipper (vex::PORT2, true);
vex::motor leftIntakeFlipper (vex::PORT18, false);

// Sensors
vex::pot trayPot (Brain.ThreeWirePort.A);
vex::limit trayLimit (Brain.ThreeWirePort.B);


