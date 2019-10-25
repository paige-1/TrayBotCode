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
vex::motor frontRight (vex::PORT10, false); 
vex::motor frontLeft (vex::PORT9, false);
vex::motor backRight (vex::PORT8, false);
vex::motor backLeft (vex::PORT7, false); 

vex::motor arm (vex::PORT4, false);
vex::motor tray (vex::PORT20, true);

vex::motor rightIntakeFlipper (vex::PORT5, true);
vex::motor leftIntakeFlipper (vex::PORT6, false);

// Sensors
vex::pot trayPot (Brain.ThreeWirePort.A);
vex::limit trayLimit (Brain.ThreeWirePort.B);


