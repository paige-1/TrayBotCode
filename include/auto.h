#include "vex.h"
#include "config.h"
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
/**
void gyroTurn(double target) {
  target = Gyro.value(vex::analogUnits::range12bit) + target * 10 + target * 41 / 90;
  double error = target - Gyro.value(vex::analogUnits::range12bit);
  double totalError = 0;

  double kp = 0.064;
  double ki = 0.009;

  while (std::abs(error) > 2) {
    vex::task::sleep(6);
    error = target - Gyro.value(vex::analogUnits::range12bit);
    totalError += error;
    if (std::abs(error) > 69) {
      totalError = 0;
    }
    turn2(error * kp + totalError * ki);  
  }
  setHolding();
  stopAll();
}
//DegreeAmount (0 - 360) degrees robot will turn
//veloc (0 - 100) percent of motor power given
void gyroTurn2 (double DegreeAmount, int velocL, int velocR)
{
    //Set speeds of both Drive motors
    frontLeft.setVelocity(velocL,velocityUnits::pct);
    frontRight.setVelocity(velocR,velocityUnits::pct);
    backLeft.setVelocity(velocL,velocityUnits::pct);
    backRight.setVelocity(velocR,velocityUnits::pct);
    
    //Prints the DegreeAmount for debugging puroses to ensure that it is going for the right degree amount
    Controller.Screen.clearScreen();
    Controller.Screen.print(DegreeAmount);
    Controller.Screen.print(Gyro.value(rotationUnits::deg));

    //While loop to do the spin
    if(Gyro.value(rotationUnits::deg) < DegreeAmount){
      while (Gyro.value(rotationUnits::deg) < DegreeAmount)
      {
        /**
        if(Gyro.value(rotationUnits::deg) > DegreeAmount - 25){
          frontLeft.setVelocity(velocL/2,velocityUnits::pct);
          frontRight.setVelocity(velocR/2,velocityUnits::pct);
          backLeft.setVelocity(velocL/2,velocityUnits::pct);
          backRight.setVelocity(velocR/2,velocityUnits::pct);
        }
        
        
        Controller.Screen.clearScreen();
        Controller.Screen.print(DegreeAmount);
        Controller.Screen.print(Gyro.value(rotationUnits::deg));

        frontLeft.spin(directionType::fwd); // Assuming this is the polarity needed for a clockwise turn
        backLeft.spin(directionType::fwd);
        frontRight.spin(directionType::rev);
        backRight.spin(directionType::rev);
        
        this_thread::sleep_for(10);
      }
    } else if (Gyro.value(rotationUnits::deg) > DegreeAmount) {
      while (Gyro.value(rotationUnits::deg) > DegreeAmount)
      {
        /**
        if(Gyro.value(rotationUnits::deg) < DegreeAmount + 25){
          frontLeft.setVelocity(velocL/2,velocityUnits::pct);
          frontRight.setVelocity(velocR/2,velocityUnits::pct);
          backLeft.setVelocity(velocL/2,velocityUnits::pct);
          backRight.setVelocity(velocR/2,velocityUnits::pct);
        }
        

        Controller.Screen.clearScreen();
        Controller.Screen.print(DegreeAmount);
        Controller.Screen.print(Gyro.value(rotationUnits::deg));

        frontLeft.spin(directionType::rev); // Assuming this is the polarity needed for a counterclockwise turn
        backLeft.spin(directionType::rev);
        frontRight.spin(directionType::fwd);
        backRight.spin(directionType::fwd);
        
        this_thread::sleep_for(10);
      }
    }
    //Stop motors after reached degree turn
    stopAll();
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Gyro Turn Finished");
}

//DegreeAmount (0 - 360) degrees robot will turn
//veloc (0 - 100) percent of motor power given
void gyroTurn3 (double DegreeAmount, int velocL, int velocR)
{

    //Prints the DegreeAmount for debugging puroses to ensure that it is going for the right degree amount
    Controller.Screen.clearScreen();
    Controller.Screen.print(DegreeAmount);
    Controller.Screen.print(Gyro.value(rotationUnits::deg));

    //While loop to do the spin
    if(Gyro.value(rotationUnits::deg) < DegreeAmount){
      while (Gyro.value(rotationUnits::deg) < DegreeAmount)
      {
        /**
        double val = Gyro.value(rotationUnits::deg);
        frontLeft.setVelocity(velocL*((DegreeAmount - val)/(DegreeAmount)),velocityUnits::pct);
        backLeft.setVelocity(velocL*((DegreeAmount - val)/(DegreeAmount)),velocityUnits::pct);
        frontRight.setVelocity(velocR*((DegreeAmount - val)/(DegreeAmount)),velocityUnits::pct);
        backRight.setVelocity(velocR*((DegreeAmount - val)/(DegreeAmount)),velocityUnits::pct);
        frontLeft.spin(directionType::fwd); // Assuming this is the polarity needed for a clockwise turn
        backLeft.spin(directionType::fwd);
        frontRight.spin(directionType::rev);
        backRight.spin(directionType::rev);
        
        double val = Gyro.value(rotationUnits::deg);

        frontLeft.spin(vex::directionType::fwd, velocL*((DegreeAmount - val)/(DegreeAmount)), vex::percentUnits::pct);
        backLeft.spin(vex::directionType::fwd, velocL*((DegreeAmount - val)/(DegreeAmount)), vex::percentUnits::pct);
        frontRight.spin(vex::directionType::rev, velocR*((DegreeAmount - val)/(DegreeAmount)), vex::percentUnits::pct);
        backRight.spin(vex::directionType::rev, velocR*((DegreeAmount - val)/(DegreeAmount)), vex::percentUnits::pct);
        
        Controller.Screen.clearScreen();
        Controller.Screen.print(DegreeAmount);
        Controller.Screen.print(Gyro.value(rotationUnits::deg));

        this_thread::sleep_for(10);
      }
    } else if (Gyro.value(rotationUnits::deg) > DegreeAmount) {
      double initial = Gyro.value(rotationUnits::deg);
      while (Gyro.value(rotationUnits::deg) > DegreeAmount)
      {
        double val = Gyro.value(rotationUnits::deg);

        frontLeft.spin(vex::directionType::rev, velocL*((val - DegreeAmount)/(initial - DegreeAmount)), vex::percentUnits::pct);
        backLeft.spin(vex::directionType::rev, velocL*((val - DegreeAmount)/(initial - DegreeAmount)), vex::percentUnits::pct);
        frontRight.spin(vex::directionType::fwd, velocR*((val - DegreeAmount)/(initial - DegreeAmount)), vex::percentUnits::pct);
        backRight.spin(vex::directionType::fwd, velocR*((val - DegreeAmount)/(initial - DegreeAmount)), vex::percentUnits::pct);

        Controller.Screen.clearScreen();
        Controller.Screen.print(DegreeAmount);
        Controller.Screen.print(Gyro.value(rotationUnits::deg));
        
        this_thread::sleep_for(10);
      }
    }
    //Stop motors after reached degree turn
    stopAll();
    
    Controller.Screen.clearScreen();
    Controller.Screen.print("Gyro Turn Finished");
}

void resetGyro() {
  Controller.Screen.clearScreen();
  Controller.Screen.print("Calibrating Gyro...");
  Gyro.startCalibration();
    while(Gyro.isCalibrating());
  Controller.Screen.print("Done!");
}
*/


//ONE SQUARE = 1.79 rev
//RIGHT TURN ~ 1.0rev



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

  basicEncoderDrive(25,4,true);
  
  basicEncoderDrive(25,-4,true);

  rightIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  leftIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);

  frontLeft.spin(vex::directionType::fwd, 50,vex::velocityUnits::pct);
  backLeft.spin(vex::directionType::fwd, 50,vex::velocityUnits::pct);
  frontRight.spin(vex::directionType::rev, 50,vex::velocityUnits::pct);
  backRight.spin(vex::directionType::rev, 50,vex::velocityUnits::pct);
  
  wait(1750);

  stopAll();
  basicEncoderDrive(30,.35,true, 2);

  tray.spin(vex::directionType::fwd,  100, vex::velocityUnits::pct);

  wait(7000);

  tray.spin(vex::directionType::fwd,  0, vex::velocityUnits::pct);

  rightIntakeFlipper.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
  leftIntakeFlipper.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);

  basicEncoderDrive(35,-1,true, 2);

  rightIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  leftIntakeFlipper.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
}


