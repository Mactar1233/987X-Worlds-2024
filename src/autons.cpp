#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <atomic>
#include <memory>

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;  
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(15, 0, 40);
  chassis.pid_drive_constants_set(14, 0, 30);
  chassis.pid_turn_constants_set(6, 0, 60);
  chassis.pid_swing_constants_set(14.5, 0, 130);

  chassis.pid_turn_exit_condition_set(10_ms, 3_deg, 30_ms, 7_deg, 100_ms, 100_ms);
  chassis.pid_swing_exit_condition_set(10_ms, 3_deg, 30_ms, 7_deg, 100_ms, 100_ms);
  chassis.pid_drive_exit_condition_set(10_ms, 1_in, 30_ms, 3_in, 100_ms, 100_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}


///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .

void safeSafe(){
  chassis.pid_drive_set(13_in,DRIVE_SPEED);
  chassis.pid_wait();
  scooperControl(true);
  chassis.pid_drive_set(-10_in,DRIVE_SPEED);
  chassis.pid_wait();
  scooperControl(false);
 chassis.pid_drive_set(5_in,DRIVE_SPEED);
 chassis.pid_wait();
chassis.pid_drive_set(-5_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(135_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(35_in,DRIVE_SPEED);
chassis.pid_wait();


}

void midSafe(){
chassis.drive_angle_set(135_deg);
chassis.pid_drive_set(15_in,DRIVE_SPEED);
 chassis.pid_wait();
scooperControl(true);
chassis.pid_drive_set(-11_in,DRIVE_SPEED); 
scooperControl(false);
chassis.pid_drive_set(18_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(180_deg,DRIVE_SPEED);
chassis.pid_wait();
setIntake(-100);
pros::delay(1000);
chassis.pid_drive_set(5_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(0_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-15_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(10_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-30_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(22_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-90_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(35_in,DRIVE_SPEED);
chassis.pid_wait();






}

void riskyDef(){
  setIntake(100);
  scooperControl(true);
  wingControl(true);
chassis.pid_turn_set(20_deg,TURN_SPEED);
chassis.pid_wait();
scooperControl(false);
wingControl(false);
chassis.pid_drive_set(53_in,DRIVE_SPEED,true);
pros::delay(500);
chassis.pid_wait();
chassis.pid_drive_set(-55_in,70,true);
chassis.pid_wait();
chassis.pid_turn_set(-50_deg,70,true);
chassis.pid_wait();
chassis.pid_drive_set(15_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-30_in,DRIVE_SPEED);
scooperControl(true);
chassis.pid_wait();
scooperControl(false);
chassis.pid_drive_set(5_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-8_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(90_deg,DRIVE_SPEED,true);
chassis.pid_wait();
chassis.pid_drive_set(35.5_in,DRIVE_SPEED);
setIntake(-100);
chassis.pid_wait();



}


void elimsDef(){

 setIntake(100);
  scooperControl(true);
  wingControl(true);
chassis.pid_turn_set(20_deg,TURN_SPEED);
chassis.pid_wait();
scooperControl(false);
wingControl(false);
chassis.pid_drive_set(50_in,DRIVE_SPEED,true);
pros::delay(500);
chassis.pid_wait();
chassis.pid_turn_set(90_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(30_in,DRIVE_SPEED);
setIntake(-100);
chassis.pid_wait();
chassis.pid_drive_set(-20_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(30_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-53_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-45_deg,TURN_SPEED);
scooperControl(true);
chassis.pid_wait();
chassis.pid_drive_set(-10_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(90_deg,TURN_SPEED);
chassis.pid_wait();
scooperControl(false);
chassis.pid_drive_set(30.5_in,DRIVE_SPEED);
chassis.pid_wait();








}
void sixBall(){
setIntake(100);
chassis.pid_drive_set(10_in,DRIVE_SPEED);
chassis.pid_wait();
setIntake(0);
chassis.pid_drive_set(-35_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-45_deg,TURN_SPEED);
chassis.pid_wait();
scooperControl(true);
chassis.pid_drive_set(-25_in,DRIVE_SPEED);
chassis.pid_wait();
scooperControl(false);
chassis.pid_drive_set(5_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-10_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(-90_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(-20_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(15_in,DRIVE_SPEED);
chassis.pid_wait();
chassis.pid_turn_set(90_deg,70);
chassis.pid_wait();
setIntake(-100);
chassis.pid_drive_set(20_in,DRIVE_SPEED);

chassis.pid_wait();
chassis.pid_drive_set(-13_in,DRIVE_SPEED);
chassis.pid_wait();
setIntake(100);
chassis.pid_turn_set(20_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(50_in,127);
chassis.pid_wait();
setIntake(0);
chassis.pid_turn_set(150_deg,TURN_SPEED);
chassis.pid_wait();
setIntake(-100);
chassis.pid_drive_set(10_in,DRIVE_SPEED);
chassis.pid_wait();
setIntake(100);
chassis.pid_turn_set(45_deg,TURN_SPEED);
chassis.pid_wait();
chassis.pid_drive_set(25_in,127);
chassis.pid_wait();
chassis.pid_turn_set(180_deg,TURN_SPEED);
chassis.pid_wait();
wingControl(true);
setIntake(-100);
chassis.pid_drive_set(40_in,127);
chassis.pid_wait();









}