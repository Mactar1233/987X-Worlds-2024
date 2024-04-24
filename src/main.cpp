#include "main.h"
#include "EZ-Template/auton.hpp"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
ez::Drive chassis (
    // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-11 ,-12, -13}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{18, 19, 20}

  // IMU Port
  ,10

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,2.75

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.333333
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

  auto consts = chassis.turnPID.constants_get();

  void manualSetDrive(float Left, float Right){
      chassis.drive_set(Left, Right);
  }

      bool climbAngleLock = false;

  ez::PID ClimbPID{consts.kp, consts.ki, consts.kd, consts.start_i, "ClimbPID"};

  
void climbTask(){
  pros::delay(2000);
  double output = 0.0;
  while (true) {
    if (chassis.drive_imu_get() > 180) {
      ClimbPID.target_set(180);
    }
    else {
      ClimbPID.target_set(0);
    }

    output = ClimbPID.compute(chassis.drive_imu_get());

    if (!climbAngleLock){
      output = 0.0;
    }

    manualSetDrive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + output, master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - output);

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ClimbTask(climbTask);


void initialize() {
  //midOFmidOF
  
  
  //Print our branding over your terminal :D
  ez::ez_template_print();
  ClimbPID.exit_condition_set(80, 50, 300, 150, 500, 500);
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0); // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    Auton("David AUTO\n\nRIGO SAID OFFENSIVE WIN POINT",david),
    Auton("Mid Safe\n\n Descores Ball, Scores Preload, and Touches Bar",riskyOf),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



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
  // . . .
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
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
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
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  
   while (true) {
        /*/
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
      // Enable / Disable PID Tuner
      //  When enabled: 
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X)) 
        chassis.pid_tuner_toggle();
        
      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) 
        autonomous();

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

    chassis.opcontrol_tank(); // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
   /*/

   if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
    climbAngleLock = !climbAngleLock;
   }

    intakeControl();
    wingTeleControl();
    ptoTeleControl();
    climbReleaseTeleRelease();
    scooperTeleControl();
    climbTask();

  

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME)
   }
}
  