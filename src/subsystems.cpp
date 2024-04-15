#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Motor Intake1(14, pros::E_MOTOR_GEARSET_06, false,
                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake2(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);    

pros::ADIDigitalOut wingActuation('D');
pros::ADIDigitalOut PTO('B');
pros::ADIDigitalOut ClimbRelease('A');
pros::ADIDigitalOut Scooper('C');

pros::Motor_Group Intake({Intake1, Intake2});

bool climberLock = false;
bool ptoState = false;
bool WingState = false;

void setIntake(int speed) { Intake.move_voltage(speed * 120); }

void wingControl(bool state) { wingActuation.set_value(state); }

void scooperControl(bool state){ Scooper.set_value(state); }

void ptoControl(bool state) { PTO.set_value(state); }


void intakeControl() {
  setIntake((master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) -
             master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) *
            100);
}

void scooperTeleControl() {
    scooperControl(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
  }

void wingTeleControl(){

wingControl(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
}
void ptoTeleControl(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
    ptoState = !ptoState;
  }

  PTO.set_value(ptoState);
}

void wingTeleControl2(){
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
    WingState = !WingState;
  }

  wingActuation.set_value(WingState);
}

void climbReleaseTeleRelease(){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    climberLock = !climberLock;
  }

  ClimbRelease.set_value(climberLock);
}