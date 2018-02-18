/*
 * Bowie Mini Shoreline Servo Calibration
 * --------------------------------------
 * 
 * Calibrate the positions of the servo motors
 * 
 * Erin RobotGrrl for Robot Missions
 * Feb. 10th, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "MiniBowieShoreline.h"

#define ROBOT_ID 3

MiniBowieShoreline bowie;

int calibrate_step = -1;
bool just_changed = true;
bool just_started = true;

void setup() {
  Serial.begin(9600);
  delay(5000);
  bowie = MiniBowieShoreline();
  bowie.setRobotID(ROBOT_ID);
  bowie.begin();
}

void loop() {

  if(just_started) {
    Serial << "------------------------------------" << endl;
    Serial << "Welcome to Bowie's servo calibration" << endl;
    Serial << "------------------------------------" << endl;
    Serial << "This step by step process will guide you through " << endl;
    Serial << "setting up the servos to their proper home positions." << endl;
    Serial << "These steps are important to ensure your Bowie's motors " << endl;
    Serial << "align with the positions in the Bowie codebase!" << endl;
    just_started = false;
    just_changed = true;
    calibrate_step = 0;
  }

  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c == 'n') {
      calibrate_step++;
      just_changed = true;
    }
    if(c == 'p') {
      calibrate_step--;
      just_changed = true;
    }
  }

  if(calibrate_step > 12) {
    Serial << "\n\nCalibration complete! CONGRATS! *\\(^_^)/* " << endl;
    Serial << "Press n to begin again." << endl;
    calibrate_step = -1;
  }

  if(calibrate_step < -1) {
    calibrate_step = -1;
  }

  switch(calibrate_step) {
    case 0:
      if(just_changed) {
        disableAllServos();
        Serial << "\n   n = next step \t p = previous step" << endl;
        just_changed = false;
      }
    break;
    case 1:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- HOPPER TILT UP ---" << endl;
        Serial << "Servo horn should be perpendicular to the ground" << endl;
        Serial << "This position is when it is tilted and emptying" << endl;
        bowie.bowiehopper.tilt.attach(SERVO_HOPPER_PIVOT);
        just_changed = false;
      }
      bowie.bowiehopper.moveHopper(TILT_MIN, 3, 1);
    break;
    case 2:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- HOPPER TILT FLUSH ---" << endl;
        Serial << "Servo horn should be parallel to the enclosure top" << endl;
        Serial << "This position is when it is 'resting' waiting for debris" << endl;
        bowie.bowiehopper.tilt.attach(SERVO_HOPPER_PIVOT);
        just_changed = false;
      }
      bowie.bowiehopper.moveHopper(TILT_MAX, 3, 1);
    break;
    case 3:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- LID OPEN ---" << endl;
        Serial << "Servo horn should be parallel with long edge of servo" << endl;
        Serial << "This position is when the lid is open" << endl;
        bowie.bowiehopper.lid.attach(SERVO_HOPPER_LID);
        just_changed = false;
      }
      bowie.bowiehopper.moveLid(LID_MIN, 3, 1);
    break;
    case 4:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- LID CLOSED ---" << endl;
        Serial << "Servo horn should be 90 degrees to the long edge of servo" << endl;
        Serial << "This position is when the lid is open" << endl;
        bowie.bowiehopper.lid.attach(SERVO_HOPPER_LID);
        just_changed = false;
      }
      bowie.bowiehopper.moveLid(LID_MAX, 3, 1);
    break;
    case 5:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- ARM UP ---" << endl;
        Serial << "Servo horn should be 90 degrees to the long edge of servo" << endl;
        Serial << "This position is when the arm is perpendicular to the base" << endl;
        bowie.bowiearm.arm.attach(SERVO_ARM1);
        bowie.bowiearm.arm2.attach(SERVO_ARM2);
        just_changed = false;
      }
      bowie.bowiearm.moveArm(ARM_MAX, 3, 5);
    break;
    case 6:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- ARM MIDDLE ---" << endl;
        Serial << "Servo horn should be parallel to the long edge of servo" << endl;
        Serial << "This position is when the arm is sticking out to the middle" << endl;
        bowie.bowiearm.arm.attach(SERVO_ARM1);
        bowie.bowiearm.arm2.attach(SERVO_ARM2);
        just_changed = false;
      }
      bowie.bowiearm.moveArm(ARM_HOME, 3, 5);
    break;
    case 7:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- ARM DOWN ---" << endl;
        Serial << "This position is when the arm is down, it should not be pushing against the base" << endl;
        bowie.bowiearm.arm.attach(SERVO_ARM1);
        bowie.bowiearm.arm2.attach(SERVO_ARM2);
        just_changed = false;
      }
      bowie.bowiearm.moveArm(ARM_MIN, 3, 5);
    break;
    case 8:
    if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- ARM MIDDLE ---" << endl;
        Serial << "Servo horn should be parallel to the long edge of servo" << endl;
        Serial << "This position is when the arm is sticking out to the middle" << endl;
        bowie.bowiearm.arm.attach(SERVO_ARM1);
        bowie.bowiearm.arm2.attach(SERVO_ARM2);
        just_changed = false;
      }
      bowie.bowiearm.moveArm(ARM_HOME, 3, 5);
    break;
    case 9:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- END UP ---" << endl;
        Serial << "Servo horn should be 90 deg to the long edge of servo" << endl;
        Serial << "This position is when the end effector is up" << endl;
        bowie.bowiescoop.scoop.attach(SERVO_END_EFFECTOR);
        just_changed = false;
      }
      bowie.bowiescoop.moveEnd(END_MIN, 3, 1);
    break;
    case 10:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- END DOWN ---" << endl;
        Serial << "Servo horn should now be opposite to the UP position" << endl;
        Serial << "This position is when the end effector is down" << endl;
        bowie.bowiescoop.scoop.attach(SERVO_END_EFFECTOR);
        just_changed = false;
      }
      bowie.bowiescoop.moveEnd(END_MAX, 3, 1);
    break;
    case 11:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- END REFLEX ---" << endl;
        Serial << "Servo horn should be -90 deg to the long edge of servo" << endl;
        Serial << "This position is when the end effector is up" << endl;
        bowie.bowiescoop.scoop.attach(SERVO_END_EFFECTOR);
        just_changed = false;
      }
      bowie.bowiescoop.moveEnd(END_REFLEX, 3, 1);
    break;
    case 12:
      if(just_changed) {
        disableAllServos();
        Serial << "\n\n--- END UP ---" << endl;
        Serial << "Servo horn should be 90 deg to the long edge of servo" << endl;
        Serial << "This position is when the end effector is up" << endl;
        bowie.bowiescoop.scoop.attach(SERVO_END_EFFECTOR);
        just_changed = false;
      }
      bowie.bowiescoop.moveEnd(END_MIN, 3, 1);
    break;
  }

}

void disableAllServos() {
  bowie.bowiearm.arm.detach();
  bowie.bowiearm.arm2.detach();
  bowie.bowiescoop.scoop.detach();
  bowie.bowiehopper.tilt.detach();
  bowie.bowiehopper.lid.detach();
}


