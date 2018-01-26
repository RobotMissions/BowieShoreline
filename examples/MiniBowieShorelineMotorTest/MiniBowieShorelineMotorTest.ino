/*
 * Bowie Mini Shoreline Motor Test
 * --------------------------------
 * 
 * Run through quick routines to test the motors on Mini Bowie.
 * 
 * Erin RobotGrrl for RobotMissions
 * Jan. 21st, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "MiniBowieShoreline.h"

#define ROBOT_ID 3

MiniBowieShoreline bowie;

void setup() {
  Serial.begin(9600);
  bowie = MiniBowieShoreline();
  bowie.setRobotID(ROBOT_ID);
  bowie.begin();
}

void loop() {
  //bowie.update(false);
  
  Serial << "--- LID ---" << endl;
  for(int i=0; i<2; i++) {
    bowie.bowiehopper.moveLid(LID_MIN, 3, 1);
    bowie.bowiehopper.moveLid(LID_MIN+400, 3, 1);
  }
  delay(100);

  Serial << "--- ARM ---" << endl;
  bowie.bowiearm.moveArm(ARM_MAX-400, 3, 3);
  bowie.bowiearm.moveArm(ARM_MAX-200, 3, 3);
  delay(100);

  Serial << "--- SCOOP ---" << endl;
  for(int i=0; i<2; i++) {
    bowie.bowiescoop.moveEnd(END_PARALLEL_TOP, 3, 1);
    bowie.bowiescoop.moveEnd(END_PARALLEL_TOP-300, 3, 1);
  }
  delay(100);

  Serial << "--- HOPPER TILT ---" << endl;
  bowie.bowiehopper.moveHopper(TILT_MAX, 3, 1);
  bowie.bowiehopper.moveHopper(TILT_MIN, 3, 1);
  bowie.bowiehopper.moveHopper(TILT_MAX, 3, 1);
  delay(100);
  
}

void slowTest() {
  
  Serial << "--- LID ---" << endl;
  bowie.bowiehopper.moveLid(LID_MAX);
  delay(500);
  bowie.bowiehopper.moveLid(LID_MIN);
  delay(500);

  Serial << "--- SCOOP ---" << endl;
  bowie.bowiescoop.moveEnd(END_PARALLEL_BOTTOM);
  delay(500);
  bowie.bowiescoop.moveEnd(END_MIN);
  delay(500);

  Serial << "--- ARM ---" << endl;
  bowie.bowiearm.moveArm(ARM_HOME);
  delay(500);
  bowie.bowiearm.moveArm(ARM_MAX);
  delay(500);
  bowie.bowiearm.moveArm(ARM_HOME);
  delay(500);

  Serial << "--- HOPPER TILT ---" << endl;
  bowie.bowiehopper.moveHopper(TILT_MAX);
  delay(500);
  bowie.bowiehopper.moveHopper(TILT_MIN);
  delay(500);
  bowie.bowiehopper.moveHopper(TILT_MAX);
  delay(500);
}

