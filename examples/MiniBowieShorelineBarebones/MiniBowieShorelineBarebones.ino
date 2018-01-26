/*
 * Bowie Mini Shoreline Barebones Example
 * ---------------------------------------
 * 
 * Barebones template for testing the shoreline mission with
 * Bowie (Mini, using Teensy 3.2).
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
  bowie.update(false);
}


