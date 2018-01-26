/*
 * Bowie Mega Shoreline Barebones Example
 * ---------------------------------------
 * 
 * Barebones template for testing the shoreline mission with
 * Bowie (Mega, using Teensy 3.6).
 * 
 * Erin RobotGrrl for RobotMissions
 * Dec. 30th, 2017
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "MegaBowieShoreline.h"

#define ROBOT_ID 3

MegaBowieShoreline bowie;

void setup() {
  Serial.begin(9600);
  bowie = MegaBowieShoreline();
  bowie.setRobotID(ROBOT_ID);
  bowie.begin();
}

void loop() {
  bowie.update(false);
}


