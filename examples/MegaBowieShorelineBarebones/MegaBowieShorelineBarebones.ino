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

void control(Msg m);

void setup() {
  Serial.begin(9600);
  bowie = MegaBowieShoreline();
  bowie.begin();
  bowie.setRobotID(ROBOT_ID);
  bowie.set_control_callback(control);
}

void loop() {
  bowie.update(false);
}

void control(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "---RECEIVED ACTION---" << endl;
  Serial << "action: " << m.action << endl;
  Serial << "command: " << m.pck1.cmd << endl;
  Serial << "key: " << m.pck1.key << endl;
  Serial << "val: " << m.pck1.val << endl;
  Serial << "command: " << m.pck2.cmd << endl;
  Serial << "key: " << m.pck2.key << endl;
  Serial << "val: " << m.pck2.val << endl;
  Serial << "delim: " << m.delim << endl;

  Packet packets[2] = { m.pck1, m.pck2 };

}
