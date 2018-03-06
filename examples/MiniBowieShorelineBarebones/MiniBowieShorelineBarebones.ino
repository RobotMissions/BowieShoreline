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

void control(Msg m);

void setup() {
  Serial.begin(9600);
  bowie = MiniBowieShoreline();
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
