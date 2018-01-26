#include "MiniBowieShoreline.h"

MiniBowieShoreline::MiniBowieShoreline() {

  
  
}

MiniBowieShoreline *MiniBowieShoreline::bowieInstance;

void MiniBowieShoreline::setRobotID(uint8_t the_robot_id) {
  ROBOT_ID = the_robot_id;
}

void MiniBowieShoreline::begin() {

  // Instance of the class for the callbacks from Promulgate
  bowieInstance = this;
  ROBOT_ID = 3;
  
  performing_large_action = false;

  REMOTE_OP_ENABLED = true;
  PREVENT_OVER_CURRENT = false;
  LOGGING_ENABLED = true;
  TURN_SEQUENCE_MODE = true;

  unlikely_count = 0;
  current_time = 0;
  last_ctrl = 0;
  last_update_periodic = 0;

  arm_pos_over_current = ARM_HOME;
  end_pos_over_current = END_HOME;
  hopper_pos_over_current = TILT_MAX;
  lid_pos_over_current = LID_MIN;
  servos_deactivated_over_current = false;

  // Outputs
  pinMode(BOARD_LED, OUTPUT);
  pinMode(COMM_LED, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  
  // Much thanks to Jeremy Cole @jeremycole for helping track down the
  // servo bug we were encountering in this code!

  // Arm
  bowiearm.begin();
  
  bowiearm.setArm1ServoPin(SERVO_ARM1);
  bowiearm.setArm2ServoPin(SERVO_ARM2);
  bowiearm.setServoInterruptCallback(servoInterrupt);
  bowiearm.initServos();
  

  // Hopper
  bowiehopper.begin();

  bowiehopper.setServoHopperPivotPin(SERVO_HOPPER_PIVOT);
  bowiehopper.setServoHopperLidPin(SERVO_HOPPER_LID);

  bowiehopper.setServoInterruptCallback(servoInterrupt);

  bowiehopper.initServos();


  // Scoop
  bowiescoop.begin();

  bowiescoop.setServoScoopPin(SERVO_END_EFFECTOR);
  bowiescoop.setProbeLPin(SCOOP_PROBE_LEFT);
  bowiescoop.setProbeRPin(SCOOP_PROBE_RIGHT);

  bowiescoop.setServoInterruptCallback(servoInterrupt);

  bowiescoop.initServos();


  // Current Sensors
  servoCurrent.begin();
  motorCurrent.begin();
  
  servoCurrent.setCurrentSensePin(CURRENT_SERVO_SENS);
  servoCurrent.setCurrentSenseName("Servo");
  servoCurrent.initCurrentSensor();
  servoCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_ServosCallback);
  servoCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_ServosCallback);
  servoCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_ServosCallback);
  
  motorCurrent.setCurrentSensePin(CURRENT_MOTOR_SENS);
  motorCurrent.setCurrentSenseName("Motor");
  motorCurrent.initCurrentSensor();
  motorCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_MotorsCallback);
  motorCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_MotorsCallback);
  motorCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_MotorsCallback);


  // Drive
  bowiedrive.begin();

  bowiedrive.setMotorASpeedPin(MOTORA_SPEED);
  bowiedrive.setMotorBSpeedPin(MOTORB_SPEED);
  bowiedrive.setMotorACtrl1Pin(MOTORA_CTRL1);
  bowiedrive.setMotorACtrl2Pin(MOTORA_CTRL2);
  bowiedrive.setMotorBCtrl1Pin(MOTORB_CTRL1);
  bowiedrive.setMotorBCtrl2Pin(MOTORB_CTRL2);

  bowiedrive.initMotors();


  // Lights
  bowielights.begin();

  bowielights.setFrontLeftPin(BRIGHT_LED_FRONT_LEFT);
  bowielights.setFrontRightPin(BRIGHT_LED_FRONT_RIGHT);
  bowielights.setBackLeftPin(BRIGHT_LED_BACK_LEFT);
  bowielights.setBackRightPin(BRIGHT_LED_BACK_RIGHT);

  bowielights.initLeds();


  // Comms
  bowiecomms_xbee.begin();

  bowiecomms_xbee.setRobotID(ROBOT_ID);
  bowiecomms_xbee.setCommLed(COMM_LED);
  bowiecomms_xbee.set_received_action_callback(this->receivedAction_Xbee);
  bowiecomms_xbee.set_comms_timeout_callback(commsTimeout_Xbee);
  bowiecomms_xbee.set_controller_added_callback(controllerAdded_Xbee);
  bowiecomms_xbee.set_controller_removed_callback(controllerRemoved_Xbee);

  bowiecomms_xbee.initComms(XBEE_CONN, 9600);

  current_sensor_periodic = bowiecomms_xbee.msg_none;
  bowiecomms_xbee.addPeriodicMessage(current_sensor_periodic);


  // Arduino Comms
  bowiecomms_arduino.begin();
  
  bowiecomms_arduino.setRobotID(ROBOT_ID);
  bowiecomms_arduino.setCommLed(COMM_LED);
  bowiecomms_arduino.set_received_action_callback(receivedAction_Arduino);
  bowiecomms_arduino.set_comms_timeout_callback(commsTimeout_Arduino);
  bowiecomms_arduino.set_controller_added_callback(controllerAdded_Arduino);
  bowiecomms_arduino.set_controller_removed_callback(controllerRemoved_Arduino);

  bowiecomms_arduino.initComms(PI_CONN, 9600); // on Teensy 3.2, we will use Serial1

  bowiecomms_arduino.addPeriodicMessage(current_sensor_periodic);
  
}


/*

---- States ----

*/

void MiniBowieShoreline::enableRemoteOp() {
  REMOTE_OP_ENABLED = true;
}

void MiniBowieShoreline::disableRemoteOp() {
  REMOTE_OP_ENABLED = false;
}

void MiniBowieShoreline::enableLogging() {
  LOGGING_ENABLED = true;
}

void MiniBowieShoreline::disableLogging() {
  LOGGING_ENABLED = false;
}

void MiniBowieShoreline::enableOverCurrentProtection() {
  PREVENT_OVER_CURRENT = true;
}

void MiniBowieShoreline::disableOverCurrentProtection() {
  PREVENT_OVER_CURRENT = false;
}


/*

---- Bowie Comms Arduino Callbacks ----

*/

void MiniBowieShoreline::receivedAction_Arduino(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "Conn Arduino RX <---- " << m.action << m.pck1.cmd << m.pck1.key << ",";
  Serial << m.pck1.val << "," << m.pck2.cmd << m.pck2.key << ",";
  Serial << m.pck2.val << m.delim << endl;

  if(COMM_DEBUG_MINI) {
    Serial << "---RECEIVED ACTION---" << endl;
    Serial << "action: " << m.action << endl;
    Serial << "command: " << m.pck1.cmd << endl;
    Serial << "key: " << m.pck1.key << endl;
    Serial << "val: " << m.pck1.val << endl;
    Serial << "command: " << m.pck2.cmd << endl;
    Serial << "key: " << m.pck2.key << endl;
    Serial << "val: " << m.pck2.val << endl;
    Serial << "delim: " << m.delim << endl;
  }

  bowieInstance->control(m);
  
}

void MiniBowieShoreline::commsTimeout_Arduino() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void MiniBowieShoreline::controllerAdded_Arduino() {
  // This will never be called for the Arduino
  
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MiniBowieShoreline::controllerRemoved_Arduino() {
  // This will never be called for the Arduino
  
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}


/*

---- Bowie Comms Xbee Callbacks ----

*/

void MiniBowieShoreline::receivedAction_Xbee(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "Conn RX <---- " << m.action << m.pck1.cmd << m.pck1.key << ",";
  Serial << m.pck1.val << "," << m.pck2.cmd << m.pck2.key << ",";
  Serial << m.pck2.val << m.delim << endl;

  if(COMM_DEBUG_MINI) {
    Serial << "---RECEIVED ACTION---" << endl;
    Serial << "action: " << m.action << endl;
    Serial << "command: " << m.pck1.cmd << endl;
    Serial << "key: " << m.pck1.key << endl;
    Serial << "val: " << m.pck1.val << endl;
    Serial << "command: " << m.pck2.cmd << endl;
    Serial << "key: " << m.pck2.key << endl;
    Serial << "val: " << m.pck2.val << endl;
    Serial << "delim: " << m.delim << endl;
  }

  bowieInstance->control(m);
  
}

void MiniBowieShoreline::commsTimeout_Xbee() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void MiniBowieShoreline::controllerAdded_Xbee() {
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MiniBowieShoreline::controllerRemoved_Xbee() {
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}


/*

---- Control ----

*/

void MiniBowieShoreline::update(bool force_no_sleep) {

  bowieInstance = this;
  current_time = millis();

  // comms
  bowiecomms_xbee.updateComms();
  bowiecomms_arduino.updateComms();

  // specific things to do if remote operation is enabled
  if(REMOTE_OP_ENABLED) {

    if(!force_no_sleep) {
      // go to sleep if we haven't heard in a while
      if(millis()-last_ctrl >= REMOTE_OP_SLEEP) {
        digitalWrite(COMM_LED, LOW);
        bowiedrive.motor_setDir(0, MOTOR_DIR_REV);
        bowiedrive.motor_setSpeed(0, 0);
        bowiedrive.motor_setDir(1, MOTOR_DIR_REV);
        bowiedrive.motor_setSpeed(1, 0);
        if(!servos_deactivated_over_current) {
          bowielights.dimLights();
          bowiearm.parkArm();
          bowiescoop.parkEnd();
          bowiehopper.parkHopper();
          bowiehopper.parkLid();
        }
      }
    }
    
  }

  // sensors
  servoCurrent.updateCurrentSensor();
  motorCurrent.updateCurrentSensor();

  // periodic
  if(current_time-last_update_periodic >= 1000) {
    updatePeriodicMessages();
    last_update_periodic = current_time;
  }

}

void MiniBowieShoreline::control(Msg m) {

  if(BOT_DEBUG_MINI) Serial << "Control! WOOOOT" << endl;
  
  last_ctrl = millis();

  Packet packets[2] = { m.pck1, m.pck2 };

  if(COMM_DEBUG_MINI) {
    Serial << "*c1 cmd: " << packets[0].cmd << " key: " << packets[0].key << " val: " << packets[0].val << endl;
    Serial << "*c2 cmd: " << packets[1].cmd << " key: " << packets[1].key << " val: " << packets[1].val << endl;
  }

  // we've seen this happen *sometimes*, and it is highly unlikely that this would be an
  // intentional command. let's make sure they mean this at least 2 times before listening
  if(m.pck1.val == 255 && m.pck2.val == 255 && m.pck1.cmd == 'L' && m.pck2.cmd == 'R') {
    unlikely_count++;
    if(unlikely_count <= 2) return;
  } else {
    unlikely_count = 0;
  }

  if(m.action == '#') {

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'P') { // red button
        if(packets[i].val == 1) { // sends drive joystick cmds on operator side
        }
      }

      if(packets[i].cmd == 'Y') { // yellow button
        if(packets[i].val == 1) { // sends arm joystick cmds on operator side
        }
      }

      if(packets[i].cmd == 'G') { // green button - dump
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection

          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          bool was_lid_parked = bowiehopper.getLidParked();
          if(was_lid_parked) bowiehopper.unparkLid();
          bowiehopper.moveLid(LID_MIN, 5, 1); // open lid
          delay(50);
          bowiescoop.moveEnd(END_MIN, 3, 2); // dump scoop
          delay(300);
          bowiescoop.moveEnd(END_PARALLEL_TOP, 5, 1); // back into position
          bowiehopper.moveLid(LID_MAX, 5, 1); // close lid
          if(was_lid_parked) bowiehopper.parkLid();
          performing_large_action = false;
        }
      }

      if(packets[i].cmd == 'W') { // white button - scoop slow
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection
          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          scoopSequence(1000);
          performing_large_action = false;
        }
      }

      if(packets[i].cmd == 'B') { // blue button - scoop fast
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection
          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          scoopSequence(0);
          performing_large_action = false;
        }
      }

      if(packets[i].cmd == 'N') { // black button - empty
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection
          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          bool was_hopper_parked = bowiehopper.getHopperParked();
          if(was_hopper_parked) bowiehopper.unparkHopper();
          bool was_lid_parked = bowiehopper.getLidParked();
          if(was_lid_parked) bowiehopper.unparkLid();
          deposit();
          if(was_hopper_parked) bowiehopper.parkHopper();
          if(was_lid_parked) bowiehopper.parkLid();
          performing_large_action = false;
        }
      }

    }
  } // -- end of '#' action specifier

  if(m.action == '@') {

    if(packets[0].cmd == 'L' && packets[0].key == 1 && packets[1].cmd == 'R' && packets[1].key == 0) {
      // turning right
      // for mini bowie, we will just keep all the lights on
      bowielights.setLight(99, MAX_BRIGHTNESS);
      
      if(TURN_SEQUENCE_MODE) {
        bowiedrive.turnSequence(false);
      } else {
        bowiedrive.motor_setDir(1, MOTOR_DIR_FWD);
        bowiedrive.motor_setSpeed(1, 255);
        bowiedrive.motor_setDir(0, MOTOR_DIR_REV);
        bowiedrive.motor_setSpeed(0, 255);
      }
      return; // we don't want the default stuff below when turning
    } else if(packets[0].cmd == 'L' && packets[0].key == 0 && packets[1].cmd == 'R' && packets[1].key == 1) {
      // turning left
      // for mini bowie, we will just keep all the lights on
      bowielights.setLight(99, MAX_BRIGHTNESS);

      if(TURN_SEQUENCE_MODE) {
        bowiedrive.turnSequence(true);
      } else {
        bowiedrive.motor_setDir(0, MOTOR_DIR_FWD);
        bowiedrive.motor_setSpeed(0, 255);
        bowiedrive.motor_setDir(1, MOTOR_DIR_REV);
        bowiedrive.motor_setSpeed(1, 255);
      }
      return; // we don't want the default stuff below when turning
    }
    
    // stop the motors when zeroed
    if(packets[0].cmd == 'L' && packets[0].key == 0 && packets[0].val == 0 && packets[1].cmd == 'R' && packets[1].key == 0 && packets[1].val == 0) {
      // TODO (future) ramp this down from last speed
      bowiedrive.motor_setDir(0, MOTOR_DIR_FWD);
      bowiedrive.motor_setSpeed(0, 0);
      bowiedrive.motor_setDir(1, MOTOR_DIR_FWD);
      bowiedrive.motor_setSpeed(1, 0);
    }

    // if it reaches here, then we know we can reset this flag
    bowiedrive.resetTurnSequence();

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          bowielights.setLight(0, packets[i].val);
          //leftBork();
          bowiedrive.motor_setDir(0, MOTOR_DIR_FWD);
          bowiedrive.motor_setSpeed(0, packets[i].val);
        } else if(packets[i].key == 0) { // bwd
          bowielights.setLight(2, packets[i].val);
          //leftBork();
          bowiedrive.motor_setDir(0, MOTOR_DIR_REV);
          bowiedrive.motor_setSpeed(0, packets[i].val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          bowielights.setLight(1, packets[i].val);
          //leftBork();
          bowiedrive.motor_setDir(1, MOTOR_DIR_FWD);
          bowiedrive.motor_setSpeed(1, packets[i].val);
        } else if(packets[i].key == 0) { // bwd
          bowielights.setLight(3, packets[i].val);
          //leftBork();
          bowiedrive.motor_setDir(1, MOTOR_DIR_REV);
          bowiedrive.motor_setSpeed(1, packets[i].val);
        }
      }
      
      if(packets[i].cmd == 'S') { // arm (data from 0-100)
        
        if(servos_deactivated_over_current) return; // over current protection

        int temp_pos = bowiearm.getArmPos();
        int new_pos = temp_pos;
        int the_increment = (int)map(packets[i].val, 0, 100, 1, 70);

        if(packets[i].key == 0) {
          new_pos -= the_increment;
        } else if(packets[i].key == 1) {
          new_pos += the_increment;
        }

        if(new_pos < ARM_HOME) {
          moveArmAndEnd(new_pos, 3, 1, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-100, END_HOME-300); // END_PARALLEL_BOTTOM-700
        } else if(new_pos > ARM_HOME) {
          moveArmAndEnd(new_pos, 3, 1, ARM_HOME, ARM_MAX, END_HOME-300, END_PARALLEL_TOP-200);
        }

      }
      
    }
  } // -- end of '@' action specifier

}

void MiniBowieShoreline::servoInterrupt(int key, int val) {
  bowieInstance->processServoInterrupt(key, val);
}

void MiniBowieShoreline::processServoInterrupt(int key, int val) {

  // We are forcing a refresh here, which will be able to send a heartbeat
  // back to the controller, and the robot will read the latest command.
  // There is a flag in control (performing_large_action) that will prevent
  // other large actions from taking place if there's one running currently.
  if(current_time-bowiecomms_xbee.getLastRXTime() >= 500) {
    bowiecomms_xbee.updateComms();
  }
  if(current_time-bowiecomms_arduino.getLastRXTime() >= 500) {
    bowiecomms_arduino.updateComms();
  }

  switch(key) {
    case SERVO_ARM_KEY:
    break;
    case SERVO_END_KEY:
      Serial << "End" << endl;
    break;
    case SERVO_HOPPER_KEY:
    break;
    case SERVO_LID_KEY:
      Serial << "Lid" << endl;
    break;
    case SERVO_EXTRA_KEY:
    break;
    case SERVO_ARM_AND_END_KEY:
    break;
  }

}

void MiniBowieShoreline::updatePeriodicMessages() {
  // Updating our periodic messages

  current_sensor_periodic.priority = 10;
  current_sensor_periodic.action = '#';
  current_sensor_periodic.pck1.cmd = 'E';
  current_sensor_periodic.pck1.key = 1;
  current_sensor_periodic.pck1.val = motorCurrent.getCurrentSensorReading();
  current_sensor_periodic.pck2.cmd = 'F';
  current_sensor_periodic.pck2.key = 1;
  current_sensor_periodic.pck2.val = servoCurrent.getCurrentSensorReading();
  current_sensor_periodic.delim = '!';

  bowiecomms_xbee.updatePeriodicMessage(current_sensor_periodic);
  bowiecomms_arduino.updatePeriodicMessage(current_sensor_periodic);
  
}


/*

---- Current Sensors ----

*/

void MiniBowieShoreline::waitingToCoolDown_ServosCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
  bowieInstance->waitingToCoolDown_Servos(first);
}

void MiniBowieShoreline::reactivateAfterCoolDown_ServosCallback() {
  // you might want to re-attach the servos here
  // and send them back to a position
  // (or re-activate the dc motors)
  bowieInstance->reactivateAfterCoolDown_Servos();
}

void MiniBowieShoreline::overCurrentThreshold_ServosCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
  bowieInstance->overCurrentThreshold_Servos(first);
}

void MiniBowieShoreline::waitingToCoolDown_MotorsCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
  bowieInstance->waitingToCoolDown_Motors(first);
}

void MiniBowieShoreline::reactivateAfterCoolDown_MotorsCallback() {
  // you might want to re-attach the servos here
  // and send them back to a position
  // (or re-activate the dc motors)
  bowieInstance->reactivateAfterCoolDown_Motors();
}

void MiniBowieShoreline::overCurrentThreshold_MotorsCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
  bowieInstance->overCurrentThreshold_Motors(first);
}


void MiniBowieShoreline::waitingToCoolDown_Servos(bool first) {

  if(!PREVENT_OVER_CURRENT) return;

  servos_deactivated_over_current = true;

  if(first) {

    buzz(NOTE_E3, 200);
    for(int i=0; i<3; i++) {
      buzz(NOTE_B1, 100);
      delay(100);
      buzz(NOTE_B0, 100);
      delay(100);
    }

    arm_pos_over_current = bowiearm.getArmPos();
    end_pos_over_current = bowiescoop.getEndPos();
    hopper_pos_over_current = bowiehopper.getHopperPos();
    lid_pos_over_current = bowiehopper.getLidPos();

    bowiearm.parkArm();
    bowiescoop.parkEnd();
    bowiehopper.parkHopper();
    bowiehopper.parkLid();

    bowielights.dimLights();

  }

}

void MiniBowieShoreline::reactivateAfterCoolDown_Servos() {

  if(!PREVENT_OVER_CURRENT) return;

  servos_deactivated_over_current = false;

  for(int i=0; i<2; i++) {
    buzz(NOTE_E7, 100);
    delay(100);
  }

  bowiearm.unparkArm();
  bowiescoop.unparkEnd();
  bowiehopper.unparkHopper();
  bowiehopper.unparkLid();

  bowiearm.moveArm(arm_pos_over_current);
  bowiescoop.moveEnd(end_pos_over_current);
  bowiehopper.moveHopper(hopper_pos_over_current);
  bowiehopper.moveLid(lid_pos_over_current);

  bowielights.turnOnLights();

}

void MiniBowieShoreline::overCurrentThreshold_Servos(bool first) {
  if(!PREVENT_OVER_CURRENT) return;

  servos_deactivated_over_current = true;

  for(int i=0; i<2; i++) {
    buzz(NOTE_B1, 100);
    delay(100);
  }

}

void MiniBowieShoreline::waitingToCoolDown_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}

void MiniBowieShoreline::reactivateAfterCoolDown_Motors() {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}

void MiniBowieShoreline::overCurrentThreshold_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}


/*

---- Movements ----

*/

void MiniBowieShoreline::scoopSequence(int frame_delay) {

  // 1. move arm down (not all the way)
  // 2. move scoop slightly above parallel
  // 3. drive forward
  // 4. move scoop to parallel a bit
  // 5. drive forward
  // 6. tilt up quickly
  // 7. move down slightly slowly
  // 8. tilt up quickly

  //bool DEBUGGING_ANIMATION = false;
  //bool was_hopper_parked = bowiehopper.getHopperParked();
  //bool was_lid_parked = bowiehopper.getLidParked();
  //if(was_hopper_parked) bowie.unparkHopper();
  //if(was_lid_parked) bowie.unparkLid();

  // 2. 
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM, 5, 1);
  delay(frame_delay);

  // 1.
  bowiearm.moveArm(ARM_MIN+100, 4, 4);
  delay(frame_delay);


  // 2. 
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM+300, 5, 1);
  delay(frame_delay);


  // 3.
  // drive forward a bit
  // Serial << "Going to MOTORS FWD 255...";
  // bowie.rampSpeed(true, 100, 255, 20, 10);
  // bowie.goSpeed(true, 255, 500);
  // // stop motors!
  // Serial << "Going to MOTORS FWD 0...";
  // bowie.rampSpeed(true, 255, 100, 10, 5);
  // bowie.goSpeed(true, 0, 0);
  // if(DEBUGGING_ANIMATION) delay(3000);

  
  // 4.
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM, 5, 1);
  delay(frame_delay);


  // 5.
  // drive forward a bit
  if(BOT_DEBUG_MINI) Serial << "Going to MOTORS FWD 255...";
  bowiedrive.rampSpeed(true, 100, 255, 20, 10);
  bowiedrive.goSpeed(true, 255, 250);
  // stop motors!
  if(BOT_DEBUG_MINI) Serial << "Going to MOTORS FWD 0...";
  bowiedrive.rampSpeed(true, 255, 100, 10, 5);
  bowiedrive.goSpeed(true, 0, 0);

  delay(frame_delay);

  // 6.
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM-700, 5, 1);
  delay(frame_delay);

  // 7.
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM-400, 5, 1);
  delay(frame_delay);

  // 8.
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM-700, 5, 1);
  delay(frame_delay);

  // -----------

  // move arm up a bit
  bowiearm.moveArm(ARM_MIN+100, 3, 1);

  delay(frame_delay);

  // tilt the scoop upwards to avoid losing the items
  if(BOT_DEBUG_MINI) Serial << "Going to END_PARALLEL_BOTTOM-700...";
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM-700, 3, 1); // todo - check this again, its less than end min
  delay(20);

  delay(frame_delay);

  // lift arm with scoop parallel to ground
  // this has to be adjusted if going faster
  if(BOT_DEBUG_MINI) Serial << "Going to ARM_HOME...";
  moveArmAndEnd(ARM_HOME, 5, 2, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-700, END_HOME-550);//END_PARALLEL_BOTTOM-400);

  delay(150);

  delay(frame_delay);
  
  // lift arm with scoop parallel to ground
  if(BOT_DEBUG_MINI) Serial << "Going to ARM_MAX...";
  moveArmAndEnd(ARM_MAX, 3, 3, ARM_HOME, ARM_MAX, END_HOME-550, END_PARALLEL_TOP-400);//END_PARALLEL_BOTTOM-400, END_PARALLEL_TOP-100);
  
  delay(frame_delay);

  // open lid
  if(BOT_DEBUG_MINI) Serial << "Going to LID_MIN...";
  bowiehopper.moveLid(LID_MIN, 5, 2);
  
  delay(frame_delay);

  // dump scoop
  if(BOT_DEBUG_MINI) Serial << "Going to END_MIN...";
  bowiescoop.moveEnd(END_MIN, 5, 3);
  
  delay(frame_delay);

  // bring scoop back to position
  if(BOT_DEBUG_MINI) Serial << "Going to END_PARALLEL_TOP-200...";
  bowiescoop.moveEnd(END_PARALLEL_TOP-200, 5, 3);
  
  delay(frame_delay);

  // close lid
  if(BOT_DEBUG_MINI) Serial << "Going to LID_MAX...";
  bowiehopper.moveLid(LID_MAX, 5, 2);
  
  delay(frame_delay);

  // 
  // drive backward a bit
  if(BOT_DEBUG_MINI) Serial << "Going to MOTORS FWD 255...";
  bowiedrive.rampSpeed(false, 100, 255, 20, 10);
  bowiedrive.goSpeed(false, 255, 100);
  
  // stop motors!
  if(BOT_DEBUG_MINI) Serial << "Going to MOTORS FWD 0...";
  bowiedrive.rampSpeed(false, 255, 100, 10, 5);
  bowiedrive.goSpeed(true, 0, 5);
  
  delay(frame_delay);

  // park arm
  // Serial << "Parking arm...";
  // bowie.parkArm();
  // bowie.parkEnd();  
  // if(DEBUGGING_ANIMATION) delay(3000);
  // if(was_hopper_parked) bowie.parkHopper();
  // if(was_lid_parked) bowie.parkLid();

}

void MiniBowieShoreline::deposit() {

  bool was_arm_parked = bowiearm.getArmParked();

  // if the arm is up let's move it to home
  if(was_arm_parked) {
    bowiescoop.unparkEnd();
    bowiearm.unparkArm();
  }
  int temp_arm = bowiearm.getArmPos();
  int temp_end = bowiescoop.getEndPos();
  if(temp_arm > 2000) {
    bowiescoop.moveEnd(END_HOME);
    bowiearm.moveArm(ARM_HOME);
  }

  bowiehopper.unparkHopper();
  bowiehopper.unparkLid();

  // open lid
  if(BOT_DEBUG_MINI) Serial << "Going to LID_MIN";
  bowiehopper.moveLid(LID_MIN, 3, 2);
  delay(10);

  if(BOT_DEBUG_MINI) Serial << "Going to TILT_MIN";
  bowiehopper.moveHopper(TILT_MIN, 2, 2);
  delay(100);

  delay(1000);

  if(BOT_DEBUG_MINI) Serial << "Shaking the hopper TILT_MIN+300 to TILT_MIN";
  for(int i=0; i<3; i++) {
    bowiehopper.moveHopper(TILT_MIN+300, 1, 2);
    delay(50);
    bowiehopper.moveHopper(TILT_MIN, 1, 2);
    delay(50);
  }
  
  delay(1000);

  if(BOT_DEBUG_MINI) Serial << "Going to TILT_MAX";
  bowiehopper.moveHopper(TILT_MAX, 2, 2);
  delay(100);

  if(BOT_DEBUG_MINI) Serial << "Positioning closed just in case TILT_MAX-200 to TILT_MAX";
  for(int i=0; i<2; i++) {
    bowiehopper.moveHopper(TILT_MAX-200, 1, 2);
    delay(50);
    bowiehopper.moveHopper(TILT_MAX, 1, 2);
    delay(50);
  }
  
  bowiehopper.parkHopper();

  // close lid
  if(BOT_DEBUG_MINI) Serial << "Going to LID_MAX";
  bowiehopper.moveLid(LID_MAX, 3, 2);
  delay(10);

  bowiehopper.parkLid();

  // moving the arm back
  if(temp_arm > 2000) {
    bowiescoop.moveEnd(temp_end);
    bowiearm.moveArm(temp_arm);
  }

  if(was_arm_parked) {
    bowiescoop.parkEnd();
    bowiearm.parkArm();
  }

}

void MiniBowieShoreline::moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax) {

  bool did_move_hopper = false;
  int hopper_original_pos = bowiehopper.getHopperPos();
  int endPos = 0;

  int arm_position = 0;
  int end_position = 0;
  
  if(bowiearm.getArmPos() == ARM_MIN && bowiescoop.getEndPos() < END_MAX) { // check if the arm is down and if the end is going past being down
    Serial << "!!! Cannot move end-effector here when arm down" << endl;
    return;
  }
  
  bowiearm.unparkArm();
  bowiescoop.unparkEnd();

  if(bowiehopper.getHopperPos() == TILT_MIN) { // check if the hopper is up
    bowiehopper.moveHopper(TILT_MAX); // move it flush if not
    did_move_hopper = true;
  }

  if(bowiearm.getArmPos() > armPos) { // headed towards ARM_MIN
    for(int i=bowiearm.getArmPos(); i>armPos; i-=step) {
      bowiearm.arm.writeMicroseconds(i);
      bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      bowiescoop.scoop.writeMicroseconds(endPos);
      arm_position = i;
      end_position = endPos;
      delay(del);
      servoInterrupt(SERVO_ARM_AND_END_KEY, i);
      //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(bowiearm.getArmPos() <= armPos) { // headed towards ARM_MAX
    for(int i=bowiearm.getArmPos(); i<armPos; i+=step) {
      bowiearm.arm.writeMicroseconds(i);
      bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      bowiescoop.scoop.writeMicroseconds(endPos);
      arm_position = i;
      end_position = endPos;
      delay(del); 
      servoInterrupt(SERVO_ARM_AND_END_KEY, i);
      //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  bowiearm.arm.writeMicroseconds(armPos);
  bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - armPos + SERVO_MIN_US);
  endPos = clawParallelValBounds(armPos, armMin, armMax, endMin, endMax);
  bowiescoop.scoop.writeMicroseconds(endPos);
  arm_position = armPos;
  end_position = endPos;
  delay(del);
  servoInterrupt(SERVO_ARM_AND_END_KEY, armPos);
  //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_hopper) { // move hopper back to original position
    bowiehopper.moveHopper(hopper_original_pos);
  }

}

// ---------
// This code is from Micah Black, during Random Hacks of Kindness in Ottawa 2017!
// Check out his store: a2delectronics.ca

//get a parallel claw value
int MiniBowieShoreline::clawParallelVal(int arm_Val) {
  return 1;
  // TODO
  //return (int)constrain( map(arm_Val, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP) , END_PARALLEL_BOTTOM, END_PARALLEL_TOP);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.

//get a parallel claw value
int MiniBowieShoreline::clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax){
  return 1;
  // TODO
  //return (int)constrain( map(arm_Val, armMin, armMax, endMin, endMax) , endMin, endMax);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.



/*

---- Specific ----

*/

void MiniBowieShoreline::buzz(long frequency, long length) {
  long delayValue = 1000000 / frequency / 2;
  long numCycles = frequency * length / 1000;
  for (long i = 0; i < numCycles; i++) { 
    digitalWrite(SPEAKER, HIGH); 
    delayMicroseconds(delayValue); 
    digitalWrite(SPEAKER, LOW); 
    delayMicroseconds(delayValue); 
  }
}

