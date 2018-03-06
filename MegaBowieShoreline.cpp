#include "MegaBowieShoreline.h"

MegaBowieShoreline::MegaBowieShoreline() {

  
  
}

MegaBowieShoreline *MegaBowieShoreline::bowieInstance;

void MegaBowieShoreline::setRobotID(uint8_t the_robot_id) {
  ROBOT_ID = the_robot_id;
}

void MegaBowieShoreline::begin() {

  Serial << "Bowie is getting started......." << endl;

  // Instance of the class for the callbacks from Promulgate
  bowieInstance = this;
  ROBOT_ID = 3;
  
  performing_large_action = false;

  REMOTE_OP_ENABLED = true;
  PREVENT_OVER_CURRENT = false;
  LOGGING_ENABLED = true;
  TURN_SEQUENCE_MODE = true;
  DEFAULT_ACTIONS = true;

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


  // Logger
  bowielogger.begin();

  bowielogger.initTime();

  bowielogger.setLoggingLed(13);
  bowielogger.initLogging();


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

  bowiecomms_arduino.initComms(ARDUINO_CONN, 9600);

  bowiecomms_arduino.addPeriodicMessage(current_sensor_periodic);
  
  Serial << "Bowie is ready" << endl;

}

void MegaBowieShoreline::set_control_callback( void (*controlCallback)(Msg m) ) {
  _controlCallback = controlCallback;
}


/*

---- States ----

*/

void MegaBowieShoreline::enableRemoteOp() {
  REMOTE_OP_ENABLED = true;
}

void MegaBowieShoreline::disableRemoteOp() {
  REMOTE_OP_ENABLED = false;
}

void MegaBowieShoreline::enableLogging() {
  LOGGING_ENABLED = true;
}

void MegaBowieShoreline::disableLogging() {
  LOGGING_ENABLED = false;
}

void MegaBowieShoreline::enableOverCurrentProtection() {
  PREVENT_OVER_CURRENT = true;
}

void MegaBowieShoreline::disableOverCurrentProtection() {
  PREVENT_OVER_CURRENT = false;
}

void MegaBowieShoreline::enableDefaultActions() {
  DEFAULT_ACTIONS = true;
}

void MegaBowieShoreline::disableDefaultActions() {
  DEFAULT_ACTIONS = false;
}


/*

---- Bowie Comms Arduino Callbacks ----

*/

void MegaBowieShoreline::receivedAction_Arduino(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "Conn Arduino RX <---- " << m.action << m.pck1.cmd << m.pck1.key << ",";
  Serial << m.pck1.val << "," << m.pck2.cmd << m.pck2.key << ",";
  Serial << m.pck2.val << m.delim << endl;

  if(COMM_DEBUG_MEGA) {
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

void MegaBowieShoreline::commsTimeout_Arduino() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void MegaBowieShoreline::controllerAdded_Arduino() {
  // This will never be called for the Arduino
  
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MegaBowieShoreline::controllerRemoved_Arduino() {
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

void MegaBowieShoreline::receivedAction_Xbee(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "Conn RX <---- " << m.action << m.pck1.cmd << m.pck1.key << ",";
  Serial << m.pck1.val << "," << m.pck2.cmd << m.pck2.key << ",";
  Serial << m.pck2.val << m.delim << endl;

  if(COMM_DEBUG_MEGA) {
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

void MegaBowieShoreline::commsTimeout_Xbee() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void MegaBowieShoreline::controllerAdded_Xbee() {
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MegaBowieShoreline::controllerRemoved_Xbee() {
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}


/*

---- Control ----

*/

void MegaBowieShoreline::update(bool force_no_sleep) {

  bowieInstance = this;
  current_time = millis();

  // comms
  bowiecomms_xbee.updateComms();
  /*
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
          // uncomment the below two lines if you want to have the robot
          // save power when idle by always moving the arm back into its
          // parked position
          //bowiearm.parkArm();
          //bowiescoop.parkEnd();
          bowiehopper.parkHopper();
          bowiehopper.parkLid();
        }
      }
    }
    
  }

  // sensors
  // TODO - this is disabled temporarily as it interferes with
  // the communication latency (for some reason...)
  // it causes sproadic 5000ms delays
  //servoCurrent.updateCurrentSensor();
  //motorCurrent.updateCurrentSensor();

  // log
  updateLogSensorData();
  bowielogger.updateLogging();

  // periodic
  if(current_time-last_update_periodic >= 1000) {
    Serial << "~";
    updatePeriodicMessages();
    last_update_periodic = current_time;
  }
  */

  // arm control
  armOperatorControl();

}

void MegaBowieShoreline::received_action(Msg m) {
  bowieInstance->control(m);
}

void MegaBowieShoreline::control(Msg m) {

  last_ctrl = millis();

  Packet packets[2] = { m.pck1, m.pck2 };

  if(COMM_DEBUG_MEGA) {
    Serial << "*c1 cmd: " << packets[0].cmd << " key: " << packets[0].key << " val: " << packets[0].val << endl;
    Serial << "*c2 cmd: " << packets[1].cmd << " key: " << packets[1].key << " val: " << packets[1].val << endl;
  }


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
      bowielights.setLight(99, MIN_BRIGHTNESS);
    }

    // if it reaches here, then we know we can reset this flag
    bowiedrive.resetTurnSequence();

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          if(packets[i].val > MIN_BRIGHTNESS) {
            bowielights.setLight(0, packets[i].val);  
          } else {
            bowielights.setLight(0, MIN_BRIGHTNESS);
          }
          bowielights.setLight(2, MIN_BRIGHTNESS);
          //leftBork();
          bowiedrive.motor_setDir(0, MOTOR_DIR_FWD);
          bowiedrive.motor_setSpeed(0, packets[i].val);
        } else if(packets[i].key == 0) { // bwd
          bowielights.setLight(0, MIN_BRIGHTNESS);
          if(packets[i].val > MIN_BRIGHTNESS) {
            bowielights.setLight(2, packets[i].val);  
          } else {
            bowielights.setLight(2, MIN_BRIGHTNESS);
          }
          //leftBork();
          bowiedrive.motor_setDir(0, MOTOR_DIR_REV);
          bowiedrive.motor_setSpeed(0, packets[i].val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          if(packets[i].val > MIN_BRIGHTNESS) {
            bowielights.setLight(1, packets[i].val);  
          } else {
            bowielights.setLight(1, MIN_BRIGHTNESS);
          }
          bowielights.setLight(1, packets[i].val);
          bowielights.setLight(3, MIN_BRIGHTNESS);
          //leftBork();
          bowiedrive.motor_setDir(1, MOTOR_DIR_FWD);
          bowiedrive.motor_setSpeed(1, packets[i].val);
        } else if(packets[i].key == 0) { // bwd
          if(packets[i].val > MIN_BRIGHTNESS) {
            bowielights.setLight(3, packets[i].val);  
          } else {
            bowielights.setLight(3, MIN_BRIGHTNESS);
          }
          bowielights.setLight(1, MIN_BRIGHTNESS);
          //leftBork();
          bowiedrive.motor_setDir(1, MOTOR_DIR_REV);
          bowiedrive.motor_setSpeed(1, packets[i].val);
        }
      }
      
      if(packets[i].cmd == 'S') { // arm (data from 0-100)

        if(servos_deactivated_over_current) return; // over current protection

        uint8_t the_increment = 250;
        int current_pos = bowiearm.getArmPos();
        int new_pos = current_pos;

        // update the direction the arm is heading
        prev_target_heading = new_heading;
        if(packets[i].val > 0) {
          if(packets[i].key == 0) {
            new_pos -= the_increment;
            new_heading = false; // down
            last_arm_control_cmd = current_time;
            updating_arm = true;
          } else if(packets[i].key == 1) {
            new_pos += the_increment;
            new_heading = true; // up
            last_arm_control_cmd = current_time;
            updating_arm = true;
          }
        } else {
          updating_arm = false;
        }

        // check if the position has met the target
        if(target_heading == false && current_pos >= (target_arm_pos-20)) {
          update_target = true;
        } else if(target_heading == true && current_pos <= (target_arm_pos+20)) {
          update_target = true;
        } else {
          update_target = false;
        }

        // check if the heading changed
        if(new_heading != prev_target_heading) {
          update_target = true;
        }

        // set the new values if updating the target
        if(update_target) {
          if(BOT_DEBUG_MEGA) Serial << "Updating arm target" << endl;
          target_arm_pos = new_pos;
          target_heading = new_heading;
        } else {
          if(BOT_DEBUG_MEGA) Serial << "Not updating arm target" << endl;
        }

      }
      
    }
  } // -- end of '@' action specifier

  // they want to handle the control messages on the sketch side
  // (most likely for custom behaviours)
  if(!DEFAULT_ACTIONS) {
    _controlCallback(m);
    return;
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

      if(packets[i].cmd == 'G') { // green button - empty
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection

          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          bool was_lid_parked = bowiehopper.getLidParked();
          if(was_lid_parked) bowiehopper.unparkLid();

          emptyScoop();

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

      if(packets[i].cmd == 'N') { // black button - dump
        if(packets[i].val == 1) {

          if(servos_deactivated_over_current) return; // over current protection
          if(performing_large_action) return; // don't do this action if we're already doing one

          performing_large_action = true;
          bool was_hopper_parked = bowiehopper.getHopperParked();
          bool was_lid_parked = bowiehopper.getLidParked();

          if(was_hopper_parked) bowiehopper.unparkHopper();
          if(was_lid_parked) bowiehopper.unparkLid();
          
          deposit();
          
          if(was_hopper_parked) bowiehopper.parkHopper();
          if(was_lid_parked) bowiehopper.parkLid();
          performing_large_action = false;
        }
      }

    }
  } // -- end of '#' action specifier

}

void MegaBowieShoreline::armOperatorControl() {

  int current_arm_pos = bowiearm.getArmPos();
  uint8_t step = 1;
  uint8_t del = 3;

  if(current_time-last_arm_control_cmd <= 500 && current_time > 3000 && updating_arm == true) {

    if(target_heading == false) { // headed down to ARM_MIN
      for(int i=current_arm_pos; i>target_arm_pos; i-=step) {
        bowiearm.arm.writeMicroseconds(i);
        bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
        bowiearm.setArmPos(i);

        uint16_t endPos = 0;
        if(i >= ARM_HOME) {
          endPos = clawParallelValBounds(i, ARM_HOME, ARM_MAX, END_HOME-100, END_PARALLEL_TOP+300);
        } else {
          endPos = clawParallelValBounds(i, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-100, END_HOME-100);
        }

        bowiescoop.scoop.writeMicroseconds(endPos);
        bowiescoop.setEndPos(endPos);
        delay(del);
        servoInterrupt(SERVO_ARM_AND_END_KEY, i);
      }
    } else if(target_heading == true) { // headed up to ARM_MAX
    
      for(int i=current_arm_pos; i<target_arm_pos; i+=step) {
        bowiearm.arm.writeMicroseconds(i);
        bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
        bowiearm.setArmPos(i);
        
        uint16_t endPos = 0;
        if(i >= ARM_HOME) {
          endPos = clawParallelValBounds(i, ARM_HOME, ARM_MAX, END_HOME-100, END_PARALLEL_TOP+300);
        } else {
          endPos = clawParallelValBounds(i, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-100, END_HOME-100);
        }

        bowiescoop.scoop.writeMicroseconds(endPos);
        bowiescoop.setEndPos(endPos);
        delay(del);
        servoInterrupt(SERVO_ARM_AND_END_KEY, i);
      }
    }

  }

}

void MegaBowieShoreline::servoInterrupt(int key, int val) {
  bowieInstance->processServoInterrupt(key, val);
}

void MegaBowieShoreline::processServoInterrupt(int key, int val) {

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
    break;
    case SERVO_HOPPER_KEY:
    break;
    case SERVO_LID_KEY:
    break;
    case SERVO_EXTRA_KEY:
    break;
    case SERVO_ARM_AND_END_KEY:
    break;
  }

}

void MegaBowieShoreline::updatePeriodicMessages() {
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

---- Logging ----

*/

void MegaBowieShoreline::updateLogSensorData() {

  bowielogger.setLogData_t(LOG_TIME, now());
  bowielogger.setLogData_u8(LOG_MOTOR_A_SPEED, bowiedrive.getMotorASpeed());
  int r = 99;
  bool b = bowiedrive.getMotorADir();
  b = !b; // have to reverse it to make sense for the logging (now 1 = fwd)
  r = b ? 1 : 0;
  bowielogger.setLogData_u8(LOG_MOTOR_A_DIR, r);

  bowielogger.setLogData_u8(LOG_MOTOR_B_SPEED, bowiedrive.getMotorBSpeed());
  r = 99;
  b = bowiedrive.getMotorBDir();
  b = !b; // have to reverse it to make sense for the logging (now 1 = fwd)
  r = b ? 1 : 0;
  bowielogger.setLogData_u8(LOG_MOTOR_B_DIR, r);

  bowielogger.setLogData_u16(LOG_MOTOR_CURRENT_SENSOR, motorCurrent.getCurrentSensorReading());

  bowielogger.setLogData_u16(LOG_SERVO_POS_ARM_L, bowiearm.getArmPos());
  bowielogger.setLogData_u16(LOG_SERVO_POS_ARM_R, SERVO_MAX_US - bowiearm.getArmPos() + SERVO_MIN_US);
  bowielogger.setLogData_u16(LOG_SERVO_POS_END, bowiescoop.getEndPos());
  bowielogger.setLogData_u16(LOG_SERVO_POS_HOPPER, bowiehopper.getHopperPos());
  bowielogger.setLogData_u16(LOG_SERVO_POS_LID, bowiehopper.getLidPos());
  bowielogger.setLogData_u16(LOG_SERVO_POS_EXTRA, 0);
  bowielogger.setLogData_u16(LOG_SERVO_CURRENT_SENSOR, servoCurrent.getCurrentSensorReading());

  bowielogger.setLogData_u8(LOG_LED_FRONT_L, bowielights.getLight(0));
  bowielogger.setLogData_u8(LOG_LED_FRONT_R, bowielights.getLight(1));
  bowielogger.setLogData_u8(LOG_LED_BACK_L, bowielights.getLight(2));
  bowielogger.setLogData_u8(LOG_LED_BACK_R, bowielights.getLight(3));

  // The IMU & GPS vars can be set by the user in the sketch, like this!
  // bowielogger.setLogData_f(LOG_IMU_PITCH, imu_pitch);
  // bowielogger.setLogData_f(LOG_IMU_ROLL, imu_roll);
  // bowielogger.setLogData_f(LOG_IMU_YAW, imu_yaw);
  // bowielogger.setLogData_f(LOG_COMPASS_HEADING, compass_heading);
  // bowielogger.setLogData_u8(LOG_GPS_SATS, gps_sats);
  // bowielogger.setLogData_f(LOG_GPS_HDOP, gps_hdop);
  // bowielogger.setLogData_f(LOG_GPS_LATITUDE, gps_lat);
  // bowielogger.setLogData_f(LOG_GPS_LONGITUDE, gps_lon);
  // bowielogger.setLogData_f(LOG_GPS_ALTITUDE, gps_alt);

  // TODO - battery sensor
  //bowielogger.setLogData_u16(LOG_BATTERY_SENSOR, analogRead(A24));  

  bowielogger.setLogData_u16(LOG_COMM_XBEE_LATENCY, bowiecomms_xbee.getCommLatency());
  bowielogger.setLogData_u16(LOG_COMM_ARDUINO_LATENCY, bowiecomms_arduino.getCommLatency());

}


/*

---- Current Sensors ----

*/

void MegaBowieShoreline::waitingToCoolDown_ServosCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
  bowieInstance->waitingToCoolDown_Servos(first);
}

void MegaBowieShoreline::reactivateAfterCoolDown_ServosCallback() {
  // you might want to re-attach the servos here
  // and send them back to a position
  // (or re-activate the dc motors)
  bowieInstance->reactivateAfterCoolDown_Servos();
}

void MegaBowieShoreline::overCurrentThreshold_ServosCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
  bowieInstance->overCurrentThreshold_Servos(first);
}

void MegaBowieShoreline::waitingToCoolDown_MotorsCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
  bowieInstance->waitingToCoolDown_Motors(first);
}

void MegaBowieShoreline::reactivateAfterCoolDown_MotorsCallback() {
  // you might want to re-attach the servos here
  // and send them back to a position
  // (or re-activate the dc motors)
  bowieInstance->reactivateAfterCoolDown_Motors();
}

void MegaBowieShoreline::overCurrentThreshold_MotorsCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
  bowieInstance->overCurrentThreshold_Motors(first);
}


void MegaBowieShoreline::waitingToCoolDown_Servos(bool first) {

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

void MegaBowieShoreline::reactivateAfterCoolDown_Servos() {

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

void MegaBowieShoreline::overCurrentThreshold_Servos(bool first) {
  if(!PREVENT_OVER_CURRENT) return;

  servos_deactivated_over_current = true;

  for(int i=0; i<2; i++) {
    buzz(NOTE_B1, 100);
    delay(100);
  }

}

void MegaBowieShoreline::waitingToCoolDown_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}

void MegaBowieShoreline::reactivateAfterCoolDown_Motors() {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}

void MegaBowieShoreline::overCurrentThreshold_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
  // TODO (future) - see how often this occurs after looking at the logs
}


/*

---- Movements ----

*/
void MegaBowieShoreline::scoopSequence(int frame_delay) {

  // 1. move arm down (not all the way)
  // 2. move scoop slightly above parallel
  // 3. drive forward
  // 4. move scoop to parallel a bit
  // 5. drive forward
  // 6. tilt up quickly
  // 7. move down slightly slowly
  // 8. tilt up quickly

  //bool DEBUGGING_ANIMATION = false;
  
  // bool was_hopper_parked = bowiehopper.getHopperParked();
  // bool was_lid_parked = bowiehopper.getLidParked();
  // if(was_hopper_parked) bowie.unparkHopper();
  // if(was_lid_parked) bowie.unparkLid();

  // 2. 
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM, 5, 1);
  delay(frame_delay);

  // 1.
  bowiearm.moveArm(ARM_MIN+100, 1, 4);
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
  if(BOT_DEBUG_MEGA) Serial << "Going to MOTORS FWD 255...";
  bowiedrive.rampSpeed(true, 100, 255, 20, 10);
  bowiedrive.goSpeed(true, 255, 250);
  // stop motors!
  if(BOT_DEBUG_MEGA) Serial << "Going to MOTORS FWD 0...";
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
  if(BOT_DEBUG_MEGA) Serial << "Going to END_PARALLEL_BOTTOM-700...";
  bowiescoop.moveEnd(END_PARALLEL_BOTTOM-700, 3, 1); // todo - check this again, its less than end min
  delay(20);

  delay(frame_delay);

  // lift arm with scoop parallel to ground
  // this has to be adjusted if going faster
  if(BOT_DEBUG_MEGA) Serial << "Going to ARM_HOME...";
  moveArmAndEnd(ARM_HOME, 5, 2, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-700, END_HOME-550);//END_PARALLEL_BOTTOM-400);

  delay(150);

  delay(frame_delay);
  
  // lift arm with scoop parallel to ground
  if(BOT_DEBUG_MEGA) Serial << "Going to ARM_MAX...";
  moveArmAndEnd(ARM_MAX, 3, 3, ARM_HOME, ARM_MAX, END_HOME-550, END_PARALLEL_TOP-400);//END_PARALLEL_BOTTOM-400, END_PARALLEL_TOP-100);
  
  delay(frame_delay);

  // open lid
  if(BOT_DEBUG_MEGA) Serial << "Going to LID_MIN...";
  bowiehopper.moveLid(LID_MIN, 5, 2);
  
  delay(frame_delay);

  // dump scoop
  if(BOT_DEBUG_MEGA) Serial << "Going to END_MIN...";
  bowiescoop.moveEnd(END_MIN, 5, 3);
  
  delay(frame_delay);

  // bring scoop back to position
  if(BOT_DEBUG_MEGA) Serial << "Going to END_PARALLEL_TOP-200...";
  bowiescoop.moveEnd(END_PARALLEL_TOP-200, 5, 3);
  
  delay(frame_delay);

  // close lid
  if(BOT_DEBUG_MEGA) Serial << "Going to LID_MAX...";
  bowiehopper.moveLid(LID_MAX, 5, 2);
  
  delay(frame_delay);

  // 
  // drive backward a bit
  if(BOT_DEBUG_MEGA) Serial << "Going to MOTORS FWD 255...";
  bowiedrive.rampSpeed(false, 100, 255, 20, 10);
  bowiedrive.goSpeed(false, 255, 100);
  
  // stop motors!
  if(BOT_DEBUG_MEGA) Serial << "Going to MOTORS FWD 0...";
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

void MegaBowieShoreline::emptyScoop() {

  uint16_t prev_arm_pos = bowiearm.getArmPos();

  // raise the arm while keeping scoop parallel to ground

  uint16_t endPos;
  uint16_t armPos;

  for(int i=prev_arm_pos; i<ARM_MAX; i+=2) {
    bowiearm.arm.writeMicroseconds(i);
    bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
    bowiearm.setArmPos(i);
    endPos = clawParallelValBounds(i, prev_arm_pos, ARM_MAX, END_HOME, END_PARALLEL_TOP);
    bowiescoop.scoop.writeMicroseconds(endPos);
    bowiescoop.setEndPos(endPos);
    delay(3); 
    servoInterrupt(SERVO_ARM_AND_END_KEY, i);
    //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
  }
  bowiearm.arm.writeMicroseconds(ARM_MAX);
  bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - ARM_MAX + SERVO_MIN_US);
  bowiearm.setArmPos(ARM_MAX);
  bowiescoop.scoop.writeMicroseconds(END_PARALLEL_TOP);
  bowiescoop.setEndPos(endPos);
  delay(3);
  servoInterrupt(SERVO_ARM_AND_END_KEY, armPos);

  // ---

  bowiehopper.moveLid(LID_MIN, 4, 1); // open lid

  bowiescoop.moveEnd(END_MIN, 3, 2); // dump scoop
  for(int i=0; i<4; i++) {
    bowiescoop.moveEnd(END_MIN-100, 3, 1);
    delay(10);
    bowiescoop.moveEnd(END_MIN+100, 3, 1);
    delay(10);
  }

  // lower the arm back to the previous position

  for(int i=ARM_MAX; i>prev_arm_pos; i-=2) {
    bowiearm.arm.writeMicroseconds(i);
    bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
    bowiearm.setArmPos(i);
    endPos = clawParallelValBounds(i, prev_arm_pos, ARM_MAX, END_HOME, END_PARALLEL_TOP);
    bowiescoop.scoop.writeMicroseconds(endPos);
    bowiescoop.setEndPos(endPos);
    delay(3); 
    servoInterrupt(SERVO_ARM_AND_END_KEY, i);
    //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
  }
  bowiearm.arm.writeMicroseconds(prev_arm_pos);
  bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - prev_arm_pos + SERVO_MIN_US);
  bowiearm.setArmPos(prev_arm_pos);
  bowiescoop.scoop.writeMicroseconds(END_HOME);
  bowiescoop.setEndPos(endPos);
  delay(3);
  servoInterrupt(SERVO_ARM_AND_END_KEY, armPos);


  bowiehopper.moveLid(LID_MAX, 4, 1); // close lid

}

void MegaBowieShoreline::deposit() {

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
  if(BOT_DEBUG_MEGA) Serial << "Going to LID_MIN";
  bowiehopper.moveLid(LID_MIN, 3, 2);
  delay(10);

  if(BOT_DEBUG_MEGA) Serial << "Going to TILT_MIN";
  bowiehopper.moveHopper(TILT_MIN, 2, 2);
  delay(100);

  delay(1000);

  if(BOT_DEBUG_MEGA) Serial << "Shaking the hopper TILT_MIN+300 to TILT_MIN";
  for(int i=0; i<3; i++) {
    bowiehopper.moveHopper(TILT_MIN+300, 1, 2);
    delay(50);
    bowiehopper.moveHopper(TILT_MIN, 1, 2);
    delay(50);
  }
  
  delay(1000);

  if(BOT_DEBUG_MEGA) Serial << "Going to TILT_MAX";
  bowiehopper.moveHopper(TILT_MAX, 2, 2);
  delay(100);

  if(BOT_DEBUG_MEGA) Serial << "Positioning closed just in case TILT_MAX-200 to TILT_MAX";
  for(int i=0; i<2; i++) {
    bowiehopper.moveHopper(TILT_MAX-200, 1, 2);
    delay(50);
    bowiehopper.moveHopper(TILT_MAX, 1, 2);
    delay(50);
  }
  
  bowiehopper.parkHopper();

  // close lid
  if(BOT_DEBUG_MEGA) Serial << "Going to LID_MAX";
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

void MegaBowieShoreline::moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax) {

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
      bowiearm.setArmPos(i);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      bowiescoop.scoop.writeMicroseconds(endPos);
      bowiescoop.setEndPos(endPos);
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
      bowiearm.setArmPos(i);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      bowiescoop.scoop.writeMicroseconds(endPos);
      bowiescoop.setEndPos(endPos);
      arm_position = i;
      end_position = endPos;
      delay(del); 
      servoInterrupt(SERVO_ARM_AND_END_KEY, i);
      //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  // bowiearm.arm.writeMicroseconds(armPos);
  // bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - armPos + SERVO_MIN_US);
  // bowiearm.setArmPos(armPos);
  // endPos = clawParallelValBounds(armPos, armMin, armMax, endMin, endMax);
  // bowiescoop.scoop.writeMicroseconds(endPos);
  // bowiescoop.setEndPos(endPos);
  // arm_position = armPos;
  // end_position = endPos;
  // delay(del);
  // servoInterrupt(SERVO_ARM_AND_END_KEY, armPos);
  //if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_hopper) { // move hopper back to original position
    bowiehopper.moveHopper(hopper_original_pos);
  }

}

// ---------
// This code is from Micah Black, during Random Hacks of Kindness in Ottawa 2017!
// Check out his store: a2delectronics.ca

//get a parallel claw value
int MegaBowieShoreline::clawParallelVal(int arm_Val) {
  return 1;
  // TODO
  //return (int)constrain( map(arm_Val, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP) , END_PARALLEL_BOTTOM, END_PARALLEL_TOP);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.

//get a parallel claw value
int MegaBowieShoreline::clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax) {
  return (int)constrain( map(arm_Val, armMin, armMax, endMin, endMax) , endMin, endMax);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.



/*

---- Specific ----

*/

void MegaBowieShoreline::buzz(long frequency, long length) {
  long delayValue = 1000000 / frequency / 2;
  long numCycles = frequency * length / 1000;
  for (long i = 0; i < numCycles; i++) { 
    digitalWrite(SPEAKER, HIGH); 
    delayMicroseconds(delayValue); 
    digitalWrite(SPEAKER, LOW); 
    delayMicroseconds(delayValue); 
  }
}

