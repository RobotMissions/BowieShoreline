/*
 * Robot Missions Mega Bowie Shoreline (Mission #1)
 * ------------------------------------------------
 *
 * Library for the control and logic of the Robot
 * Missions robot platform (Bowie) that is designed
 * for Mission #1 - Shoreline Cleanup. Mega version,
 * using Teensy 3.6. (For Teensy 3.2, please see
 * Mini version).
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 * 
 * Erin RobotGrrl
 * Dec. 30th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>
#include <Servo.h>

#include "BowieArm.h"
#include "BowieCurrentSensor.h"
#include "BowieDrive.h"
#include "BowieHopper.h"
#include "BowieLights.h"
#include "BowieLogger.h"
#include "BowieScoop.h"
#include "BowieComms.h"

#ifndef _MEGABOWIESHORELINE_H_
#define _MEGABOWIESHORELINE_H_

#define BOT_DEBUG_MEGA false   // anything with the robot
#define COMM_DEBUG_MEGA false // anything with promulgate
#define OP_DEBUG_MEGA false    // anything with buttons, or op in general
#define XBEE_DEBUG_MEGA false // anything with the xbee scope
#define CONN_DEBUG_MEGA false // anything with the connection stack
#define MSG_DEBUG_MEGA false  // anything with adding / removing Msgs

// motor pins
#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL1 17 // ?BR
#define MOTORA_CTRL2 16 // ?FR
#define MOTORB_CTRL1 11 // ?FL
#define MOTORB_CTRL2 12 // ?BL

// servo pins
#define SERVO_ARM1 6
#define SERVO_ARM2 5
#define SERVO_END_EFFECTOR 4
#define SERVO_HOPPER_PIVOT 3
#define SERVO_HOPPER_LID 25
#define SERVO_EXTRA 24

// sensor pins
#define CURRENT_SERVO_SENS 15
#define CURRENT_MOTOR_SENS 14
  
// led pins
#define BOARD_LED 13
#define BRIGHT_LED_FRONT_LEFT 21
#define BRIGHT_LED_BACK_LEFT 20
#define BRIGHT_LED_FRONT_RIGHT 36
#define BRIGHT_LED_BACK_RIGHT 35
#define COMM_LED 2
#define SPEAKER 30

// touch probe pins
#define SCOOP_PROBE_LEFT A18
#define SCOOP_PROBE_RIGHT A17

#define REMOTE_OP_SLEEP 300

#ifndef SERVO_ARM_KEY
#define SERVO_ARM_KEY 1
#endif

#ifndef SERVO_END_KEY
#define SERVO_END_KEY 2
#endif

#ifndef SERVO_HOPPER_KEY
#define SERVO_HOPPER_KEY 3
#endif

#ifndef SERVO_LID_KEY
#define SERVO_LID_KEY 4
#endif

#ifndef SERVO_EXTRA_KEY
#define SERVO_EXTRA_KEY 5
#endif

// speaker
// source: https://www.hackster.io/sanyam-chugh/super-mario-beats-on-arduino-5d96a8
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

class MegaBowieShoreline {

  static MegaBowieShoreline *bowieInstance;

  public:
    MegaBowieShoreline();
    void setRobotID(uint8_t the_robot_id);
    void begin();
    uint8_t ROBOT_ID;

    // Components
    BowieArm bowiearm;
    BowieCurrentSensor servoCurrent;
    BowieCurrentSensor motorCurrent;
    BowieDrive bowiedrive;
    BowieHopper bowiehopper;
    BowieLights bowielights;
    BowieLogger bowielogger;
    BowieScoop bowiescoop;
    BowieComms bowiecomms_xbee;
    BowieComms bowiecomms_arduino;

    // Control
    void update(bool force_no_sleep);
    void control(Msg m);
    static void servoInterrupt(int key, int val);
    bool performing_large_action;
    
    // States
    void enableRemoteOp();
    void disableRemoteOp();
    void enableLogging();
    void disableLogging();
    void enableOverCurrentProtection();
    void disableOverCurrentProtection();

    // Movements
    void scoopSequence(int frame_delay);
    void deposit();
    void moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax);
    int clawParallelVal(int arm_Val);
    int clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax);

  private:

    // States
    bool REMOTE_OP_ENABLED; // true by default
    bool PREVENT_OVER_CURRENT; // false by default (advanced functionality)
    bool LOGGING_ENABLED; // true by default
    bool TURN_SEQUENCE_MODE; // true by default

    // Other
    uint8_t unlikely_count;
    long current_time;
    long last_ctrl;
    long last_update_periodic;

    // Periodic messages
    Msg current_sensor_periodic;
    void updatePeriodicMessages();

    // Comms Callbacks - Arduino
    static void receivedAction_Arduino(Msg m);
    static void commsTimeout_Arduino();
    static void controllerAdded_Arduino();
    static void controllerRemoved_Arduino();

    // Comms Callbacks - Xbee
    static void receivedAction_Xbee(Msg m);
    static void commsTimeout_Xbee();
    static void controllerAdded_Xbee();
    static void controllerRemoved_Xbee();

    // Current Callbacks
    static void waitingToCoolDown_ServosCallback(bool first);
    static void reactivateAfterCoolDown_ServosCallback();
    static void overCurrentThreshold_ServosCallback(bool first);
    static void waitingToCoolDown_MotorsCallback(bool first);
    static void reactivateAfterCoolDown_MotorsCallback();
    static void overCurrentThreshold_MotorsCallback(bool first);

    // Current
    int arm_pos_over_current;
    int end_pos_over_current;
    int hopper_pos_over_current;
    int lid_pos_over_current;
    bool servos_deactivated_over_current;
    void waitingToCoolDown_Servos(bool first);
    void reactivateAfterCoolDown_Servos();
    void overCurrentThreshold_Servos(bool first);
    void waitingToCoolDown_Motors(bool first);
    void reactivateAfterCoolDown_Motors();
    void overCurrentThreshold_Motors(bool first);

    // Servo interruption
    void processServoInterrupt(int key, int val);

    // Update logging sensors
    void updateLogSensorData();

    // Specific
    void buzz(long frequency, long length);

};

#endif