# Bowie Shoreline v1.0

![Robot Missions logo, a green leaf on a white diamond situated within a purple circle](http://robotmissions.org/images/github/robot_missions_colour_500px.png)

Bowie Shoreline is for the [Robot Missions](http://robotmissions.org) introductory robot platform, _Bowie_, as applied to Mission #1: Shoreline Cleanup. This contains behaviours that use the core modules in the Bowie Library.

### More information about our robot platform at the [Robot Missions Website ➞](http://robotmissions.org)

Compatible with the Robot Missions Brain Board, using either a [Teensy 3.6](https://www.pjrc.com/store/teensy36.html) for full functionality (Mega), or a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) (Mini).

## System Diagram

![System diagram showing the information flow between core modules](http://robotmissions.org/images/github/BowieShorelineFirmware_web.jpg)

## Required Libraries

You will need to have these libraries installed:

* [BowieLib](https://github.com/RobotMissions/BowieLib)
* [Streaming](http://arduiniana.org/libraries/streaming/)
* [PromulgateBig](https://github.com/RobotGrrl/PromulgateBig)
* [Xbee](https://github.com/andrewrapp/xbee-arduino)
* [SD](https://www.arduino.cc/en/Reference/SD)
* [TimeLib](https://www.pjrc.com/teensy/td_libs_Time.html)
* Servo _Included with Arduino_
* SPI _Included with Arduino_

## API

Below you will find the messages that can be input to the robot, and messages the robot can output. To review how messages are sent through the communications interface, please see [this section of the Bowie Lib](https://github.com/RobotMissions/BowieLib#comms-messages).

**Input: Operator to Robot**

| Description | Action specifier | Command | Key | Value |
| --- | --- | --- | --- | --- |
| **Left motor** | `#` | `L` | `1` is forward, `0` is backward | `0` is slowest, `255` is fastest |
| **Right motor** | `#` | `R` | `1` is forward, `0` is backward | `0` is slowest, `255` is fastest |
| **Arm** | `#` | `S` | `1` for up, `0` for down | Value to increment the position by |
| **Scoop** | `#` | `C` | _TBD_ | _TBD_ |
| **Button #1 (Red)** | `#` | `P` | `1` is pressed on, `0` is pressed off | _None_ |
| **Button #2 (Green)** | `#` | `G` | `1` is pressed on, `0` is pressed off | _None_ |
| **Button #3 (Yellow)** | `#` | `Y` | `1` is pressed on, `0` is pressed off | _None_ |
| **Button #4 (Blue)** | `#` | `B` | `1` is pressed on, `0` is pressed off | _None_ |
| **Button #5 (White)** | `#` | `W` | `1` is pressed on, `0` is pressed off | _None_ |
| **Button #6 (Black)** | `#` | `N` | `1` is pressed on, `0` is pressed off | _None_ |
| **Super Bright LEDs** | `#` | `Q` | `1` is front left TODO | `0` is off, `255` is full brightness |
| **Joystick Button** | `#` | `J` | `1` is pressed on, `0` is pressed off | _None_ |
| **Mode Switch** | `#` | `M` | _TBD_ | _TBD_ |
| **State** | `#` | `Z` | _TBD_ | _TBD_ |
| **Operator ID** | `$` | `X` | `1` | `OPERATOR_ID` from `0` to `255` |


**Output: Robot to Operator**

| Description | Action specifier | Command | Key | Value |
| --- | --- | --- | --- | --- |
| **Motor Current Sensor** | `#` | `E` | `1` | Current sensor reading of type `uint16_t` |
| **Servo Current Sensor** | `#` | `F` | `1` | Current sensor reading of type `uint16_t` |
| **Robot ID** | `$` | `W` | `1` | `ROBOT_ID` from `0` to `255` |
| **Reserved for testing** | `@` | `Z` | `0` | `0` |

If your robot running Mission #1 needs more functionality, feel free to add new API commands. For example, one could add an API command for an output of the IMU.

## License

Code is released under the [MIT license](https://opensource.org/licenses/MIT).

## Bug Reports / Feature Requests

Found a bug? Looking for a feature? Let us know by opening an Issue report. Further questions can be posted to our [forum](http://forum.robotmissions.org).

---

🤖✌️🌎

**Robot Missions - Helping the planet with robots**