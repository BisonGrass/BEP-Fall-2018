#ifndef Car_h
#define Car_h

#include "Arduino.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <Servo.h>


class Car {
  public:
    void initActuators();
    void armActuators();
    void writeToActuators(const geometry_msgs::Twist& twistMsg);

    int readQD();

    // Interrupt service routines
    void killMotor();

  private:
    //Define Pin Connections
    const uint8_t BR_PIN = 5;
    const uint8_t MOTOR_PIN   = 8;
    const uint8_t SERVO_PIN   = 9;

    //Servo neutral state (milliseconds)
    float throttle_neutral_ms = 1500.0;
    float servo_neutral_ms = 1500.0;

    // Motor limits
    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 1000;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1000;
    const int THETA_MAX = 2000;
    const int THETA_MIN = 1050;

    //Create Servo Objects
    Servo steering;
    Servo motor;

    // Utility functions
    double fmap (double toMap, double in_min, double in_max, double out_min, double out_max);
    float saturateMotor(float x);
    float saturateServo(float x);
};

#endif
