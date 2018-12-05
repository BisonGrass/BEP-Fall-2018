#include "Car.h"
#include "Arduino.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

//Implement Boundries for servo and motor input
float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) { x = MOTOR_MAX; }
  if (x < MOTOR_MIN) { x = MOTOR_MIN; }
  return x;
}
float Car::saturateServo(float x) {
  if (x > THETA_MAX) { x = THETA_MAX; }
  if (x < THETA_MIN) { x = THETA_MIN; }
  return x;
}

//Encoder read and incrementing functions
int Car::readQD(){
  //Returns value from the QRE1113
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( BR_PIN, OUTPUT );
  digitalWrite( BR_PIN, HIGH );
  delayMicroseconds(10);
  pinMode( BR_PIN, INPUT );
  long time = micros();
  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while(digitalRead(BR_PIN) == HIGH && micros() - time < 3000);
  int diff = micros() - time;
  return diff;
}

//Turn on motor & servo and initialize neutral position
void Car::initActuators() {
    motor.attach(MOTOR_PIN, 1000, 2000);
    steering.attach(SERVO_PIN, 1000, 2000);
  }
void Car::armActuators(){
      motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
      steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
      delay(1000);
    }

//Actuator callback function for /cmd_vel subscriber
void Car::writeToActuators(const geometry_msgs::Twist& twistMsg) {
  int steeringAngle = fmap(-twistMsg.angular.z, -1.0, 1.0, THETA_MIN, THETA_MAX) ;
  steering.writeMicroseconds( (uint16_t) saturateServo( steeringAngle ) );

  int escCommand = fmap(-twistMsg.linear.x, -20.0, 20.0, MOTOR_MIN, MOTOR_MAX) ;
  motor.writeMicroseconds( (uint16_t) saturateMotor( escCommand ) );
}

//Function to stop the car if signal is lost
void Car::killMotor() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
}

// Arduino 'map' funtion for floating point
double Car::fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
