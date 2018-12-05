#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <Car.h>

// Initialize an instance of the Car class as car
Car car;

//Boolean keeping track of whether the arduino has received a signal from the ecu
uint8_t received_ecu_signal = 0;

//Actuator callback function
void driveCallback(const geometry_msgs::Twist& twistMsg) {
  car.writeToActuators(twistMsg);
  received_ecu_signal = 1;
}

// Variables for time step
volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long ecu_t0;

//Variables for encoder reading
int QRE_Value;
const uint16_t threshold_value = 200;  // 0 = maximum reflection, 3000 = no reflection
const uint8_t encoder_count = 8;
uint8_t current_color = 0;  // Black = 0, white = 1;
long count = 0;
int rotations = 0;
volatile unsigned long rot_dt;
volatile unsigned long rot_t0;
volatile unsigned long old_count;
volatile unsigned long new_count;
volatile unsigned long enc_dt;
volatile unsigned long enc_rot;

//Create Nodehandle
ros::NodeHandle  nh;

//Global message variables
std_msgs::Int32 wheel_msg;

//Create publishers & subscribers
ros::Publisher pub_encoder("encoder", &wheel_msg);
ros::Subscriber<geometry_msgs::Twist> sub_drive("/cmd_vel", driveCallback);


/**************************************************************************
  ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up actuators
  car.initActuators();

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.advertise(pub_encoder);
  nh.subscribe(sub_drive);

  //Initialize actuators in neutral position, 1 sec delay for arming and ROS
  car.armActuators();
  t0 = millis();
  ecu_t0 = millis();
}


/**************************************************************************
  ARDUINO MAIN lOOP
**************************************************************************/
void loop() {

  

  //Kill ECU if there is no signal in the last 1s
  if ( (millis() - ecu_t0) >= 5000) {
    if (!received_ecu_signal) {
      car.killMotor();
    } else {
      received_ecu_signal = 0;
    }
    ecu_t0 = millis();
  }

  //Read encoder reflection value
  QRE_Value = car.readQD();

  //Check if encoder state has changed
  if(QRE_Value > threshold_value && current_color == 1){
      current_color = 0;
      ++count;
             
  }
  else if(QRE_Value < threshold_value && current_color == 0){
      current_color = 1;
      ++count;     
  }

//calculate the amount of time it takes per rotation
//  if(count == 16)
//    {
//      rotations = rotations + 2;
//      count = 0;  
//      rot_dt = (millis() - rot_t0);
//      rot_t0 = millis();
//      wheel_msg.data = rot_dt/2;
//    }
    
  dt = millis() - t0;
  //Publish on topics at sample rate
  if (dt > 50) 
  {
    new_count = count;
    enc_dt = new_count - old_count;
    //enc_rot = (enc_dt*(7500/dt));
    wheel_msg.data = enc_dt; 
    pub_encoder.publish(&wheel_msg);
    t0 = millis();
    old_count=new_count;
  }

  nh.spinOnce();
}
