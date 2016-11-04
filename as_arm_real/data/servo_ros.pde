#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <rosserial_arduino/Adc.h>

#define ROS_CALLBACK_SERVO(x) ROS_CALLBACK(servo_cb##x, std_msgs::UInt16, cmd_msg##x) \
                                        servo1.write(cmd_msg##x.data);}

// digitalWrite(13, HIGH-digitalRead(13));  //toggle led
Servo servo1, servo2, servo3, servo4, servo5, servo6;

ROS_CALLBACK_SERVO(1)
ROS_CALLBACK_SERVO(2)
ROS_CALLBACK_SERVO(3)
ROS_CALLBACK_SERVO(4)
ROS_CALLBACK_SERVO(5)
ROS_CALLBACK_SERVO(6)

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::UInt16> sub1("servo1", &cmd_msg1, servo_cb1);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", &cmd_msg2, servo_cb2);
ros::Subscriber<std_msgs::UInt16> sub3("servo3", &cmd_msg3, servo_cb3);
ros::Subscriber<std_msgs::UInt16> sub4("servo4", &cmd_msg4, servo_cb4);
ros::Subscriber<std_msgs::UInt16> sub5("servo5", &cmd_msg5, servo_cb5);
ros::Subscriber<std_msgs::UInt16> sub6("servo6", &cmd_msg6, servo_cb6);

void setup(){
  nh.initNode();
  // subscribe
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  //attach servo to pin NO.
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);
  servo5.attach(8);
  servo6.attach(9);
}

void loop(){

  nh.spinOnce();
  delay(1);
}
