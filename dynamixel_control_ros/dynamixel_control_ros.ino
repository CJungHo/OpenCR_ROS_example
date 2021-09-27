#include <ros.h>
#include "Motor_driver.h"
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;
MotorDriver motor_driver;

void messageCb( const std_msgs::Int16& msg) {
  motor_driver.PosControl(1, (int64_t)msg.data); // id, pos(0~4096)
}
ros::Subscriber<std_msgs::Int16> sub("motor_pos", messageCb );

void setup() {
  Serial.begin(115200);

  nh.initNode();
  nh.subscribe(sub);
  motor_driver.init();
}

void loop() {
  nh.spinOnce();
}
