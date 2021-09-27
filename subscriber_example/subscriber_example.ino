#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };

void messageCb( const std_msgs::Bool& msg) {
  for(int i = 0; i < 4; i++)
  {
    if(msg.data == true)
    {
      digitalWrite(led_pin_user[i], LOW);
    }
    if(msg.data == false)
    {
      digitalWrite(led_pin_user[i], HIGH);
    }
  }
}
ros::Subscriber<std_msgs::Bool> sub("Led_state", messageCb );

void setup() {
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}
