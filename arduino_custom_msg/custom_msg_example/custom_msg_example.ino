#include <ros.h>
#include <test_msgs.h>

ros::NodeHandle nh;

test_msgs::test_msgs msg;
ros::Publisher pub("custom_msg", &msg);
int count = 0;

void setup() {
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  uint32_t t = millis();
  if((t - tTime) >= 1000)
  {
    msg.id.data = 1;
    msg.data.data = count;
    data_pub.publish(&msg);
    count++;
    tTime = t;
  }
  nh.spinOnce();
}
