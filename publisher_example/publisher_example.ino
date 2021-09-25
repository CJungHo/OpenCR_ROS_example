#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 value;
ros::Publisher data_pub("topic", &value);
static uint32_t tTime = 0;
int count = 0;

void setup()
{
  nh.initNode();
  nh.advertise(data_pub);
}

void loop()
{
  uint32_t t = millis();
  if((t - tTime) >= 1000)
  {
    value.data = count;
    data_pub.publish(&value);
    count++;
    tTime = t;
  }
  nh.spinOnce();
}
