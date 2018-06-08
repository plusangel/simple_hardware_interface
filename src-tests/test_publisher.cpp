#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include <sstream>

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "velocity_publisher");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("sonar_velocity", 10);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::Int16 msg;
    
    msg.data = count % 255;
   
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}