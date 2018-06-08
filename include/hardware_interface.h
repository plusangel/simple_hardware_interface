#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <string.h>
#include <stdint.h>
#include <ctime>
#include <chrono>
#include <ratio>

#include "arduinoSerial.h"

class hardware_interface_robot_sonar {
  public:
    // constructor & destructor
    hardware_interface_robot_sonar();
    ~hardware_interface_robot_sonar();
    
    // spin the comms
    void spin();
    
  private:
    // arduino serial comm object
    Arduino* my_arduino;

    // arduino comm details
    std::string node_name;
    std::string port;
    int baud_rate, period;
    
    int16_t velocity;
    
    // ROS variables
    ros::NodeHandle n;                  //handle for the ROS node
    ros::Subscriber velocity_sub;       //height subscriber
    ros::Publisher encoder_pub;         //encoder publisher

    void velocity_callback(const std_msgs::Int16::ConstPtr& );
};
