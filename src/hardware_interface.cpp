/**
 *
 * @file hardware_interface.cpp
 *
 * @brief Implementation of the Hardware Interface for the sonar application
 *
 * @author Angelos Plastropoulos
 * @date 24.07.2018
 *
 */

#include "hardware_interface.h"

/**
 * @detail This is the place where we set-up all the required paramters to allow the interface to work
 */

hardware_interface_robot_sonar::hardware_interface_robot_sonar() {
  node_name  = ros::this_node::getName();

  encoder_pub = n.advertise<std_msgs::Float32>("encoder", 10);
  velocity_sub = n.subscribe("sonar_velocity", 10, &hardware_interface_robot_sonar::velocity_callback, this);
  
  n.getParam(node_name + "/" + "port", port);
  n.getParam(node_name + "/" + "baud", baud_rate);
  n.getParam(node_name + "/" + "period", period);

  
  try {
	my_arduino = new Arduino(port.c_str(), baud_rate);
  } catch (int e) {
	ROS_FATAL("Problem bringing up Arduino");
	ros::shutdown();
	exit(EXIT_FAILURE);
  }
  ROS_INFO("board started at port:%s with baud:%d and transmission period T:%d", port.c_str(), baud_rate, period);
}

// destructor
hardware_interface_robot_sonar::~hardware_interface_robot_sonar() {
  fprintf (stderr, "Shutting down the serial connection\n");
  my_arduino->close_serial();
}


//velocity callback
void hardware_interface_robot_sonar::velocity_callback(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("Velocity: [%d]", msg->data);
  velocity = msg->data;
}

//spin the comms
void hardware_interface_robot_sonar::spin() {
  
  int counter;
  
  while (ros::ok()) {
	
	std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
	
	//ROS_INFO("Looping");
    //ROS_INFO("packet #%ld", my_arduino->count++);
    
    //velocity = (int16_t)(counter % 255);
    
    //sending driller switch
    ROS_INFO("Sending to mc requested height: [%d]", velocity);
    my_arduino->send_to_mc(velocity);

    ros::spinOnce();
    usleep(period*1000);

    float value = my_arduino->read_from_mc();
    
    std_msgs::Float32 encoder;
	encoder.data = value;
	
    encoder_pub.publish(encoder);
	ROS_INFO("Received encoder %f", value);
    counter++;
	
	std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
	std::chrono::milliseconds diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	ROS_INFO("ROS iteration duration %lu ms", diff.count());
  }
  
  int16_t v = 0;
  my_arduino->send_to_mc(v);

  ros::spinOnce();
  usleep(period);

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "hardware_interface_sonar_node");

  ROS_INFO("Starting the node\n");
  hardware_interface_robot_sonar my_hardware_interface;

  //ros::ServiceServer service = my_hardware_interface.n.advertiseService<hardware_interface_sonar::Height::Request, hardware_interface_sonar::Height::Response>("/height", boost::bind(&hardware_interface_robot_sonar::height_callback, my_hardware_interface, _1, _2));
  my_hardware_interface.spin();

  return 0;
}
