/**
 *
 * @file hardware_interface_sonar.h
 *
 * @class hardware_interface_robot_sonar
 * 
 * @brief This class implements the interface between the ROS ecosystem and the arduino layer 
 * for a custom sonar communication
 *
 * @author Angelos Plastropoulos
 * @date 25.07.2018
 *
 */

#ifndef HARDWAREINTERFACESONAR_H
#define HARDWAREINTERFACESONAR_H

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
    /** 
     * @brief constructor
	 */
    hardware_interface_robot_sonar();

    /** 
     * @brief destructor
	 */
    ~hardware_interface_robot_sonar();
    
    /** 
     * @brief spin the communication loop
	 */
    void spin();
    
  private:
	/** 
     * @brief arduino serial communication object
	 */
    Arduino* my_arduino;

    /**
     * @brief arduino node name
	 */
    std::string node_name;
	
    /**
     * @brief arduino port
	 */
    std::string port;
    
	/**
     * @brief baud rate of the communication
	 */
    int baud_rate;
	
	/**
     * @brief period of the communication
	 */
    int period;
    
	/**
     * @brief requested velocity for the motor
	 */
    int16_t velocity;
    
    /**
     * @brief handle for the ROS node
	 */
    ros::NodeHandle n;                  
	
	/**
     * @brief height subscriber
	 */    
	ros::Subscriber velocity_sub;       
    
	/**
     * @brief encoder publisher
	 */
    ros::Publisher encoder_pub;         
	
	/** 
     * @brief callback that listens to requested velocity
	 * @param pointer to Int16 message
	 */
    void velocity_callback(const std_msgs::Int16::ConstPtr& msg);
};

#endif // HARDWAREINTERFACESONAR_H
