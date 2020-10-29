#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

/**
This module sends commands to the Prober URDF robot

Messages will be structured like that:

Topic: /joint_states 
Type: sensor_msgs/JointState
name: 
  - base_to_probe_arm
  - probe_arm_to_probe
  - base_to_radar
  - base_to_left_front_wheel
  - base_to_left_back_wheel
  - base_to_right_back_wheel
  - base_to_right_front_wheel
position: []
velocity: []
effort: []
**/

// Define some link limits
#define PROBE_ARM_MIN 0.0
#define PROBE_ARM_MAX 0.3

#define PROBE_MIN 0.0
#define PROBE_MAX 0.15

#define RADAR_MIN -1.57
#define RADAR_MAX 1.57

int sequence_numer = 0; 

// Helper funciton to send messages
void publish(ros::Publisher* publisher, sensor_msgs::JointState& msg)
{
  ros::Rate loop_rate(50);

  msg.header.stamp = ros::Time::now();
  msg.header.seq = sequence_numer;
  publisher->publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
}

int main(int argc, char **argv)
{
  // Create ros node
  ros::init(argc, argv, "joint_position_controller");
  ros::NodeHandle nh;

  // Create publisher
  ros::Publisher joint_state_publisher = 
    nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  // Fill constant fields 
  sensor_msgs::JointState msg;

  msg.name.push_back("base_to_probe_arm");
  msg.name.push_back("probe_arm_to_probe");
  msg.name.push_back("base_to_radar");
  msg.name.push_back("base_to_left_front_wheel");
  msg.name.push_back("base_to_left_back_wheel");
  msg.name.push_back("base_to_right_back_wheel");
  msg.name.push_back("base_to_right_front_wheel");

  msg.position.resize(msg.name.size());
  
  double wheel_angle = 0.0;
  double radar_angle = 0.0;
  double radar_dir = 1;
    
  while(ros::ok())
  {
    // Spin the wheels
    for(int i = 0; i < 250; ++i)
    {
      wheel_angle += 0.05;
      for(int j = 3; j < msg.position.size(); ++j)
      {
        msg.position[j] = wheel_angle;
      }

      publish(&joint_state_publisher, msg);
      ++sequence_numer;
    }

    // Expand the probe arm
    for(int i = 1; i < 250; ++i)
    {
      msg.position[0] = PROBE_ARM_MIN + ((float)i / 250.0) * PROBE_ARM_MAX;

      publish(&joint_state_publisher, msg);

      ++sequence_numer;
    }

    // Expand the probe
    for(int i = 1; i < 250; ++i)
    {
      msg.position[1] = PROBE_MIN + ((float)i / 250.0) * PROBE_MAX;

      publish(&joint_state_publisher, msg);

      ++sequence_numer;
    }

    // Retract the probe
    for(int i = 249; i > 0; --i)
    {
      msg.position[1] = PROBE_MIN + ((float)i / 250.0) * PROBE_MAX;
      
      publish(&joint_state_publisher, msg);

      ++sequence_numer;
    }

    // Retract the probe arm
    for(int i = 249; i > 0; --i)
    {
      msg.position[0] = PROBE_ARM_MIN + ((float)i / 250.0) * PROBE_ARM_MAX;
      
      publish(&joint_state_publisher, msg);

      ++sequence_numer;
    }

    // Move the radar
    for(int i = 249; i > 0; --i)
    {
      if (msg.position[2] >= RADAR_MAX)
      {
        radar_dir = -1;
      }
      else if (msg.position[2] <= RADAR_MIN)
      {
        radar_dir = 1;
      }
      
      radar_angle += radar_dir * 0.05;
      msg.position[2] = radar_angle;
      
      publish(&joint_state_publisher, msg);

      ++sequence_numer;
    }

    sequence_numer = 0;
  }

  return 0;
}
