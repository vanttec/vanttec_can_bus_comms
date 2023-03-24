#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <sstream>
#include <vector>

float left_thrust = 0;
float right_thrust = 0;

float map_thruster(float in){
  if(in > 36.5)
    in = 1;
  else if(in < -30)
    in = -1;
  else if(in > 0)
    in /= 36.5;
  else
    in /= 30;
  return in;
}

void update_thrust(ros::Publisher &pub, float left_thrust, float right_thrust){
  std_msgs::Float32MultiArray msg;
  msg.data = std::vector<float>{left_thrust, right_thrust, 0,0,0,0,0,0};
  pub.publish(msg);
}

void leftThrustCallback(const std_msgs::Float32::ConstPtr& left, ros::Publisher &pub){
  std_msgs::Float32MultiArray msg;
  left_thrust = map_thruster(left->data);
  update_thrust(pub, left_thrust, right_thrust);
}

void rightThrustCallback(const std_msgs::Float32::ConstPtr& right, ros::Publisher &pub){
  std_msgs::Float32MultiArray msg;
  right_thrust = map_thruster(right->data);
  update_thrust(pub, left_thrust, right_thrust);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "individual_thrusters");
  ros::NodeHandle n;
  ros::Publisher motors_pub = n.advertise<std_msgs::Float32MultiArray>("/motors", 10);
  auto left_sub = n.subscribe<std_msgs::Float32>("/motors/left_thrust", 10, boost::bind(&leftThrustCallback, _1, boost::ref(motors_pub)));
  auto right_sub = n.subscribe<std_msgs::Float32>("/motors/right_thrust", 10, boost::bind(&rightThrustCallback, _1, boost::ref(motors_pub)));
  ros::spin();
  return 0;
}