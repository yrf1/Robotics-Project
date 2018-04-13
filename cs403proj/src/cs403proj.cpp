#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sstream>
#include <time.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

// Include any additional header or service/message files

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::MatrixXf;
using geometry_msgs::Point32;

ros::Subscriber PC_Sub;
ros::Publisher image_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment3");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 3
  srand(time(NULL));

  ros::spin();

  return 0;
}
