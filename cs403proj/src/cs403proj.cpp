#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sstream>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/imgproc/imgproc.hpp>

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

Vector3f calcNorm(Vector3f P0, std::vector<geometry_msgs::Point32> points){
  int pointsSize = points.size();
  MatrixXf M;
  M.resize(pointsSize,3);

  for(int i = 0; i<pointsSize; i++){
    geometry_msgs::Point32 curr_point = points.at(i);
    Vector3f Pi = Vector3f(curr_point.x,curr_point.y,curr_point.z);
    Vector3f newVect = Pi-P0;
    M(i,0) = newVect.x();
    M(i,1) = newVect.y();
    M(i,2) = newVect.z();
  }

  Matrix3f newM = M.transpose()*M;

  Eigen::EigenSolver<Matrix3f> eigen_solver;
  eigen_solver.compute(newM);
  Vector3f eigen_values = eigen_solver.eigenvalues().real();
  Matrix3f eigen_vectors = eigen_solver.eigenvectors().real();

  float smallest_eigen_value = eigen_values(0);
  Vector3f smallest_eigen_vector = eigen_vectors.col(0);

  for(int j = 1; j<3 ; j++){
    if(fabs(eigen_values(j)) < smallest_eigen_value){
      smallest_eigen_value = eigen_values(j);
      smallest_eigen_vector = eigen_vectors.col(j);
    }
  }

  return smallest_eigen_vector;
}

void part5Fn(const sensor_msgs::PointCloud msg){

  float epsilon = 0.01;
  float f_val = 0.8;
  int pointsArrSize = msg.points.size();

  bool goodModelFound = false;

  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = msg.header;

  while(!goodModelFound){

    //Randomly select n points
    std::vector<geometry_msgs::Point32> randPoints;
    float x0_1 = 0.0, y0_1 = 0.0, z0_1 = 0.0;
    for(int m = 0; m<10; m++){
      int randnum = rand()%pointsArrSize;
      randPoints.push_back(msg.points[randnum]);
      x0_1 += msg.points[randnum].x;
      y0_1 += msg.points[randnum].y;
      z0_1 += msg.points[randnum].z;
    }

    Vector3f P0 = Vector3f(x0_1/10,y0_1/10,z0_1/10);

    Vector3f norm = calcNorm(P0,randPoints);

    //Filter points that fit the plane created
    std::vector<geometry_msgs::Point32> inlinePoints;

    for(int n = 0; n<pointsArrSize; n++){
      geometry_msgs::Point32 current_point = msg.points[n];
      Vector3f Pi = Vector3f(current_point.x,current_point.y,current_point.z);
      float dot_prod = (Pi-P0).dot(norm);
      if(fabs(dot_prod)<=epsilon){
        inlinePoints.push_back(current_point);
      }
    }

    float ratio = (float)inlinePoints.size()/(float)pointsArrSize;

    //Found good model
    if(ratio>f_val){
      goodModelFound = true;
      int inlinePointSize = inlinePoints.size();
      MatrixXf M;
      M.resize(inlinePointSize,3);
      float x0 = 0.0, y0 = 0.0, z0 = 0.0;

      //Push point clouds into object
      for(int k = 0; k<inlinePointSize; k++){
        point_cloud.points.push_back(inlinePoints.at(k));
      }

      //Calculate new P0
      for(int h = 0; h<inlinePointSize; h++){
        geometry_msgs::Point32 curr_point = inlinePoints.at(h);
        x0 += curr_point.x;
        y0 += curr_point.y;
        z0 += curr_point.z;
      }

      x0 = x0/inlinePointSize;
      y0 = y0/inlinePointSize;
      z0 = z0/inlinePointSize;

      Vector3f new_P0 = Vector3f(x0,y0,z0);

      //Create matrix M
      Vector3f smallest_eigen_vector = calcNorm(new_P0,inlinePoints);

      //Set plane msgs

      // part5Pub_1.publish(point_cloud);
      // part5Pub_2.publish(plane_msg);

    }

  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment3");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 3
  srand(time(NULL));

  ros::spin();

  return 0;
}
