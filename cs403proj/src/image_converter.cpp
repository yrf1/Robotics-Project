#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sstream>
#include <time.h>
#include <deque>
#include <tuple>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Original";

int iLowH = 0;
int iHighH = 255;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

int iLastX = -1;
int iLastY = -1;

int queueLimit = 32;
deque< tuple<int,int> > queue;

void CreateBar(){
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 50, CV_RGB(255,0,0));

    Mat imgHSV;

    cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 10000)
    {
    //calculate the position of the ball
      int posX = dM10 / dArea;
      int posY = dM01 / dArea;
      queue.push_front(tuple<int,int>(posX,posY));
      if(queue.size() >= queueLimit){
        queue.pop_back();
      }
    }
    else{
      if(!queue.empty()){
        queue.pop_back();
      }
    }
    if(queue.size() >=2){
      for(int n = 1; n<queue.size(); n++){
      // if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
      // {
      //  //Draw a red line from the previous point to the current point
      //   line(cv_ptr->image, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 10);
      // }
      //   iLastX = posX;
      //   iLastY = posY;
      // }
        int currPosX = get<0>(queue[n]);
        int currPosY = get<1>(queue[n]);
        int prevPosX = get<0>(queue[n-1]);
        int prevPosY = get<1>(queue[n-1]);
        int thickness = sqrt(queueLimit/(n+1))*2.5;
        line(cv_ptr->image, Point(currPosX, currPosY), Point(prevPosX, prevPosY), Scalar(0,0,255), thickness);
      }
    }

    // Update GUI Window
    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv_ptr->image = cv_ptr->image + imgLines;
    imshow(OPENCV_WINDOW, cv_ptr->image); //show the original image

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  CreateBar();
  ImageConverter ic;
  ros::spin();
  return 0;
}
