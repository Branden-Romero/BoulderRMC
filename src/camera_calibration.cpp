#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

String window_name = "Chessboard";

void detectAndDisplay( Mat frame , float *distance);
void drawImage(Mat frame, vector<Point> corners, vector<Point2d> imgpts);
void saveCameraParams( string s, Size imageSize, Mat cameraMatrix, Mat distCoeffs,
                              vector<Mat> rvecs, vector<Mat> tvecs);


vector<vector<Point2f> > imagePoints;



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher posePublish;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/chessboard/output_video", 1);

    posePublish = nh_.advertise<std_msgs::Float64>("/chessboard/distance", 1);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
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

    float distance = -1;
    detectAndDisplay(cv_ptr->image, &distance);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

void detectAndDisplay( Mat frame , float *distance)
{
  imshow( window_name, frame );
  waitKey(3);
  Size boardSize(9,6);
  vector<Point2f> corners;

	vector<Mat> rvec;// = Mat(Size(3,1), CV_64F);
	vector<Mat> tvec;// = Mat(Size(3,1), CV_64F);

  vector<vector<Point3f> > boardPoints(1);
  Mat intrinsics=  Mat::zeros(8, 1, CV_64F), distortion =  Mat::zeros(8, 1, CV_64F);

  for (int i=0; i<9; i++)
	{
		for (int j=0; j<6; j++)
		{
			boardPoints[0].push_back( Point3f( double(i), double(j), 0.0) );
		}
	}

  bool found = findChessboardCorners(frame, boardSize, corners, CALIB_CB_FAST_CHECK);
  if(found){
    sleep(2);
    printf("-------------BEGIN------------\n");
    /*
    for(int i = 0; i < corners.size(); i++){
      printf("Point: %f, %f \n", corners[i].x, corners[i].y);
    }
    for(int i = 0; i < boardPoints.size(); i++){
      printf("Board Point: %f, %f \n", boardPoints[i].x, boardPoints[i].y);
    }*/

    printf("Recorded %d images\n", (int)imagePoints.size());
    printf("--------------END-------------\n\n\n\n\n");

    imagePoints.push_back(corners);

    if(imagePoints.size() >= 10){
      boardPoints.resize(imagePoints.size(), boardPoints[0]);

      calibrateCamera(boardPoints, imagePoints, frame.size(),  intrinsics, distortion, rvec, tvec);
      saveCameraParams("cameraSettings.yml", frame.size(), intrinsics, distortion, rvec, tvec);
      exit(0);
    }

    Size imageSize(frame.rows, frame.cols);
  }else{
    printf("No Chessboard here \n");
  }
}

// Print camera parameters to the output file
void saveCameraParams( string filename, Size imageSize, Mat cameraMatrix, Mat distCoeffs, vector<Mat> rvecs, vector<Mat> tvecs)
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty())
        fs << "nrOfFrames" << (int)rvecs.size();
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << 9;
    fs << "board_Height" << 6;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;
    fs.release();
}
