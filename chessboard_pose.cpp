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
void drawImage(Mat frame, vector<Point2f> corners, vector<Point2d> imgpts);
void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles);

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
  //system("clear");
  Size boardSize(9,6);
  vector<Point2f> corners;

	Mat rvec = Mat(Size(3,1), CV_64FC1);
	Mat tvec = Mat(Size(3,1), CV_64FC1);

  vector<Point2d> imagePoints, imageOrigin;
  vector<Point2d> imageFramePoints;
  vector<Point3d> boardPoints;
  vector<Point3d> framePoints;
  Mat intrinsics, distortion;
  for (int i=0; i<6; i++)
	{
		for (int j=8; j>=0; j--)
		{
			boardPoints.push_back( Point3d( double(j), double(i), 0.0) );
		}
	}

    //generate points in the reference frame
  framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
  framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
  framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
  framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );

  bool found = findChessboardCorners(frame, boardSize, corners);
  if(found){
    sleep(1/10);
    drawChessboardCorners(frame, boardSize, Mat(corners), found);
    printf("-------------BEGIN------------\n");
    /*
    for(int i = 0; i < corners.size(); i++){
      printf("Point: %f, %f \n", corners[i].x, corners[i].y);
    }

    for(int i = 0; i < boardPoints.size(); i++){
      printf("Board Point: %f, %f \n", boardPoints[i].x, boardPoints[i].y);
    }*/
    intrinsics = Mat::eye(3, 3, CV_32F); // dummy camera matrix
    intrinsics.at<float>(0,0) = 400;
    intrinsics.at<float>(1,1) = 400;
    intrinsics.at<float>(0,2) = 640 / 2;
    intrinsics.at<float>(1,2) =  480 / 2;

    distortion = Mat::zeros(4,1, CV_64FC1);


    Size imageSize(frame.rows, frame.cols);
    //calibrateCamera(Mat(boardPoints), Mat(corners), frame.size(),  intrinsics, distortion, rvec, tvec);
    solvePnP( Mat(boardPoints), Mat(corners), intrinsics, distortion, rvec, tvec, false, CV_ITERATIVE);
    projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );
    Rodrigues(rvec, rvec);
    Vec3d eulerAngles;
    getEulerAngles(rvec, eulerAngles);
    cout << "Roll:" << eulerAngles[2] << "Pitch" << eulerAngles[0] << "Yaw:" << eulerAngles[1] << endl;
    drawImage(frame, corners, imageFramePoints);

    for(int i = 0; i< imageFramePoints.size(); i++){
      printf("Point: %f,%f\n", imageFramePoints[i].x, imageFramePoints[i].y);
    }
    cout << fixed << setprecision(2) << "rvec = ["
         << rvec.at<double>(0,0) << ", "
         << rvec.at<double>(1,0) << ", "
         << rvec.at<double>(2,0) << "] \t" << "tvec = ["
         << tvec.at<double>(0,0) << ", "
         << tvec.at<double>(1,0) << ", "
         << tvec.at<double>(2,0) << "]" << endl;

    double dist = tvec.at<double>(0,0)*tvec.at<double>(0,0)+tvec.at<double>(1,0)*tvec.at<double>(1,0)+tvec.at<double>(2,0)*tvec.at<double>(2,0);
    dist = sqrt(dist)*1.371;
    char value[100];
    sprintf(value, "Distance: %f", dist);

    putText(frame, value, cvPoint(30,30),
            FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);




  printf("--------------END-------------\n\n\n\n\n");
  }else{
    printf("No Chessboard here \n");
  }

    imshow( window_name, frame );
    waitKey(3);
}

void drawImage(Mat frame, vector<Point2f> corners, vector<Point2d> imgpts){
  Point corner = imgpts[0];
  line(frame, corner, imgpts[1], CV_RGB(255,0,0), 5);
  line(frame, corner, imgpts[2], CV_RGB(0,255,0), 5);
  line(frame, corner, imgpts[3], CV_RGB(0,0,255), 5);
}

void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){

    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}
