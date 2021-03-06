/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<signal.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"
#include "../include/Kalman.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

using namespace std;

ORB_SLAM3::System* SLAM;
bool LOOP = true;
string kf_file = "kf_traj.txt";

Kalman kalmanX(0.0f, 0.003f, 0.03f), kalmanY(0.0f, 0.003f, 0.03f);
float last_frame_timestmap = 0.0f;
float kalAngleX = 0.0f, kalAngleY = 0.0f;
int cnt = 0;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe, ros::Publisher&pub, ros::Publisher&pub2): 
    mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe), marker_pub(pub), pcl_pub(pub2){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void publish();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    ros::Publisher marker_pub;
    ros::Publisher pcl_pub;
};

void SlamShutDown(int sig)
{
    SLAM->Shutdown();
    SLAM->SaveKeyFrameTrajectoryTUM(kf_file);
    SLAM->saveKeyFrameAndMapPoints("temp.txt");
    LOOP = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 6)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [kf_file]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);

  if(argc==5)
  {
      kf_file = argv[4];
       kf_file = "kf_" + kf_file +".txt";
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  SLAM = new ORB_SLAM3::System(argv[1],argv[2],ORB_SLAM3::System::STEREO,false);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  ros::Publisher pcl_pub = n.advertise<PointCloud>("pcl", 100);

  ImuGrabber imugb;
  ImageGrabber igb(SLAM,&imugb,sbRect == "true",bEqual,marker_pub,pcl_pub);
  
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  signal(SIGINT, SlamShutDown);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(LOOP)
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());

      // int mask = int(imLeft.rows * 0.65f);
      // cv::Mat pRoi = imLeft(cv::Rect(0, mask, imLeft.cols, imLeft.rows - mask));
      // pRoi.setTo(cv::Scalar(0, 0, 0));

      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      
      // pRoi = imRight(cv::Rect(0, mask, imLeft.cols, imLeft.rows - mask));
      // pRoi.setTo(cv::Scalar(0, 0, 0));

      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

          // Kalman filter
          if (last_frame_timestmap == 0.0f)
              ;
          else {
              float dt = last_frame_timestmap - t;

              // accelerometer output Gp = [Gpx, Gpy, Gpz] = R(g - a)
              //                         = Rxyz * [0, 0, 1]
              //                         = [-cos(z)sin(y),
              //                            cos(y)sin(x) + cos(y)sin(z)sin(x),
              //                            cos(x)cos(y) - sin(y)sin(x)sin(z)]
              // z = 0
              // Normailze(Gp) = [-sin(y), cos(y)sin(x), cos(y)cos(x)]

              // y
              float pitch = asin(-acc.x / sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z)) / CV_PI * 180;
              // x
              float roll = atan2(acc.y, acc.z) / CV_PI * 180;

              // cout << pitch << " " << roll << endl;

              kalmanY.filter(pitch, gyr.y, dt);
              kalmanX.filter(roll, gyr.x, dt);

              gyr.y -= kalmanY.getQbias();
              gyr.x -= kalmanX.getQbias();
          }

          last_frame_timestmap = t;

          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imLeft,imLeft);
        mClahe->apply(imRight,imRight);
      }

      if(do_rectify)
      {
        cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
      }

      mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
      
      cnt++;
      if(cnt % 20 == 0)
        publish();

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }

  delete SLAM;
  ros::shutdown();
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void ImageGrabber::publish()
{
    if (ros::ok()) {
        vector<cv::Point3f> position;
        vector<vector<float>> orientation;
        vector<cv::Point3f> pointss;
        vector<cv::Point3f> currentPoints;
        mpSLAM->getPulishData(position, orientation, pointss, currentPoints);

        visualization_msgs::Marker line_strip;
        PointCloud::Ptr allPointsMsg(new PointCloud);
        PointCloud::Ptr currentPointsMsg(new PointCloud);

        line_strip.header.frame_id = allPointsMsg->header.frame_id = currentPointsMsg->header.frame_id = "/traj";

        ros::Time time_st = ros::Time::now();
        line_strip.header.stamp = time_st;
        pcl_conversions::toPCL(time_st, allPointsMsg->header.stamp);
        pcl_conversions::toPCL(time_st, currentPointsMsg->header.stamp);

        line_strip.ns = "keyframe_trajectory";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = 0;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        line_strip.points.reserve(position.size());

        // Create the vertices for the points and lines
        for (auto& i : position) {
            geometry_msgs::Point p;
            p.x = i.x;
            p.y = i.y;
            p.z = i.z;
            
            line_strip.points.push_back(p);
        }

        for (auto &i : pointss)
        {
          pcl::PointXYZRGB Point;
          Point.x = i.x;
          Point.y = i.y;
          Point.z = i.z;
          Point.r = 255;
          Point.g = 255;
          Point.b = 255;
          allPointsMsg->points.push_back(Point);
        }
        
        for (auto &i : currentPoints)
        {
          pcl::PointXYZRGB Point;
          Point.x = i.x;
          Point.y = i.y;
          Point.z = i.z;
          Point.r = 0;
          Point.g = 255;
          Point.b = 0;
          allPointsMsg->points.push_back(Point);
        }

        // cout << allPointsMsg->points.size() << endl;
        if(pointss.size() != 0)
          pcl_pub.publish(allPointsMsg);
        marker_pub.publish(line_strip);
    }
}