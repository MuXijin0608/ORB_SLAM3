/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include <sstream>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "eigen3/Eigen/Dense"
#include<opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include"../../../include/System.h"

#include<tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include"../../../include/System.h"
#include "../../../include/Converter.h"

using namespace std;


//全局变量区
//连续次数下限 处理轨迹起始点漂移问题
const int lmtContinue = 10;
int cnt = lmtContinue;
int cnt2 = 0;
ros::Publisher pose_pub;
ros::Publisher mono_path_pub;
ros::Publisher feedback_pub;
nav_msgs::Path mono_path;

//定义 声明
bool operator ! (const cv::Mat&m) { return m.empty();}


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    ORB_SLAM3::System* mpSLAM;
};

class TextGrabber{
   public: 
    TextGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
    ORB_SLAM3::System* mpSLAM;
    void GrabText(const std_msgs::String::ConstPtr& msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    TextGrabber tgb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber subControl = nodeHandler.subscribe("/Control",1,&TextGrabber::GrabText,&tgb);

    pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/ORBSLAM/pose", 10);//数字是队列缓冲大小
    mono_path_pub = nodeHandler.advertise<nav_msgs::Path>("/ORBSLAM/path", 10);//
    feedback_pub = nodeHandler.advertise<std_msgs::String>("/ORBSLAM/feedback", 5);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void TextGrabber::GrabText(const std_msgs::String::ConstPtr& msg)
{
    const char * instruct = msg->data.c_str();
    // ROS_INFO(" heard: [%s]", instruct); //打印接收到的信息
    
    //提前定义好的数据字典
    if(strcmp(instruct,"save_map") == 0){
        // mpSLAM->SaveAtlas(1);
        ROS_INFO("接收到保存信息 执行保存");
        // std::stringstream ss;
        // std_msgs::String feed_msg;
        // ss << "执行保存地图指令";
        std_msgs::String feed_msg;
        const char * s = "执行保存地图指令";
        feed_msg.data = s;
        feedback_pub.publish(feed_msg);
    }
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    

    // cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    int mState = mpSLAM->GetTrackingState();
    //将状态返回到feedback
    if(cnt2 == 0){
        std::stringstream ss;
        std_msgs::String feed_msg;
        // ss << mState;
        ss << mState << " 预留的反馈信息";
        // ss << mState << " some info" <<endl;
        feed_msg.data = ss.str();
        feedback_pub.publish(feed_msg);
    }
    cnt2 = (cnt2 + 1) % 50;

    // cv::Mat Tcw;
    // Sophus::SE3f Tcw_SE3f=mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    // Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    // cv::eigen2cv(Tcw_Matrix, Tcw);
    
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    if(!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);
        pose.pose.position.x = - twc.at<float>(0);
        pose.pose.position.y = -twc.at<float>(2);
        pose.pose.position.z = twc.at<float>(1);
        pose.pose.orientation.x = q[0];
        pose.pose.orientation.y = q[1];
        pose.pose.orientation.z = q[2];
        pose.pose.orientation.w = q[3];
        pose_pub.publish(pose);

        //找到定位后 连续lmtContinue次以上才publish路径 否则重新计算
        if(mState == 2){
            if(cnt>0) cnt--;
            else{
                mono_path.header.frame_id="map";
                mono_path.header.stamp=ros::Time::now();
                mono_path.poses.push_back(pose);
                //publish路径
                mono_path_pub.publish(mono_path);
            }
            
        }
        else cnt=lmtContinue;

    }else{
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pose_pub.publish(pose);

    }

}


