/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FramePublisher.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<boost/thread.hpp>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<iostream>


/* 定义 ORB_SLAM::FramePublisher 类
************************************************
commented by hualong 2019/1/8
ORB_SLAM::FramePublisher 用于在tracking的过程中，将当前帧在运行中的可视化结果发布给ROS

FramePublisher::FramePublisher()             创建一个ORB_SLAM::FramePublisher实例, 用于发布ORB_SLAM/Frame 话题到ROS
FramePublisher::Update(Tracking *pTracker)   根据Tracking，提供FramePublisher更新的当前帧的信息 
FramePublisher::Refresh()                    根据FramePublisher::Update，当有更新时，处理当前帧的，发布话题
FramePublisher::SetMap(Map *pMap)             

cv::Mat FramePublisher::DrawFrame()          从Update获取当前帧信息，然后根据当前的tracking状态，对当前帧的图像处理，添加相应的关键点和标题
FramePublisher::DrawTextInfo                 根据当前的追踪状态，给输出的图片底部加入标题
FramePublisher::PublishFrame()               通过调用DrawFrame，DrawTextInfo， 发布经过添加特征点和标题字串的图片话题 ORB_SLAM/Frame       
************************************************
*/


namespace ORB_SLAM
{

/*
*************************
 创建一个ORB_SLAM::FramePublisher实例, 用于发布ORB_SLAM/Frame 话题到ROS
 - 先初始化图片大小(VGA),话题名称，发布频率
 - 调用DrawFrame、DrawTextInfo 对图像进行处理
 - 调用PublishFrame最终发布话题
 *************************
*/
FramePublisher::FramePublisher()
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mbUpdated = true;

    mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM/Frame",10,true);

    PublishFrame();
}


void FramePublisher::SetMap(Map *pMap)
{
    mpMap = pMap;
}

//根据FramePublisher::Update，当有更新时，处理当前帧的，发布话题
void FramePublisher::Refresh()
{
    if(mbUpdated)
    {
        PublishFrame();
        mbUpdated = false;
    }
}




/*
*************************
 根据当前的tracking状态，对当前帧的图像处理，添加相应的关键点和标题
 param@ 输入 mState状态/ mIm当前帧/ mvIniKeys mvCurrentKeys关键点/ mvIniMatches匹配点/ mvpMatchedMapPoints路标点/
 - 先通过mState当前状态，决定该在mIm当前帧添加什么信息， 然后用cv::line、cv::rectangle、cv::circle添加信息 -->cv::Mat im
 - 再通过mState状态，给图上添加相应的标题字串 -->  DrawTextInfo-->  cv::Mat imWithInfo;
 *************************
*/
cv::Mat FramePublisher::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<MapPoint*> vMatchedMapPoints; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variable to be used within scoped mutex
    {
        boost::mutex::scoped_lock lock(mMutex);
        //获取tracking的状态，然后根据不同的状态，进行处理
        state=mState;
        //tracking -system not ready/no image yet
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;
        //获取到图像***************
        mIm.copyTo(im);
        //tracking - not initialized  还没有初始化所以只有参考帧
        if(mState==Tracking::NOT_INITIALIZED)
        {            
            vIniKeys = mvIniKeys; //参考帧的关键点
        }
        //tracking - initializing  初始化后，所以有参考帧与当前帧
        else if(mState==Tracking::INITIALIZING)
        {
            vCurrentKeys = mvCurrentKeys;//当前帧的关键帧
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        //tracking - working  working  当前帧与地图点
        else if(mState==Tracking::WORKING)
        {
            vCurrentKeys = mvCurrentKeys;
            vMatchedMapPoints = mvpMatchedMapPoints; //当前帧追踪到到的地图点
        }
        //tracking - lost  lostz状态，所以只剩下当前帧的关键点了
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release

    if(im.channels()<3)
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw  initalizing-画出参考帧与当前帧相对应的线
    if(state==Tracking::INITIALIZING) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(255,0,0)); // 初始化，画对应的线(BGR)
            }
        }        
    }
    // working 当前帧追踪到到的地图点画出矩形框
    else if(state==Tracking::WORKING) //TRACKING
    {
        mnTracked=0;
        const float r = 5;
        for(unsigned int i=0;i<vMatchedMapPoints.size();i++)
        {
            if(vMatchedMapPoints[i] || mvbOutliers[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                if(!mvbOutliers[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,0,255));//矩形框
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
                    mnTracked++;
                }
            }
        }

    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    //返回经过DrawFrame 关键点的处理、DrawTextInfo加入标题后的图像 cv::mat
    return imWithInfo;
}


/*
*************************
 发布经过添加特征点和标题字串的图片话题 ORB_SLAM/Frame
 param@ DrawFrame()转换后他图像
 param@ 发布频率10hz
 *************************
*/
void FramePublisher::PublishFrame()
{
    cv::Mat im = DrawFrame(); 
    cv_bridge::CvImage rosImage; //cv_image --> ros_image
    rosImage.image = im.clone();
    rosImage.header.stamp = ros::Time::now();
    rosImage.encoding = "bgr8";

    mImagePub.publish(rosImage.toImageMsg());
    ros::spinOnce();
}



/*
*************************
 根据当前的追踪状态，给输出的图片底部加入标题
 param@ cv::Mat &im 输入的图片
 param@ int nState 输入的状态
 param@ cv::Mat &imText 要输出带标题的图片
*************************
*/
void FramePublisher::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " NOT INITIALIZED ";
    else if(nState==Tracking::INITIALIZING)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::WORKING)
    {
        s << " TRACKING ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << " - KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
        std::cout << s.str() << std::endl;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

    //调试 将ORB_SLAM/Frame的图片保存到本地
    // static int count = 0;
    // stringstream name;
    // name << "/home/hualong/Pictures/" << count <<".jpg";
    // cv::imwrite(name.str(), imText);
    // count += 1;

}



/*
*************************
 提供 FramePublisher更新的当前帧的信息 
 param@ mIm图片/mvCurrentKeys关键点/mvpMatchedMapPoints路标点/mvbOutliers关联的标志位/ 以及初始化状态下的前一帧的关键点，及匹配点
 param@ int nState 输入的状态
 
*************************
*/
void FramePublisher::Update(Tracking *pTracker)
{
    boost::mutex::scoped_lock lock(mMutex);
    pTracker->mCurrentFrame.im.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvpMatchedMapPoints=pTracker->mCurrentFrame.mvpMapPoints;
    mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;

    if(pTracker->mLastProcessedState==Tracking::INITIALIZING)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);

    mbUpdated=true;
}

} //namespace ORB_SLAM
