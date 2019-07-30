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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<boost/thread.hpp>

/* 定义 ORB_SLAM::FramePublisher 类
************************************************
commented by hualong 2019/1/8
ORB_SLAM::FramePublisher用于将ORBSLAM运行中的可视化结果发布给ROS

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

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalisation
   std::vector<KeyFrame*> DetectRelocalisationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  boost::mutex mMutex; //boost库中的独占式互斥量
};

} //namespace ORB_SLAM

#endif
