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

#ifndef OCTOMAPPUBLISHER_H
#define OCTOMAPPUBLISHER_H

#include<ros/ros.h>

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <octomap_msgs/GetOctomap.h>

#include "orb_slam/SaveOctomap.h"

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"

namespace ORB_SLAM
{

class OctoMapPublisher
{
public:
    OctoMapPublisher(Map* pMap);

    Map* mpMap;

    void RefreshMap();
    void UpdateOctoMapFromMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);

    void Publish();

    void reset();

    bool save(string filename);

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    bool octomapBinarySrv(octomap_msgs::GetOctomapRequest  &req,
                          octomap_msgs::GetOctomapResponse &res);

    bool octomapFullSrv(octomap_msgs::GetOctomapRequest  &req,
                        octomap_msgs::GetOctomapResponse &res);

    bool octomapSaveSrv(orb_slam::SaveOctomapRequest  &req,
                        orb_slam::SaveOctomapResponse &res);

    ros::NodeHandle nh;

    boost::mutex mMutexCamera;

    const char* MAP_FRAME_ID;

    octomap::ColorOcTree m_octoMap;

    ros::Publisher publisher;

    ros::ServiceServer m_octomapSaveService;
    ros::ServiceServer m_octomapBinaryService;
    ros::ServiceServer m_octomapFullService;
};

} //namespace ORB_SLAM

#endif // OCTOMAPPUBLISHER_H
