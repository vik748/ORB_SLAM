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

#include "OctoMapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#define DEFAULT_OCTOMAP_RESOLUTION 0.1


namespace ORB_SLAM
{

using namespace octomap;

void OctoMapPublisher::reset(octomap::ColorOcTree& octoMap)
{
  octoMap.clear();
//  ParameterServer* ps = ParameterServer::instance();
//  m_octoMap.setClampingThresMin(ps->get<double>("octomap_clamping_min"));
//  m_octoMap.setClampingThresMax(ps->get<double>("octomap_clamping_max"));
//  m_octoMap.setResolution(ps->get<double>("octomap_resolution"));
//  m_octoMap.setOccupancyThres(ps->get<double>("octomap_occupancy_threshold"));
//  m_octoMap.setProbHit(ps->get<double>("octomap_prob_hit"));
//  m_octoMap.setProbMiss(ps->get<double>("octomap_prob_miss"));

}

bool OctoMapPublisher::octomapBinarySrv(octomap_msgs::GetOctomapRequest  &req,
                                        octomap_msgs::GetOctomapResponse &res)
{
  static octomap::ColorOcTree octoMap(DEFAULT_OCTOMAP_RESOLUTION);
  refreshMap(octoMap);
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = MAP_FRAME_ID;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(octoMap, res.map)){
    return false;
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctoMapPublisher::octomapFullSrv(octomap_msgs::GetOctomapRequest  &req,
                                      octomap_msgs::GetOctomapResponse &res)
{
  static octomap::ColorOcTree octoMap(DEFAULT_OCTOMAP_RESOLUTION);
  refreshMap(octoMap);
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = MAP_FRAME_ID;
  res.map.header.stamp = ros::Time::now();

  if (!octomap_msgs::fullMapToMsg(octoMap, res.map)){
    return false;
  }

  return true;
}

bool OctoMapPublisher::save(string filename)
{
  static octomap::ColorOcTree octoMap(DEFAULT_OCTOMAP_RESOLUTION);
  refreshMap(octoMap);

  filename.append(".ot");

  std::ofstream outfile(filename.c_str(), std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    ROS_INFO("Writing octomap to %s", filename.c_str());
    octoMap.write(outfile);
    outfile.close();
    ROS_INFO("color tree written %s", filename.c_str());
    return true;
  }
  else {
    ROS_INFO("could not open  %s for writing", filename.c_str());
    return false;
  }
}

bool OctoMapPublisher::octomapSaveSrv(orb_slam::SaveOctomapRequest  &req,
                                      orb_slam::SaveOctomapResponse &res)
{
  return save(req.filename);
}


OctoMapPublisher::OctoMapPublisher(Map* pMap):nh("~"), mpMap(pMap), MAP_FRAME_ID("/ORB_SLAM/World"), CAMERA_FRAME_ID("/ORB_SLAM/Camera") {

  //Configure Publisher
  publisherPCL = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
  publisherOctomapFull = nh.advertise<octomap_msgs::Octomap>("octomap_full", 10);
  publisherOctomapBinary = nh.advertise<octomap_msgs::Octomap>("octomap_binary", 10);

  //Configure Services
  m_octomapSaveService = nh.advertiseService("save_octomap", &OctoMapPublisher::octomapSaveSrv, this);
  m_octomapBinaryService = nh.advertiseService("octomap_binary", &OctoMapPublisher::octomapBinarySrv, this);
  m_octomapFullService = nh.advertiseService("octomap_full", &OctoMapPublisher::octomapFullSrv, this);
}

void OctoMapPublisher::publishPointCloud()
{
  static sensor_msgs::PointCloud2 msgPointCloud;
  //save computation if their is no subscriber
  if (publisherPCL.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
    vector<MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();
    mapPointsToPCL(vMapPoints, vRefMapPoints, pclCloud);
    pcl::toROSMsg(pclCloud, msgPointCloud);
    msgPointCloud.header.frame_id = CAMERA_FRAME_ID;
    msgPointCloud.header.stamp = ros::Time::now();
    msgPointCloud.header.seq++;
    publisherPCL.publish(msgPointCloud);

    //TODO publish only on update (maybe save last update time (or index) in mpMap
    //    if(mpMap->isMapUpdated())
    //    {
    //        mpMap->ResetUpdated();
    //    }
  }
}

void OctoMapPublisher::publishOctomap()
{
  if (publisherOctomapBinary.getNumSubscribers() > 0 || publisherOctomapFull.getNumSubscribers() > 0)
  {
    static octomap::ColorOcTree octoMap(DEFAULT_OCTOMAP_RESOLUTION);
    ros::Time now = ros::Time::now();
    refreshMap(octoMap);

    if (publisherOctomapBinary.getNumSubscribers() > 0)
    {
      static octomap_msgs::Octomap msgOctomapBinary;
      msgOctomapBinary.header.frame_id = MAP_FRAME_ID;
      msgOctomapBinary.header.stamp = now;
      if (octomap_msgs::binaryMapToMsg(octoMap, msgOctomapBinary))
      {
        publisherOctomapBinary.publish(msgOctomapBinary);
      }
    }
    if (publisherOctomapFull.getNumSubscribers() > 0)
    {
      static octomap_msgs::Octomap msgOctomapFull;
      msgOctomapFull.header.frame_id = MAP_FRAME_ID;
      msgOctomapFull.header.stamp = now;
      if (octomap_msgs::fullMapToMsg(octoMap, msgOctomapFull))
      {
        publisherOctomapFull.publish(msgOctomapFull);
      }
    }
  }
}

void OctoMapPublisher::Publish()
{
  publishPointCloud();

  publishOctomap();
}

void OctoMapPublisher::refreshMap(octomap::ColorOcTree& octoMap)
{
  reset(octoMap);
  vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();

  mapPointsToOctomap(vMapPoints, octoMap);
}

void OctoMapPublisher::mapPointsToOctomap(const vector<MapPoint*> &vpMPs, octomap::ColorOcTree& octoMap)
{
  Pointcloud pointCloud;

  for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
  {
    if(vpMPs[i]->isBad()){
        continue;
    }
    cv::Mat pos = vpMPs[i]->GetWorldPos();

    pointCloud.push_back(pos.at<float>(0), pos.at<float>(1),pos.at<float>(2));
  }

  octomap::point3d origin;

  try{

    tf::StampedTransform transform_in_target_frame;

    m_tf_listener.lookupTransform("ORB_base_link", CAMERA_FRAME_ID, ros::Time(0) , transform_in_target_frame);

    octomap::pose6d frame = octomap::poseTfToOctomap(transform_in_target_frame);

    //TODO test discretize=true
    //TODO potential option insert only current like used for point cloud and integrate octomap
    octoMap.insertPointCloud(pointCloud, origin, frame, -1, false, false);
  }catch(...){
    ROS_ERROR("TF not available");
    octoMap.insertPointCloud(pointCloud, origin, -1, false, false);
  }
}

void OctoMapPublisher::mapPointsToPCL(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs, pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
  set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
  {
    //this checks results in only current (local) mapping points are taking into account
    if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
    {
        continue;
    }
    cv::Mat pos = vpMPs[i]->GetWorldPos();

    pcl::PointXYZ point(pos.at<float>(0), pos.at<float>(1),pos.at<float>(2));
    pclCloud.points.push_back(point);
  }
  pclCloud.height = 1; //unstructured cloud
  pclCloud.width = pclCloud.points.size();
}

} //namespace ORB_SLAM
