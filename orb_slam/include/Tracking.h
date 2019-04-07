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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <posedetection_msgs/Feature0D.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "FramePublisher.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapPublisher.h"

#include<tf/transform_broadcaster.h>


namespace ORB_SLAM
{

class FramePublisher;
class Map;
class LocalMapping;
class LoopClosing;

class Tracking
{  

public:
    Tracking(ORBVocabulary* pVoc, FramePublisher* pFramePublisher, MapPublisher* pMapPublisher, Map* pMap, string strSettingPath);

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        INITIALIZING=2,
        WORKING=3,
        LOST=4
    };

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    // This is the main function of the Tracking Thread
    void Run();

    void ForceRelocalisation();

    eTrackingState mState;
    eTrackingState mLastProcessedState;    

    // Current Frame
    Frame mCurrentFrame;

    // Initialization Variables
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;


    void CheckResetByPublishers();


protected:
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageWithFeat(const sensor_msgs::ImageConstPtr& img_msg, 
                           const posedetection_msgs::Feature0D::ConstPtr& feat_msg);
    void Feat_CB(const posedetection_msgs::Feature0D::ConstPtr& feat_msg);

    void FirstInitialization();
    void Initialize();
    void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);

    void Reset();

    bool TrackPreviousFrame();
    bool TrackWithMotionModel();

    bool RelocalisationRequested();
    bool Relocalisation();    

    void UpdateReference();
    void UpdateReferencePoints();
    void UpdateReferenceKeyFrames();

    bool TrackLocalMap();
    void SearchReferencePointsInFrustum();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();


    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    std::shared_ptr<ORBextractor> mpORBextractor;
    std::shared_ptr<ORBextractor> mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization
    std::shared_ptr<Initializer> mpInitializer;

    //Local Map
    std::shared_ptr<KeyFrame> mpReferenceKF;
    std::vector<std::shared_ptr<KeyFrame>> mvpLocalKeyFrames;
    std::vector<std::shared_ptr<MapPoint>> mvpLocalMapPoints;

    //Publishers
    FramePublisher* mpFramePublisher;
    MapPublisher* mpMapPublisher;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    std::shared_ptr<KeyFrame> mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Mutex
    boost::mutex mMutexTrack;
    boost::mutex mMutexForceRelocalisation;

    //Reset
    bool mbPublisherStopped;
    bool mbReseting;
    boost::mutex mMutexReset;

    //Is relocalisation requested by an external thread? (loop closing)
    bool mbForceRelocalisation;

    //Motion Model
    bool mbMotionModel;
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    // Transfor broadcaster (for visualization in rviz)
    tf::TransformBroadcaster mTfBr;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
