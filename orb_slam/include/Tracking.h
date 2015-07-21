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
#include <sensor_msgs/Range.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

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

#include <tf/transform_broadcaster.h>

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
    void GrabImages(const sensor_msgs::ImageConstPtr& msg,
                    const sensor_msgs::CameraInfoConstPtr& info);

    void GrabRange(const sensor_msgs::ImageConstPtr& msg,
                   const sensor_msgs::RangeConstPtr& range);

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
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization
    Initializer* mpInitializer;

    // False = images are not rectified, True = images rectified
    bool mRectified;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    //Publishers
    FramePublisher* mpFramePublisher;
    MapPublisher* mpMapPublisher;

    //Map
    Map* mpMap;

    //Camera information
    cv::Mat mK;
    cv::Mat mDistCoef;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Mutex
    boost::mutex mMutexForceRelocalisation;
    boost::mutex mMutexRange;

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

    // Range
    float mRange;
    double mRangeStamp;

    // Image sync
    image_transport::SubscriberFilter mImageSub, mImageSub2;
    message_filters::Subscriber<sensor_msgs::CameraInfo> mInfoSub;
    message_filters::Subscriber<sensor_msgs::Range> mRangeSub;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                      sensor_msgs::CameraInfo> policyImages;
    typedef message_filters::Synchronizer<policyImages> mPoliceSyncImages;
    boost::shared_ptr<mPoliceSyncImages> mSyncImages;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Range> policyRange;
    typedef message_filters::Synchronizer<policyRange> mPoliceSyncRange;
    boost::shared_ptr<mPoliceSyncRange> mSyncRange;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
