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


#ifndef CLOUDPOINT_H
#define CLOUDPOINT_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include <condition_variable>
#include<opencv2/core/core.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Settings.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "GeometricCamera.h"
#include "Converter.h"
#include "GeometricTools.h"
#include "ORBmatcher.h"

namespace ORB_SLAM3
{

class Settings;
class Frame;

class CloudPoint
{

typedef std::pair<int,int> Match;
public:
enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudPoint(const string &strVocFile, const string &strSettingsFile);

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    int Triangulate(const cv::Mat &im, const float pose[7], const double &timestamp);

    int TwoViewReconstruction(Frame& F1, Frame& F2, const vector<int> &vMatches12);

    void viewer();
    void shutdown();

private:

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    Settings* settings_;

    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;

    cv::Mat mImGray;
    cv::Mat mImRGB;

    bool mbRGB;
    bool mIsColor;

    GeometricCamera* mpCamera;
    ORBextractor* mpORBextractor;

    Frame mCurrentFrame;
    Frame mPrevFrame;
    bool mIsFirst;

    pcl::PointCloud<PointT>::Ptr globalMap;
    double resolution = 0.001;
    pcl::VoxelGrid<PointT>  voxel;
    //pcl::visualization::CloudViewer* viewer;
    condition_variable mCloudPointUpdated;
    mutex              mCloudPointUpdatedMutex;

    bool    shutDownFlag=false;
    mutex   shutDownMutex;

    //shared_ptr<thread>  viewerThread;
    std::thread* viewerThread;
    //std::thread* viewerThread;
    float mThDepth = 0; // not use for compatible

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
