#include<algorithm>

#include<chrono>


#include<iostream>
#include<fstream>
#include <sstream>
#include <iomanip>
#include<string>

#include<thread>
#include <condition_variable>
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include<vector>

#include "Eigen/Core"
#include "sophus/se3.hpp"

#include "Settings.h"
#include "GeometricCamera.h"

#include <include/CameraModels/Pinhole.h>

using namespace std;
using namespace ORB_SLAM3;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


void UndistortKeyPoints(ORB_SLAM3::GeometricCamera* mpCamera, cv::Mat& mDistCoef, cv::Mat& mK, std::vector<cv::KeyPoint>& mvKeys, std::vector<cv::KeyPoint>& mvKeysUn);


int main(int argc, char **argv)
{  
    cout << "argc: " << argc << endl;
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_depth path_to_settings img_folder depth_folder keyframeTrajectory" << endl;
        return 1;
    }

    string strSettingsFile = argv[1];
    string imgDir = argv[2];
    string depDir = argv[3];
    string keyFrameFile = argv[4];

    cout << "setting path: " << strSettingsFile << endl;
    cout << "image directory: " << imgDir << endl;
    cout << "depth directory: " << depDir << endl;
    cout << "keyFrameFile directory: " << keyFrameFile << endl; 

    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    ORB_SLAM3::GeometricCamera* mpCamera;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    int sensor = 0;
    ORB_SLAM3::Settings* settings_ = new Settings(strSettingsFile, sensor);
    cout << (*settings_) << endl;

    mpCamera = settings_->camera1();

    if(settings_->needToUndistort()){
        mDistCoef = settings_->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }

    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = mpCamera->getParameter(0);
    mK.at<float>(1,1) = mpCamera->getParameter(1);
    mK.at<float>(0,2) = mpCamera->getParameter(2);
    mK.at<float>(1,2) = mpCamera->getParameter(3);


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    cout << "start building cloud points" << endl;
    ifstream f;
    f.open(keyFrameFile.c_str());
    cv::Mat im;
    cv::Mat imD;
    
    int line = 0;
    while (!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss; 
            ss << s;
            //cout << ss.str() << endl;

            istringstream iss(s);   // 输入流
            string token;           // 接收缓冲区
            getline(iss, token, ' ');
            long imgname = std::stol(token);

            string imgPath = imgDir + "/" + std::to_string(imgname) + ".png";
            //cout << imgPath << " " << cloud->points.size() << endl;
            string depPath = depDir + "/" + std::to_string(imgname) + ".png";
            float pose[7];
            for (int i = 0; i < 7; i++)
            {
                getline(iss, token, ' ');
                pose[i] = std::stof(token);
            }

            Eigen::Matrix<float,3,1> t;
            t << pose[0], pose[1], pose[2];
            Eigen::Quaternionf q(pose[6], pose[3], pose[4], pose[5]); // w, x, y, z

            Sophus::SE3<float> Twc = Sophus::SE3<float>(q,t); //Twc

            im = cv::imread(imgPath,cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);
            if(im.empty())
            {
                //cerr << endl << "Failed to load image at: " <<  imgPath << endl;
                cout<< endl << "Failed to load image at: " <<  imgPath << endl;
                continue; 
            }

            imD = cv::imread(depPath, cv::IMREAD_UNCHANGED);
            if(imD.empty())
            {
                cout<< endl << "Failed to load image at: " <<  imgPath << endl;
                continue; 
            }

            std::vector<cv::KeyPoint> mvKeys;
            std::vector<cv::KeyPoint> mvKeysUn;
            int stride = 8;
            for (int i = 0; i < im.rows; i += stride)
            {
                for (int j = 0; j < im.cols; j+= stride)
                {
                    cv::KeyPoint kp;
                    kp.pt.x = i;
                    kp.pt.y = j;
                    mvKeys.push_back(kp);
                }
            }

            UndistortKeyPoints(mpCamera, mDistCoef, mK, mvKeys, mvKeysUn);

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            for (int i = 0; i < im.rows; i += stride)
            {
                for (int j = 0; j < im.cols; j += stride)
                {
                    
                    int ii = i / stride;
                    int jj = j / stride;
                    
                    //cv::KeyPoint kp = mvKeysUn[i * im.cols + j ];
                    cv::KeyPoint kp = mvKeysUn[ii * ((im.cols-1) / stride + 1) + jj ];
                    float x = (kp.pt.x - mK.at<float>(0,2)) / mK.at<float>(0,0);
                    float y = (kp.pt.y - mK.at<float>(1,2)) / mK.at<float>(1,1);

                    //kp = mvKeys[i * im.cols + j];
                    float z = (float) imD.at<uchar>(i, j) / 25.0;

                    if (z < 0.01)
                        continue;

                    Eigen::Vector3f pc(x * z, y * z, z);

                    Eigen::Vector3f pw = Twc.rotationMatrix() * pc + Twc.translation();

                    PointT p;
            
                    p.x = pw(0);
                    p.y = pw(1);
                    p.z = pw(2);

                    p.b = im.ptr<uchar>(i)[j*3];
                    p.g = im.ptr<uchar>(i)[j*3+1];
                    p.r = im.ptr<uchar>(i)[j*3+2];

                    cloud->points.push_back(p);
                }
            }

            cloud->is_dense = false;

            cloud->width = cloud->points.size();
            cloud->height = 1;

            viewer->addPointCloud<PointT> (cloud, "sample cloud" + std::to_string(line));

            //viewer->setCameraPosition(pose[0], pose[1], pose[2], 0, 0, 0);
            //viewer->initCameraParameters ();
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" + std::to_string(line));
            viewer->spinOnce ();
            pcl_sleep(0.1);
            //break;
            line += 1;
        }

    }

    cout << "end building cloud points" << endl;

    while (!viewer->wasStopped ()) 
    {   
        viewer->spinOnce (); 
        pcl_sleep (0.01);
    }   

    //pcl::io::savePLYFileBinary("cloudpoints.ply", *cloud);

    //pcl::io::savePCDFileASCII ("cloudpoint_pcd_from_depth.pcd", *cloud);

    
    cin.get();
    return 0;
}



void UndistortKeyPoints(ORB_SLAM3::GeometricCamera* mpCamera, cv::Mat& mDistCoef, cv::Mat& mK, std::vector<cv::KeyPoint>& mvKeys, std::vector<cv::KeyPoint>& mvKeysUn)
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    int N = mvKeys.size();
    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}
