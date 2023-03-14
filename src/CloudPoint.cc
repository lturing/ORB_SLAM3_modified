
#include "CloudPoint.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>


#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/make_shared.hpp>

namespace ORB_SLAM3
{

CloudPoint::CloudPoint(const string &strVocFile, const string &strSettingsFile)
{
    
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    settings_ = new Settings(strSettingsFile, ORB_SLAM3::CloudPoint::MONOCULAR);
    cout << (*settings_) << endl;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

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

    mK_.setIdentity();
    mK_(0,0) = mpCamera->getParameter(0);
    mK_(1,1) = mpCamera->getParameter(1);
    mK_(0,2) = mpCamera->getParameter(2);
    mK_(1,2) = mpCamera->getParameter(3);

    mbf = settings_->bf();

    //ORB parameters
    int nFeatures = settings_->nFeatures();
    int nLevels = settings_->nLevels();
    int fIniThFAST = settings_->initThFAST();
    int fMinThFAST = settings_->minThFAST();
    float fScaleFactor = settings_->scaleFactor();

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    
    mbRGB = settings_->rgb();
    mIsColor = settings_->isColor();

    mIsFirst = true;

    voxel.setLeafSize( resolution, resolution, resolution);

    //viewer = new pcl::visualization::CloudViewer("viewer");
    globalMap = std::make_shared<pcl::PointCloud<PointT>>();
    
    viewerThread = new thread(&ORB_SLAM3::CloudPoint::viewer, this);

    // 1200 * 800
    cv::namedWindow("matches", cv::WINDOW_NORMAL);
    cv::resizeWindow("matches", 600, 400);

}


int CloudPoint::Triangulate(const cv::Mat &im, const float pose[7], const double &timestamp)
{
    mImGray = im;
    im.copyTo(mImRGB);
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }
    else if (mImGray.channels() == 1)
    {
        cvtColor(mImGray, mImRGB, cv::COLOR_GRAY2RGB);
    }

    Eigen::Matrix<float,3,1> t;
    t << pose[0], pose[1], pose[2];
    Eigen::Quaternionf q(pose[6], pose[3], pose[4], pose[5]); // w, x, y, z

    Sophus::SE3<float> Twc = Sophus::SE3<float>(q,t); //Twc
    Sophus::SE3<float> Tcw = Twc.inverse();

    mCurrentFrame = Frame(mImGray,timestamp,mpORBextractor,mpVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    mCurrentFrame.SetPose(Tcw);
    mCurrentFrame.ComputeBoW();

    if (mIsFirst)
    {
        mPrevFrame = mCurrentFrame;
        mIsFirst = false;
        mImRGB.copyTo(mImPreRGB);
        return 0;

    }
    
    // Find correspondences
    ORBmatcher matcher(0.95,false);
    std::vector<cv::Point2f> vPrevMatched;
    vPrevMatched.resize(mPrevFrame.mvKeysUn.size());
    for(size_t i=0; i<mPrevFrame.mvKeysUn.size(); i++)
        vPrevMatched[i]=mPrevFrame.mvKeysUn[i].pt;

    std::vector<int> vMatches12;
    int nmatches = matcher.SearchForInitialization(mPrevFrame,mCurrentFrame,vPrevMatched,vMatches12,200);

    //cout << "point1: " << mPrevFrame.mvKeysUn.size() << ", point2: " << mCurrentFrame.mvKeysUn.size() <<  ", match number: " << nmatches << endl;
    int nGood = TwoViewReconstruction(mPrevFrame, mCurrentFrame, vMatches12);

    cout << "point1: " << mPrevFrame.mvKeysUn.size() << ", point2: " << mCurrentFrame.mvKeysUn.size() <<  ", match number: " << nmatches << ", triangle: " << nGood << endl;
    mPrevFrame = mCurrentFrame;
    mImRGB.copyTo(mImPreRGB);

    return nGood;

}


int CloudPoint::TwoViewReconstruction(Frame& F1, Frame& F2, const vector<int> &vMatches12)
{
    const vector<cv::KeyPoint> &vKeys1 = F1.mvKeysUn;
    const vector<cv::KeyPoint> &vKeys2 = F2.mvKeysUn;

    int nGood=0;
    // Calibration parameters
    const float fx = mK_(0,0);
    const float fy = mK_(1,1);
    const float cx = mK_(0,2);
    const float cy = mK_(1,2);

    float sigma = 1.0;
    float Sigma2 = sigma*sigma;
    float th2 = 8.0*Sigma2;

    // Camera 1 Projection Matrix K[I|0]
    Eigen::Matrix<float,3,4> P1;
    P1.setZero();
    P1.block<3,3>(0,0) = mK_;

    Eigen::Vector3f O1;
    O1.setZero();

    //Sophus::SE3<float> Tgw = mInitFrame.GetPose();
    Sophus::SE3<float> Tc1w = F1.GetPose();
    Sophus::SE3<float> Tc2w = F2.GetPose();

    Sophus::SE3<float> T21 = Tc2w * Tc1w.inverse();
    //Sophus::SE3<float> Tg1 = Tgw * Tc1w.inverse();

    Eigen::Matrix<float,3,3> R21 = T21.rotationMatrix();
    Eigen::Matrix<float,3,1> t21 = T21.translation();

    Eigen::Matrix<float,3,3> Rwc1 = Tc1w.inverse().rotationMatrix();
    Eigen::Matrix<float,3,1> twc1 = Tc1w.inverse().translation();

    //PointCloud::Ptr cloud( new PointCloud() );
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);


    //cout << R21 << " " << t21 << endl;
    // Camera 2 Projection Matrix K[R|t]
    Eigen::Matrix<float,3,4> P2;

    vector<cv::DMatch> matches;
    vector<cv::KeyPoint> nkeys1;
    vector<cv::KeyPoint> nkeys2;

    if (false)
    {
        Sophus::SE3f T21;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        std::vector<cv::Point3f> vP3D;
        if(mpCamera->ReconstructWithTwoViews(vKeys1,vKeys2,vMatches12,T21,vP3D,vbTriangulated))
        {
            Eigen::Matrix<float,3,3> R21 = T21.rotationMatrix();
            Eigen::Matrix<float,3,1> t21 = T21.translation();
            Eigen::Vector3f O2 = -R21.transpose() * t21;

            for(size_t i1=0, iend=vMatches12.size(); i1<iend;i1++)
            {
                if(vMatches12[i1]>=0 && vbTriangulated[i1])
                {
                    int i2 = vMatches12[i1];
                    const cv::KeyPoint &kp1 = vKeys1[i1];
                    const cv::KeyPoint &kp2 = vKeys2[i2];

                    Eigen::Vector3f p3dC1 = Eigen::Vector3f(vP3D[i1].x, vP3D[i1].y, vP3D[i1].z);
                    if (p3dC1(2) < 0)
                        continue;
                    
                    Eigen::Vector3f p3dC2 = R21 * p3dC1 + t21;
                    if (p3dC2(2) < 0)
                        continue;

                    // Check parallax
                    Eigen::Vector3f normal1 = p3dC1 - O1;
                    float dist1 = normal1.norm();

                    Eigen::Vector3f normal2 = p3dC1 - O2;
                    float dist2 = normal2.norm();

                    float cosParallax = normal1.dot(normal2) / (dist1*dist2);

                    Eigen::Vector3f p3dw = Rwc1 * p3dC1 + twc1;

                    if(cosParallax<0.99998)
                    {
                        //continue;
                        if (p3dw[2] < 0.01 || p3dw[2] > 10)
                            continue;
                        
                        PointT p;
                        
                        p.x = p3dw(0);
                        p.y = p3dw(1);
                        p.z = p3dw(2);

                        int m = kp1.pt.x;
                        int n = kp1.pt.y;

                        p.b = mImRGB.ptr<uchar>(m)[n*3];
                        p.g = mImRGB.ptr<uchar>(m)[n*3+1];
                        p.r = mImRGB.ptr<uchar>(m)[n*3+2];

                        /*
                        if (mImRGB.channels() == 3)
                        {
                            p.b = mImRGB.ptr<uchar>(m)[n*3];
                            p.g = mImRGB.ptr<uchar>(m)[n*3+1];
                            p.r = mImRGB.ptr<uchar>(m)[n*3+2];

                        }
                        else 
                        {
                            p.r = mImGray.at<uchar>(cvRound(m), cvRound(n));
                            p.g = 0;//mImRGB.ptr<uchar>(m)[n*3+1];
                            p.b = 0;//mImRGB.ptr<uchar>(m)[n*3+2];
                        }
                        */

                        cloud->points.push_back(p);

                        nGood++;

                        cv::DMatch mm;
                        mm.queryIdx = nkeys1.size();
                        mm.trainIdx = nkeys1.size();
                        matches.push_back(mm);
                        nkeys1.push_back(F1.mvKeys[i1]);
                        nkeys2.push_back(F2.mvKeys[i2]);
                    }
            

                }
            }

        }
    }
    else 
    {
        P2.block<3,3>(0,0) = R21;
        P2.block<3,1>(0,3) = t21;
    
        P2 = mK_ * P2;

        Eigen::Vector3f O2 = -R21.transpose() * t21;

        //cout << "vMatches12.size() = " << vMatches12.size() << ", vKeys1.size()=" << vKeys1.size() << ", vKeys2=" << vKeys2.size() << endl;
        //globalMap->clear();
        
        int cnt1 = 0;
        int cnt2 = 0;
        int cnt3 = 0;
        int cnt4 = 0;
        int cnt5 = 0;

        for(size_t i1=0, iend=vMatches12.size();i1<iend;i1++)
        {

            /*
            // add 极限约束
            if(!vbMatchesInliers[i])
                continue;
            */

            int i2 = vMatches12[i1];
            if (i2 < 0)
                continue;
                    
            const cv::KeyPoint &kp1 = vKeys1[i1];
            const cv::KeyPoint &kp2 = vKeys2[i2];

            Eigen::Vector3f p3dC1;
            Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
            Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

            bool flag = GeometricTools::Triangulate(x_p1, x_p2, P1, P2, p3dC1);
            if (!flag)
            {
                cnt1 += 1;
                continue;
            }
            if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
            {
                cnt2 += 1;
                continue;
            }

            // Check parallax
            Eigen::Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1(2)<=0) // && cosParallax<0.99998)
            {
                cnt3 += 1;
                continue;
            }

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            Eigen::Vector3f p3dC2 = R21 * p3dC1 + t21;

            Eigen::Vector3f p3dw = Rwc1 * p3dC1 + twc1;

            if(p3dC2(2)<=0) // && cosParallax<0.99998)
            {
                cnt4 += 1;
                continue;
            }

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1(2);
            im1x = fx*p3dC1(0)*invZ1+cx;
            im1y = fy*p3dC1(1)*invZ1+cy;

            float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if(squareError1>th2 && false) 
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2(2);
            im2x = fx*p3dC2(0)*invZ2+cx;
            im2y = fy*p3dC2(1)*invZ2+cy;

            float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if(squareError2>th2 && false) 
                continue;

            if(cosParallax<0.99998)
            {
                //continue;
                if (p3dw[2] < 0.01 || p3dw[2] > 10)
                    continue;
                
                PointT p;
                
                p.x = p3dw(0);
                p.y = p3dw(1);
                p.z = p3dw(2);

                int m = kp1.pt.x;
                int n = kp1.pt.y;

                p.b = mImRGB.ptr<uchar>(m)[n*3];
                p.g = mImRGB.ptr<uchar>(m)[n*3+1];
                p.r = mImRGB.ptr<uchar>(m)[n*3+2];

                /*
                if (mImRGB.channels() == 3)
                {
                    p.b = mImRGB.ptr<uchar>(m)[n*3];
                    p.g = mImRGB.ptr<uchar>(m)[n*3+1];
                    p.r = mImRGB.ptr<uchar>(m)[n*3+2];

                }
                else 
                {
                    p.r = mImGray.at<uchar>(cvRound(m), cvRound(n));
                    p.g = 0;//mImRGB.ptr<uchar>(m)[n*3+1];
                    p.b = 0;//mImRGB.ptr<uchar>(m)[n*3+2];
                }
                */

                cloud->points.push_back(p);

                nGood++;

                cv::DMatch mm;
                mm.queryIdx = nkeys1.size();
                mm.trainIdx = nkeys1.size();
                matches.push_back(mm);
                nkeys1.push_back(F1.mvKeys[i1]);
                nkeys2.push_back(F2.mvKeys[i2]);
            }
            
            else
            {
                cnt5 += 1;
            }
        }

        cout << "cnt1=" << cnt1 << ", cnt2=" << cnt2  << ", cnt3=" << cnt3 << ", cnt4=" << cnt4 << ", cnt5=" << cnt5 << endl;

    }
    
    if (cloud->points.size() > 0)
    {
        cloud->is_dense = false;
        *globalMap += *cloud;
        mCloudPointUpdated.notify_one();

        cv::Mat img_match;
        cv::drawMatches(mImPreRGB, nkeys1, mImRGB, nkeys2, matches, img_match);

        cv::imshow("matches", img_match);
        cv::waitKey(1);
    }

    return nGood;

}



void CloudPoint::viewer()
{
    //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::CloudViewer viewer("viewer");
    //long cnt = 0;
    //long num_points = 0;
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( mCloudPointUpdatedMutex );
            mCloudPointUpdated.wait( lck_keyframeUpdated );
        }

        {
            unique_lock<mutex>lck_globalMap(globalMapMutex);
            globalMap->width = globalMap->points.size();
            globalMap->height = 1;

            viewer.showCloud( globalMap );
            //num_points += globalMap->points.size();
            //viewer->addPointCloud<PointT> (globalMap, "sample cloud" + std::to_string(cnt));

        }
        //viewer->setCameraPosition(pose[0], pose[1], pose[2], 0, 0, 0);
        //viewer->initCameraParameters ();

        //viewer->spinOnce();
        //pcl_sleep(0.1);
        //cnt += 1;

    }

    //cout << "total " << num_points << " points" << endl;
    cout << "total " << globalMap->points.size() << " points" << endl;
    //pcl::io::savePCDFileASCII ("cloudpoint_pcd.pcd", *globalMap);

    //while (!viewer->wasStopped ()) 
    //{   
    //    viewer->spinOnce (); 
    //    pcl_sleep (0.01);
    //} 
}


void CloudPoint::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        mCloudPointUpdated.notify_one();
    }
    viewerThread->join();
}

} //namespace ORB_SLAM

