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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;



void LoadImages(const string& datadir, vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{  

    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_inertial_mi path_to_vocabulary path_to_settings data_folder" << endl;
        return 1;
    }

    cout << "data directory: " << argv[3] << endl;

    // Load all sequences:
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;

    int nImages = 0 ;

    int tot_images = 0;
    // 0 ./mono_inertial_euroc 
    // 1 /home/spurs/cpp/ORB_SLAM3/Vocabulary/ORBvoc.txt 
    // 2 /home/spurs/cpp/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml 
    // 3 /home/spurs/dataset/MH_02_easy 
    // 4 /home/spurs/cpp/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt
    
    cout << "Loading images...";

    string datadir = argv[3];

    LoadImages(datadir, vstrImageFilenames, vTimestampsCam);
    cout << "LOADED!" << endl;

    nImages = vstrImageFilenames.size();


    if((nImages<=0))
    {
        cerr << "ERROR: Failed to load images or IMU." << endl;
        return 1;
    }




    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);

    // Main loop
    cv::Mat im;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                    <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();


        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }

    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveTrajectoryEuRoC("CameraTrajectory_mi.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory_mi.txt");
    
    cin.get();
    return 0;
}


void LoadImages(const string& datadir, vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    string strPathTimes = datadir + "/timestamp.txt";
    string strImagePath = datadir + "/img";

    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

