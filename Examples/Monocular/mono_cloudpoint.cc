#include<algorithm>

#include<chrono>

#include<opencv2/core/core.hpp>


#include<iostream>
#include<fstream>
#include <sstream>
#include <iomanip>

#include "CloudPoint.h"

using namespace std;

int main(int argc, char **argv)
{  

    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_cloudpoints path_to_vocabulary path_to_settings img_folder keyframeTrajectory" << endl;
        return 1;
    }

    cout << "image directory: " << argv[3] << endl;
    string imgDir = argv[3];
    string keyFrameFile = argv[4];

    ORB_SLAM3::CloudPoint system(argv[1],argv[2]);

    // Main loop
    cv::Mat im;

    ifstream f;
    f.open(keyFrameFile.c_str());

    cout << "start building cloud points" << endl;
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
            float pose[7];
            for (int i = 0; i < 7; i++)
            {
                getline(iss, token, ' ');
                pose[i] = std::stof(token);
            }

            im = cv::imread(imgPath,cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);
            if(im.empty())
            {
                //cerr << endl << "Failed to load image at: " <<  imgPath << endl;
                cout<< endl << "Failed to load image at: " <<  imgPath << endl;
                continue; 
            }
            
            double tframe = imgname;
            system.Triangulate(im, pose, tframe);
        }
    }

    cout << "finish building cloud points." << endl;
    cin.get();

    system.shutdown();
    return 0;
}

