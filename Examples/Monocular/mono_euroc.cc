/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<iGPSTypes.h>
using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadKeyPointAndDescriptor(vector<vector<cv::Point2f>> &KeyPoint, vector<cv::Mat> &Descriptor );

void LoadiGPS(vector<Eigen::Matrix<double,6,1>> &iGPSPosition,vector<double> &vTimestampsiGPS);

int LoadiGPSDirection(ORB_SLAM3::iGPS::Direction* iGPSDirection,vector<double> &vTimestampsiGPS,const string &strSettingsFile);

int main(int argc, char **argv)
{  
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    ORB_SLAM3::iGPS::Direction* iGPSDirection = new ORB_SLAM3::iGPS::Direction[5000];
    vector<double> vTimestampsiGPSDir;
    int ret = LoadiGPSDirection(iGPSDirection,vTimestampsiGPSDir,argv[2]);

    if(1 == ret)
    {
        cout << "Read fsSettings fails";
        return 1;
    }
    else if(2 == ret)
    {
        cout<< "Read iGPSDataFile fails";
        return 1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);

    if(!vTimestampsiGPSDir.empty())
        SLAM.LoadiGPSDirection(vTimestampsiGPSDir,iGPSDirection);

    std::chrono::steady_clock::time_point test_start = std::chrono::steady_clock::now();

    for (seq = 0; seq<num_seq; seq++)
    {
        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED);
            //cv::resize(im,im,cv::Size(614,514));
            double tframe = vTimestampsCam[seq][ni];
            //cout << "frame time stamp  = " << tframe << endl;
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    SLAM.Shutdown();

    std::chrono::steady_clock::time_point test_end = std::chrono::steady_clock::now();
    double test_time= std::chrono::duration_cast<std::chrono::duration<double> >(test_end  - test_start).count();
    cout<< "test_time = " << test_time << endl;

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
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

//iGPSDirection - ChannelTransmitterTimeDirection
int LoadiGPSDirection(ORB_SLAM3::iGPS::Direction* iGPSDirection,vector<double> &vTimestampsiGPS,const string &strSettingsFile)
{
    int nChannel,nRotationSpeed;
    string nPath;
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if(fSettings.isOpened())
    {
        cv::FileNode node = fSettings["iGPS.Channel"];
        if(!node.empty() && node.isInt())
        {
            nChannel = node.operator int();
        }
        node = fSettings["iGPS.RotationSpeed"];
        if(!node.empty() && node.isInt())
        {
            nRotationSpeed = node.operator int();
        }
        node = fSettings["iGPS.Path"];
        if(!node.empty() && node.isString())
        {
            node>> nPath;
        }
    }
    else
        return 1;
    //cout << "nChannel = " << nChannel<< endl;
    //cout << "nRotationSpeed = "  << nRotationSpeed << endl;
    cout << "iGPS Data FilePath: " << nPath << endl;
    cout << "start loading igps" << endl;
    //vTimestampsiGPS.reserve(5000);
    ifstream fTimes;
    fTimes.open(nPath);
    if(!fTimes)
        return 2;
    long index = 0;

    while(!fTimes.eof()) {

        string s;
        getline(fTimes, s);
        if(s[0] == '#')
            continue;

        if (!s.empty())
        {
            stringstream ss;
            ss << s;

            double ch, trm, t, d1, d2, d3, a1, a2;  // Frame
            //ss >> ch; ss >> trm;
            ss >> t; ss >> d1; ss >> d2; ss >> d3; ss >> a1; ss >> a2;  //Euler Angle

            iGPSDirection[index].channel = nChannel;
            iGPSDirection[index].transmitter = nRotationSpeed;
            iGPSDirection[index].time = t;
            iGPSDirection[index].dir = Eigen::Vector3d(d1,d2,d3);
            iGPSDirection[index].dirAngle = Eigen::Vector2d(a1,a2);
            vTimestampsiGPS.push_back(t);
            //vTimestampsiGPS.push_back(t);
            //iGPSPosition.push_back(PositionCoord);
        }

        index++;

    }
    cout << "end read iGPS data" << endl;
    //for(int i = 0 ; i < 5000 ; i ++ )
    //cout << "iGPSDirection = " << iGPSDirection[i].channel << " "<< iGPSDirection[i].transmitter << " " << iGPSDirection[i].time << " " << iGPSDirection[i].dir.transpose() << " " << iGPSDirection[i].dirAngle.transpose() <<endl;
    return 0;
}