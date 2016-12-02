/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ORB_SLAM2/System.h"

using namespace std;

void LoadImages(const string &strSequence, const string &sensorType,
		vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_robotcar path_to_vocabulary"
        	<< "path_to_settings  path_to_sequence  sensor_type"
		<< endl << endl
		<< "e.g.: ./devel/lib/orb_slam_2/mono_robotcar   "
		<< "src/ORB_SLAM2/orb_slam_2/Vocabulary/ORBvoc.txt.proto   "
		<< "~/catkin_ws/src/multiagent_orb/multiagent_orb/settings/"
		<< "orb_settings_robotcar_monocular.yaml  "
		<< "~/catkin_ws/datasets/robotcar-dataset/2014-07-14-15-16-36/"
		<< "   mono_left"
		<< endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::Sensor::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
    	// Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
            		<< vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 =
        		std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 =
        		std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 =
        		std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 =
        		std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>
						(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, const string &sensorType,
		vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + sensorType + ".timestamps";
    fTimes.open(strPathTimeFile.c_str());

	if (!fTimes.is_open())
	{
		cerr << endl <<"Failed to load timestamps file at: "
				<< strPathTimeFile << endl;
		exit(1);
	}

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s.substr(0,16);
            double t;
            ss >> t;
            vTimestamps.push_back(t*1e-6);
        }
    }

	string strPrefixLeft;
	if(sensorType == "stereo")
	{
		cout<< endl << "Using centre images of stereo camera." << endl;
		strPrefixLeft = strPathToSequence + sensorType +
							"/centre/undistort_images/";
	}
	else if (sensorType == "mono_left"|| "mono_right" || "mono_rear")
	{
		strPrefixLeft = strPathToSequence + sensorType +
							"/undistort_images/";
	}
	else
	{
		cerr << endl <<"Unknown sensor type." << endl;
				exit(1);
	}

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << std::setprecision(16) <<vTimestamps[i]*1e6;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
