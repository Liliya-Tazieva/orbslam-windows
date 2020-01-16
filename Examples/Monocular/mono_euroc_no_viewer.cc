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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

class OpenCVDrawer
{
public:
	OpenCVDrawer() : newFrameInserted(false), stopped(false) {};
	void SetFrame(cv::Mat);
	void Run();
	void Stop() { stopped = true; }

private:
	cv::Mat frame;
	std::mutex mMutexFrame;
	bool newFrameInserted;
	bool stopped;
};

int main(int argc, char **argv)
{
	//Initialize input files paths
	string path_to_vocabulary = "../Vocabulary/ORBvoc.txt";
	string path_to_settings = "../Examples/Input-Mono/EuRoC.yaml";
	string path_to_image_folder = "../Examples/Input-Mono/EuRoC/V2_01_easy/cam0/data";
	string path_to_times_file = "../Examples/Input-Mono/EuRoC_TimeStamps/V201.txt";

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
	LoadImages(path_to_image_folder, path_to_times_file, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR);

	//OpenCV drawer
	OpenCVDrawer drawer;

	thread tDrawer = thread(&OpenCVDrawer::Run, &drawer);

    // Vector for tracking time statistics
    vector<long long> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

		drawer.SetFrame(SLAM.GetFrameDrawer()->DrawFrame());

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        long long ttrack= std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

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
	drawer.Stop();
	tDrawer.join();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	long long overall_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	for (int ni = 0; ni < nImages; ni++)
		cout << "TrackMonocular " << vTimesTrack[ni] << endl;

	cout << "Aferage fps " << 1000 / ((float)overall_duration / (float)nImages) << endl;

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

	system("pause");

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

void OpenCVDrawer::SetFrame(cv::Mat _frame)
{
	unique_lock<mutex> lock(mMutexFrame);
	frame = _frame.clone();
	newFrameInserted = true;
}

void OpenCVDrawer::Run()
{
	while (!stopped)
	{
		if (newFrameInserted)
		{
			unique_lock<mutex> lock(mMutexFrame);
			cv::imshow("Output FrameDrawer", frame);
			newFrameInserted = false;
			cv::waitKey(1);
		}
		else
		{
			usleep(1000);
		}
	}
}