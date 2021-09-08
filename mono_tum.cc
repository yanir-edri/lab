/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include <Converter.h>
#include <System.h>
#include <ctello.h>
#include <unistd.h>
#include <signal.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <thread>

#include "opencv2/opencv.hpp"
using namespace std;
using namespace chrono;
using namespace cv;
using namespace ctello;
// Scan Rotation Angle
#define sra 30
// Scan Vertical Movement
#define svm 20

atomic<int> scanFinished(0);
atomic<int> droneLocalized(0);

void commandTello(Tello &tello, string com, int dur = 0) {
    cout << "sending command " << com << endl;
    tello.SendCommand(com);
    while (!tello.ReceiveResponse());
    if (dur > 0) usleep(dur*1000000);
}
void moveTello(Tello &tello, string dir, int x, int dur = 0) {
    commandTello(tello, dir + " " + to_string(x), dur);
}
void takeOff(Tello &tello) {
    tello.SendCommand("takeoff");
    sleep(1);
    while (!(tello.ReceiveResponse()));
}
void land(Tello &tello) { commandTello(tello, "land"); }
void scan(Tello &tello) {
    int angle = 0, v = sra;
    cin >> angle;
    scanFinished.store(1);
    while (angle < 360) {
        if (droneLocalized.load()) {
            moveTello(tello, "cw", v, 1);
            angle += v;
            v = (v + sra) / 2;
        } else {
            moveTello(tello, "ccw", v, 1);
            angle -= v;
            v = v / 2;
            if (v <= 10) v = 10;
        }
        cout << "angle is now " << angle << endl;
        moveTello(tello, "up", svm, 1);
        moveTello(tello, "down", svm, 1);
    }
    // for now, land immeidiatly
    land(tello);
    scanFinished.store(1);
}

void saveMap(ORB_SLAM2::System &SLAM) {
    cout << "saving map..." << endl;
    std::vector<ORB_SLAM2::MapPoint *> mapPoints =
        SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for (auto p : mapPoints) {
        if (p != NULL) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v =
                ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
        }
    }
    pointData.close();
    cout << "done" << endl;
}
cv::Mat positionToVector(cv::Mat m) {
    //as written in https://math.stackexchange.com/a/84202
    cv::Mat R = m(cv::Rect(0,0,3,3));
    cv::Mat T = m(cv::Rect(3,0,1,3));
    R = R.t();
    return -R*T;
}
//pair<double,double> linearFit()

void savePositions(vector<cv::Mat>& P)  {
    cout << "saving positions..." << endl;
    std::ofstream posStream("/tmp/dronePosition.csv");
    for(cv::Mat p : P) {
        posStream << p.at<float>(0,0) << ',' << p.at<float>(0,1) << ',' << p.at<float>(0,2) << endl;
    }
    posStream.close();
    cout << "done." << endl;
}

int ret = 0, finish = 0, scanCalled = 0;
cv::Mat im;
double t = 0;

void takePicture() {
    VideoCapture capture{"udp://0.0.0.0:11111?fifo_size=100000", cv::CAP_FFMPEG};
    if (!capture.isOpened()) {
        cerr << "BASA camera not opened" << endl;
        exit(1);
    }
    while (!finish) {
        capture >> im;
        ret = 1;
    }
}
int main(int argc, char **argv) {
   cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
    if (argc != 3) {
        cerr << endl
             << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    Tello tello;
    if (!tello.Bind()) return 0;
    tello.SendCommand("streamon");
    thread picture_thread(takePicture);
    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR,
                           true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;
    
    // Main loop
    //takeOff(tello);
    thread scan_thread;
    while (!ret) usleep(5000);
    int i = 0;
    cv::Mat trackRet;
    vector<cv::Mat> positions;
    while (!scanFinished.load()) {
        i++;
        if (i % 10 != 0) usleep(5000);
        if (im.empty()) {
            cerr << endl << "Failed to load image from camera :(" << endl;
            exit(1);
        }
        trackRet = SLAM.TrackMonocular(im, i);
        droneLocalized.store(!trackRet.empty());
        if (!trackRet.empty() && !scanCalled) {
            scan_thread = thread(scan, std::ref(tello));
            scanCalled = 1;
        }
        if(!trackRet.empty()) {
            cout << "writing position.." << endl;
            positions.push_back(positionToVector(trackRet));
        }
    }
    finish = 1;
    // Stop all threads
    saveMap(SLAM);
    savePositions(positions);
    SLAM.Shutdown();
    tello.SendCommand("streamoff");
    // imageChoice = -100;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
