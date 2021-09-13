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

#include <iostream>
#include <math.h>
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
#include <complex>

#include "opencv2/opencv.hpp"
using namespace std;
using namespace chrono;
using namespace cv;
using namespace ctello;
// Scan Rotation Angle
#define sra 30
// Scan Vertical Movement
#define svm 20

#define DBG 1
#include "algo.cc"

atomic<int> scanFinished(0);
atomic<int> droneLocalized(0);

void commandTello(Tello &tello, string com, double dur = 0.0) {
    cout << "sending command " << com << endl;
    tello.SendCommand(com);
    while (!tello.ReceiveResponse());
    if (dur > epsilon) usleep(int(dur*1000000));

}
void moveTello(Tello &tello, string dir, int x, double dur = 0.0) {
    commandTello(tello, dir + " " + to_string(x), dur);
}
void takeOff(Tello &tello) {
    tello.SendCommand("takeoff");
    sleep(1);
    while (!(tello.ReceiveResponse()));
}
void land(Tello &tello) { commandTello(tello, "land"); }
p3d pos,rot;

void scan(Tello &tello) {
    int angle = 0, v = sra;
    cin >> angle;
    scanFinished.store(1);
    while (angle < 360 || !droneLocalized.load()) {
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
    //land(tello);
    //scanFinished.store(1);
}

ld findScale(Tello& tello) {
    cout << "findScale called" << endl;
    p3d cp = pos;
    int d = 0;
    while(d < 60) {
        moveTello(tello,"forward",20,2);
        d += 20;
        while(!droneLocalized.load()) usleep(5000);
    }
    p3d np = pos;
    moveTello(tello,"back",60,2);
    //scale*diff = how much to send tello
    //scale*diff = 60
    //scale = 60/diff
    cout << "found scale!" << endl;
    return abs(60/(np-cp).abs());
}
void moveToPoint(Tello& tello, p2d nt, ld scale);
void writeToFile(string filename, string content ){
    std::ofstream stream(filename);
    cout << "writing to file " << filename << " that " << content << endl;
    stream << content << endl;
    stream.close();
}
void secondThread(Tello& tello) {
    //scan(tello);
    cout << "finished scan" << endl;
    ld sc = findScale(tello);
    writeToFile("/tmp/scale_log.txt","scale is " + to_string(sc));
    moveToPoint(tello, {0.0,0.5},sc);
    int x;
    cin >> x;
    cout << "sleeping..." << endl;
    usleep(2000000);
    cout << "LEETSSS GOOOO" << endl;
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
    
}

cv::Mat op_v(cv::Mat m) {
    cv::Mat R = m(cv::Rect(0,0,3,3));
    cv::Mat T = m(cv::Rect(3,0,1,3));
    R = R.t();
    return -R*T;
}
p3d matToPoint(cv::Mat m) {
    ld x = ld(m.at<float>(0));
    ld y = ld(m.at<float>(1));
    ld z = ld(m.at<float>(2));
    return {x,y,z};
}
p3d positionToVector(cv::Mat m) {
    //as written in https://math.stackexchange.com/a/84202
    cv::Mat R = m(cv::Rect(0,0,3,3));
    cv::Mat T = m(cv::Rect(3,0,1,3));
    cv::Mat res = -R.t()*T;
    return matToPoint(res);
}
p3d positionToRotation(cv::Mat m) {
    cv::Mat res = m(cv::Rect(0,0,3,3)).row(2).t();
    return matToPoint(res);
}


#define PI 3.141592653589793238462643383279502884L



p2d target_position;



ld calcRot(p2d r) { //calculate rotation to given vector
    ld angle = arg(complex<ld>(r.x,r.z))-arg(complex<ld>(rot.x,rot.z));
    angle = (angle*180)/PI;
    if(abs(angle)<epsilon) return 0.0;
    return angle;
}
void rotateToVector(Tello& tello, p2d r) { //rotate tello to given vector
    ld ar = calcRot(r);

    cout << "angle to rotate = " <<ar << endl;
    if(abs(ar) < epsilon) return;
    if(ar < 0) moveTello(tello,"ccw",-ar,2);
    else moveTello(tello,"cw",ar,2);
}

void moveToPoint(Tello& tello, p2d nt, ld scale) {
	target_position = nt;
    p2d dir = nt-p2d(pos);
    cout << "dir is " << dir << endl;
    rotateToVector(tello, dir);
    ld dist = dir.abs()*scale;
    cout << "calling forward with " << int(dist) << endl;
    moveTello(tello,"forward",int(dist),2);
}
using vi = vector<int>;
using vld = vector<ld>;
int* moveToDoor(ld* pointsX, ld* pointsZ, int pointCount, ld center[3])
{
	//start by calculating the doors location:
    const int sa = 15;
	const int sc = 360/sa;
    vld sumX(sc,0), sumZ(sc,0), sumXZ(sc,0),sumXX(sc,0),m(sc,0),b(sc,0);
    ld xDiff,zDiff;
	int sector;
	vi spSize(sc,0);
	//divide into sectors and calculate needed values
	for(int i = 0; i < pointCount; i++) {
		xDiff = pointsX[i] - center[0];
		zDiff = pointsZ[i] - center[2];
		sector = (atan2l(zDiff, xDiff)*180/PI)/sa;
		sumX[sector] += pointsX[i];
		sumZ[sector] += pointsZ[i];
		sumXZ[sector] += pointsX[i] * pointsZ[i];
		sumXX[sector] += pointsX[i] * pointsX[i];
		spSize[i]++;
	}
	
	//fit line for every sector's points, and find line closest to center
	int closestLine = -1;
	ld minDistance, currDistance;
	for(int i = 0; i < sc; i++) {
		m[i] = ((spSize[i]*sumXZ[i]) - (sumX[i]*sumZ[i])) / ((spSize[i]*sumXX[i]) - (sumX[i]*sumX[i]));
		b[i] = (sumZ[i] - (m[i]*sumX[i])) / spSize[i];
		currDistance = abs((m[i]*center[0]) - center[2] + b[i])/sqrtl((m[i]*m[i]) + 1);
		if(i == 0 || minDistance > currDistance) {
			closestLine = i;
			minDistance = currDistance;
		}
	}

	//the sector with the door is the sector with the closest line to the center
	
	//find the point on the line which is closest to the center
	ld closestX, closestZ;
	closestX = (m[closestLine]*center[2] + center[0] - m[closestLine]*b[closestLine])/((m[closestLine]*m[closestLine]) + 1);
	closestZ = (m[closestLine]*center[0] + m[closestLine]*m[closestLine]*center[2] + b[closestLine])/((m[closestLine]*m[closestLine]) + 1);

	//start by moving to the point on the line which is closest to the center
	//MoveToPoint(closestX, closestZ);

	//now move in the direction of exit
	//@yanir think of something

}






void savePositions(vector<p3d>& P)  {
    cout << "saving positions..." << endl;
    std::ofstream posStream("/tmp/dronePosition.csv");
    for(auto p : P) {
        posStream << p.x << ',' << p.y << ',' << p.z << endl;
    }
    posStream.close();
    cout << "done." << endl;
}

int ret = 0, finish = 0, scanCalled = 0;
cv::Mat im;




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
    takeOff(tello);
    thread scan_thread;
    while (!ret) usleep(5000);
    int i = 0;
    cv::Mat trackRet;
    vector<p3d> positions;
    bool tmoveUp = false;
    while(!scanFinished.load()) {
        i++;
        if (i % 10 != 0) usleep(5000);
        if (im.empty()) {
            cerr << endl << "Failed to load image from camera :(" << endl;
            exit(1);
        }
        trackRet = SLAM.TrackMonocular(im, i);
        bool localized = !trackRet.empty();
        droneLocalized.store(localized);
        if(localized) {
            pos = positionToVector(trackRet);
            rot = positionToRotation(trackRet);
            cout << "pos="<<pos<<",rot="<<rot<<",ap="<< op_v(trackRet) << '\n';
            cout << "angle=" << p2d(rot).angle() << '\n';
            positions.push_back(pos);
        }
        if (localized && !scanCalled) {
            scan_thread = thread(secondThread, std::ref(tello));
            scanCalled = 1;
        }
        if(!localized&& !scanCalled) {
            if(i>100&&!tmoveUp) moveTello(tello,"up",20),tmoveUp=1;   
        }
    }
    finish = 1;
    // Stop all threads
    saveMap(SLAM);
    savePositions(positions);
    SLAM.Shutdown();
    tello.SendCommand("streamoff");
    land(tello);
    // imageChoice = -100;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
