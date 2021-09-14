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

#include "common.cc"
#define DBG 1
#include "algo.cc"
#include "drone.cc"
//set to true in order to work
//set to false for debug purposes (hold drone by hands and scan room)
bool should_fly = false;
atomic<int> secondThreadDone(0);
atomic<int> droneLocalized(0);

p3d pos,rot;




/* deprecated
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
}*/
void moveToPoint(Drone& drone, p2d nt, ld scale);

void writeToFile(string filename, string content){
    std::ofstream stream(filename);
    cout << "writing to file " << filename << " that " << content << endl;
    stream << content << endl;
    stream.close();
}

void scan(Drone& drone) {
    int angle = 0, v = sra;
    if(!should_fly) {
        cin >> angle; //pretend we're scanning until input is read
        return;
    }
    while (angle < 360 || !droneLocalized.load()) {
        if (droneLocalized.load()) {
            drone.rotate(-v,1);
            angle += v;
            v = (v + sra) / 2;
        } else {
            drone.rotate(v,1);
            angle -= v;
            v = v / 2;
            if (v <= 10) v = 10;
        }
        cout << "angle is now " << angle << endl;
        drone.goUpDown();
    }
    // for now, land immeidiatly
    //land(tello);
    //scanFinished.store(1);
}
void secondThread(Drone& drone) {
    scan(drone);
    cout << "finished scan" << endl;
    //ld sc = findScale(tello);
    //writeToFile("/tmp/scale_log.txt","scale is " + to_string(sc));
    //moveToPoint(drone, {0.0,0.5},1.0);
    int x;
    cin >> x;
    cout << "sleeping..." << endl;
    usleep(2000000);
    cout << "LEETSSS GOOOO" << endl;
    secondThreadDone.store(1);
}





p3d matToPoint(cv::Mat m) {
    ld x = ld(m.at<float>(0));
    ld y = ld(m.at<float>(1));
    ld z = ld(m.at<float>(2));
    return {x,y,z};
}

//in these two functions we took inspiration from 
// https://math.stackexchange.com/a/84202

p3d positionToVector(cv::Mat m) {
    cv::Mat R = m(cv::Rect(0,0,3,3));
    cv::Mat T = m(cv::Rect(3,0,1,3));
    cv::Mat res = -R.t()*T;
    return matToPoint(res);
}
p3d positionToRotation(cv::Mat m) {
    cv::Mat res = m(cv::Rect(0,0,3,3)).row(2).t();
    return matToPoint(res);
}


ld calcRot(p2d r) { //calculate rotation to given vector
    ld angle = r.angle()-p2d(rot).angle();
    angle = (angle*180)/PI;
    if(abs(angle)<epsilon) return 0.0;
    return angle;
}
void rotateToVector(Drone& drone, p2d r) { //rotate tello to given vector
    ld ar = calcRot(r);
    cout << "angle to rotate = " <<ar << endl;
    if(abs(ar) < epsilon) return;
    if(abs(ar) < 20+epsilon) {
        cout << "approximately in direction, not rotating" << endl;
        return;
    }
    drone.rotate(ar,2);
    drone.goUpDown();
    rotateToVector(drone,r);
}

void moveToPoint(Drone& drone, p2d nt, ld scale) {
    p2d dir = nt-p2d(pos);
    cout << "dir is " << dir << endl;
    rotateToVector(drone, dir);
    ld dist = dir.abs()*scale;
    cout << "calling forward with " << int(dist) << endl;
    drone.move("forward", int(dist), 2);
}
//save data for analysis (saveMap saves points, savePositions saves drone's positions)
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
void savePositions(vector<p3d>& P)  {
    cout << "saving positions..." << endl;
    std::ofstream posStream("/tmp/dronePosition.csv");
    for(auto p : P) posStream << p.x << ',' << p.y << ',' << p.z << endl;
    posStream.close();
    cout << "done." << endl;
}


//global variables to communicate between threads
int ret = 0, finish = 0, scanCalled = 0;
cv::Mat im;
//takePicture is called on a different thread to ensure our system gets the latest image
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
    Drone drone;
    if (!drone.init()) return 0;
    thread picture_thread(takePicture);
    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR,
                           true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;
    
    // Main loop
    if(should_fly) drone.takeOff();
    thread scan_thread;
    while (!ret) usleep(5000);
    int i = 0;
    cv::Mat trackRet;
    vector<p3d> positions;
    bool tmoveUp = false;
    while(!secondThreadDone.load()) {
        i++;
        if (i % 10 != 0) usleep(5000);
        if (im.empty()) {
            cerr << endl << "Failed to load image from camera :(" << endl;
            exit(1);
        }
        //TrackMonocular returns a 4x4 matrix
        trackRet = SLAM.TrackMonocular(im, i);
        bool localized = !trackRet.empty();
        droneLocalized.store(localized);
        if(localized) {
            pos = positionToVector(trackRet);
            rot = positionToRotation(trackRet);
            cout << "pos="<<pos<<",rot="<<rot << '\n';
            cout << "angle=" << p2d(rot).angle() << '\n';
            positions.push_back(pos);
        }
        if (localized && !scanCalled) {
            scan_thread = thread(secondThread, std::ref(drone));
            scanCalled = 1;
        }
        if(!localized&& !scanCalled) {
            //if we did not localize and did not call scan,
            //we should move the drone to help it localize
            if(i>100&&!tmoveUp) drone.move("up",20),tmoveUp=1;   
        }
    }
    finish = 1;
    // Stop all threads and save data
    saveMap(SLAM);
    savePositions(positions);
    SLAM.Shutdown();
    drone.turnOff();
    // imageChoice = -100;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
