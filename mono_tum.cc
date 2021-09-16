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
bool should_fly = true;
atomic<int> secondThreadDone(0);
atomic<int> scanFinished(0);
atomic<int> mapPointsReady(0);
atomic<int> droneLocalized(0);

p3d pos,rot;
p2d pos2d;


void moveToPoint(Drone& drone, p2d nt);

void writeToFile(string filename, string content){
    std::ofstream stream(filename);
    cout << "writing to file " << filename << " that " << content << endl;
    stream << content << endl;
    stream.close();
}

//rotate max_angle degrees in direction dir (1,-1)
//if leaves localization, goes back to relocalize
void safeRotate(Drone& drone, int dir, int angle, int go) {
    cout << "safeRotate(" << angle <<") is called" << endl;
    int cur = 0, v = sra;
    if(!should_fly) {
        cin >> cur; //pretend we're scanning until input is read
        return;
    }
    while (cur < angle || !droneLocalized.load()) {
        if (droneLocalized.load()) {
            int ca = min(v,angle-cur);
            drone.rotate(dir*ca,go,1);
            cur += ca;
            v = (v + sra) / 2;
        } else {
            drone.rotate(-dir*v,1,1);
            cur -= v;
            v = v / 2;
            if (v <= 10) v = 10;
        }
        cout << "angle is now " << cur << endl;
        //going up and down is now inside drone.rotate()
    }
}

void scan(Drone& drone) {
    safeRotate(drone,1,360);
    scanFinished.store(1);
}
vp3d map_points;
void secondThread(Drone& drone) {
    cout << "calling scan" << endl;
    scan(drone);
    cout << "finished scan :)" << endl;
    while(!mapPointsReady.load()) usleep(10000);
    cout << "converting to 2D points..." << endl;
    int N = int(map_points.size());
    vp2d map_points_2D(N);
    for(int i = 0; i < N; ++i) map_points_2D[i] = p2d(map_points[i]);
    cout << "calling algorithm!" << endl;
    pair<p2d,ld> res = algo::findExit(map_points_2D);
    drone.goUpDown(); //to avoid inactivity
    cout << "algorithm returned " << res << endl;
    cout << "it's angle is " << res.first.positive_angle() << endl;
    cout << "current angle is about " << p2d(rot).positive_angle() << endl;
    cout << "please confirm: ";
    // drone.goUpDown();
    string _USER = "y";
    // cin >> _USER;
    if(_USER=="y") {
        if(res.second < -epsilon) {
            cout << "*********************" << endl;
            cout << "NO EXIT POINT FOUND!!" << endl;
            cout << "*********************" << endl;
        } else {
            cout << "calling moveToPoint" << endl;
            moveToPoint(drone,res.first);
            cout << "moveToPoint done" << endl;
        }
    }
    cout << "second thread is done! :)" << endl;
    secondThreadDone.store(1);
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
    cout << "calculating rot to " << r << endl;
    ld angle = r.positive_angle()-p2d(rot).positive_angle();
    cout << "angle = r.angle() - rot.angle() = " << r.positive_angle() << " - " << p2d(rot).positive_angle() << " = " << angle << endl;
    if(abs(angle)<epsilon) return 0.0;
    return angle;
}
void rotateToVector(Drone& drone, p2d r) { //rotate tello to given vector
    ld ar = calcRot(r);
    cout << "angle to rotate = " <<ar << endl;
    if(abs(ar) < epsilon) return;
    if(abs(ar) < 10-epsilon) {
        cout << "approximately in direction, not rotating" << endl;
        return;
    }
    int ar_int = llroundl(ar);
    if(abs(ar_int)<10) { //we checked already if it is smaller than 10, but just in case
        if(ar_int > 0) ar_int = 10;
        else ar_int = -10;
    }
    while(ar_int >= 360) ar_int -= 360;
    while(ar_int <= -360) ar_int += 360;
    if(ar_int > 180) ar_int = 360-ar_int;
    if(ar_int < -180) ar_int += 360;
    int sgn = ar_int > 0 ? 1 : -1;
    safeRotate(drone,sgn,abs(ar_int));
    cout << "safeRotate of rotateToVector done, callign rotateToVector again" << endl;
    rotateToVector(drone,r);
}

//try to relocalize by turning around
//and turning back when localizing
//but not turning too much (up to max_angle degrees)
void relocalize(Drone& drone, int max_angle, int dir) {
    const int max_v = 25;
    int angle = 0,v=max_v;
    while (!droneLocalized.load()) {
        if (!droneLocalized.load()) {
            drone.rotate(dir*v,1,1);
            angle -= v;
            v = (v + sra) / 2;
        } else {
            if(angle+v > max_angle) {
                //if we are going to rotate by too much, return
                drone.rotate(dir*angle,1,1);
                return;
            }
            drone.rotate(-dir*v,0,1);
            angle += v;
            v = v / 2;
            if (v <= 10) v = 10;
        }
    }
}
void moveToPoint(Drone& drone, p2d target) {
    ld max_dist = 0.0;
    p2d dir = target-p2d(pos2d),last_pos;
    while(dir.abs()>max_dist) {
        cout << "pos="<<pos2d<<",target="<<target<<",dir="<<dir<<endl;
        last_pos = pos2d;
        rotateToVector(drone,dir);
        drone.move("forward",20,2);
        drone.goUpDown();
        if(!droneLocalized.load()) {
            cout << "not localized :(\ntrying to relocalize"<<endl;
            int current_max_angle = 60;
            do {
                cout << "current_max_angle for relocalize is " << current_max_angle << endl;
                relocalize(drone,current_max_angle,1);
                if(droneLocalized.load()) break;
                cout << "trying the other direction" << endl;
                relocalize(drone,current_max_angle,-1);
                current_max_angle += 10;
                if(current_max_angle > 120) {
                    cout << "couldn't relocalize! aborting :(" << endl;
                    return;
                }
            } while(!droneLocalized.load());
        }
        ld dist_trav = (pos2d-last_pos).abs();
        cout << "traveled " << dist_trav << endl;
        if(dist_trav-epsilon>max_dist) max_dist = dist_trav; 
        dir = target-pos2d;
    }
    cout << "distance left is " << dir.abs() << ", max dist traveled is " << max_dist << endl;
    cout << "moveToPoint is done :)" << endl;
}

//extract map points to global vector map_points
void extractMapPoints(ORB_SLAM2::System& slam) {
    vector<ORB_SLAM2::MapPoint*> mapPoints = slam.GetMap()->GetAllMapPoints();
    map_points.clear();
    for(auto p : mapPoints) {
        if(p != nullptr) {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double,3,1> v = ORB_SLAM2::Converter::toVector3d(point);
            map_points.pb({v.x(),v.y(),v.z()});
        }
    }
}

//save data for analysis (saveMap saves points, savePositions saves drone's positions)
//TODO: replace to extractMapPoints and a simple for loop
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
int ret = 0, finish = 0;
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
    thread picture_thread(takePicture), scan_thread;
    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR,
                           true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;
    
    // Main loop
    if(should_fly) drone.takeOff();
    while (!ret) usleep(5000);
    int i = 0;
    cv::Mat trackRet;
    vector<p3d> positions;
    bool tmoveUp = false, scanCalled = false, extractCalled = false;
    while(!secondThreadDone.load()) {
        //the i variable helps to syncronize between the main thread
        //and the pictures thread.
        //credits to the group of Daniel Greenhut
        i++;
        if (i % 10 != 0) usleep(5000);
        if (im.empty()) {
            cerr << endl << "Failed to load image from camera :(" << endl;
            exit(1);
        }
        //TrackMonocular returns an empty matrix if not localized
        //and a 4 by 4 matrix containing information about rotation and translation
        trackRet = SLAM.TrackMonocular(im, i);
        bool localized = !trackRet.empty();
        droneLocalized.store(localized);
        if(localized) {
            //compute position and rotation only if localized
            pos = positionToVector(trackRet);
            rot = positionToRotation(trackRet);
            pos2d = p2d(pos);
            //cout << "pos="<<pos<<",rot="<<rot << '\n';
            //cout << "angle=" << p2d(rot).angle() << '\n';
            positions.push_back(pos);
        }
        if (localized && !scanCalled) {
            //secondthread receives a reference of drone,
            //however this is checked at compile time.
            //std::ref "converts" drone to reference at "compile-time"
            scan_thread = thread(secondThread, std::ref(drone));
            scanCalled = 1;
        }
        if(!localized&& !scanCalled) {
            //if we did not localize and did not call scan,
            //we should move the drone to help it localize
            //if(i>100&&!tmoveUp) drone.move("up",20),tmoveUp=1;   
        }
        if(scanFinished.load() && !extractCalled) {
            cout << "Extracting map points..." << endl;
            extractMapPoints(SLAM);
            cout << "Finished extracting map points :)" << endl;
            extractCalled = 1;
            mapPointsReady.store(1);
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
    //join threads and finish execution.
    picture_thread.join();
    scan_thread.join();
    return 0;
}
