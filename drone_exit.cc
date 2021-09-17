
#include "common.cc"
#include "algo.cc"
#include "drone.cc"
namespace drone_exit_room {

    atomic<int> algoThreadDone(0),scanFinished(0),mapPointsReady(0),droneLocalized(0);

    //our real time position and rotation in 3D
    p3d pos,rot;
    //and also position in 2D
    p2d pos2d;

    //move to target point
    void moveToPoint(Drone& drone, p2d nt);

    //rotate max_angle degrees in direction dir (1,-1)
    //if leaves localization, goes back to relocalize
    void safeRotate(Drone& drone, int dir, int angle, int go) {
        //current angle and velocity.
        //if the drone is not localized, we divide velocity by half.
        //if it is localized, we use v = (v+sra)/2 to slowly bring it back to sra.
        int cur = 0, v = sra;
        //true if during the last iteration we were localized,
        //false otherwise
        bool lastLoc = true;
        //while we haven't completed the rotation or the drone isn't localized,
        while (cur < angle || !droneLocalized.load()) {
            if (droneLocalized.load()) {
                //we can move forward
                //make sure we're not passing required angle
                int ca = min(v,angle-cur);
                drone.rotate(dir*ca,go||(!lastLoc),1);
                cur += ca;
                //increase v if slowed down earlier
                v = (v + sra) / 2;
                lastLoc = true;
            } else {
                //go back and try to relocalize
                drone.rotate(-dir*v,1,1);
                cur -= v;
                //and then come back slower
                v = max(v/2,10);
                lastLoc = false;
            }
            //going up and down is now inside drone.rotate()
        }
    }

    void scan(Drone& drone) {
        //safely rotate 360 degrees
        safeRotate(drone,1,360,1);
        //if we didn't complete the circle, continue rotating
        int angle = int(p2d(rot).angle());
        while(angle < 0) {
            safeRotate(drone,1,abs(angle),1);
            angle = int(p2d(rot).angle());
        }
        scanFinished.store(1);
    }

    //a global vector to store all map points extracted from
    //ORB_SLAM system.
    vp3d map_points;

    //the heart of our control
    //first we scan the room. we save the points, use our algorithm
    //to find the exit, and move towards it.
    void algoThread(Drone& drone) {
        scan(drone);
        //scanFinished is now 1,
        //and main should extract the map's points for us.
        //wait for it.
        while(!mapPointsReady.load()) usleep(10000);
        int N = int(map_points.size());
        //convert points to 2D
        vp2d map_points_2D(N);
        for(int i = 0; i < N; ++i) map_points_2D[i] = p2d(map_points[i]);
        pair<p2d,ld> res = algo::findExit(map_points_2D);
        drone.goUpDown(); //to avoid inactivity
        if(res.second < -epsilon) {
            //if the distance is negative, no exit point was found.
            //we should simply finish our program.
            cout << "*********************" << endl;
            cout << "NO EXIT POINT FOUND!!" << endl;
            cout << "*********************" << endl;
        } else moveToPoint(drone,res.first);

        algoThreadDone.store(1);
    }

    //given camera pose matrix, calculate our position
    //as shown in // https://math.stackexchange.com/a/84202
    p3d positionToVector(cv::Mat m) {
        cv::Mat R = m(cv::Rect(0,0,3,3));
        cv::Mat T = m(cv::Rect(3,0,1,3));
        cv::Mat res = -R.t()*T;
        return matToPoint(res);
    }

    //given camera pose matrix, calculate our rotation
    //as shown in // https://math.stackexchange.com/a/84202
    p3d positionToRotation(cv::Mat m) {
        cv::Mat res = m(cv::Rect(0,0,3,3)).row(2).t();
        return matToPoint(res);
    }

    //calculate rotation to given vector
    ld calcRot(p2d r) { 
        ld angle = r.positive_angle()-p2d(rot).positive_angle();
        if(abs(angle)<epsilon) return 0.0;
        return angle;
    }
    //rotate tello to given vector
    void rotateToVector(Drone& drone, p2d r) {
        ld ar = calcRot(r); //angle (to) rotate

        //if angle is smaller then 20, we're pretty much in direction
        if(abs(ar) < 20-epsilon) return;
        int ar_int = llroundl(ar);
        if(abs(ar_int)<20) { //we checked already if it is smaller than 20, but just in case
            if(ar_int > 0) ar_int = 20;
            else ar_int = -20;
        }
        //make sure angle is in the [-360,360] range
        while(ar_int >= 360) ar_int -= 360;
        while(ar_int <= -360) ar_int += 360;
        //make sure the angle is in the [-180,180] range
        if(ar_int > 180) ar_int = 360-ar_int;
        if(ar_int < -180) ar_int += 360;
        //rotate by just a little bit (at most sra)
        int sgn = ar_int > 0 ? 1 : -1;
        ar_int = abs(ar_int);
        int ca = min(ar_int, sra);
        safeRotate(drone,sgn,ca,0);
        //calling recursively to make sure we reach the right angle
        rotateToVector(drone,r);
    }

    //try to find a localization
    void relocalize(Drone& drone) {
        //current angle
        int angle = 0;
        const int v = 15,max_angle = 360;
        while (angle<max_angle) {
            if (!droneLocalized.load()) {
                //go on and try to relocalize
                //however, if we are going to rotate by too much, return
                if(angle+v > max_angle) {
                    drone.rotate(-angle,1,1);
                    return;
                }
                drone.rotate(v,1,1);
                angle += v;
            } else {
                //we relocalized! return
                return;
            } 
        }
    }
    //move to target point
    void moveToPoint(Drone& drone, p2d target) {
        ld max_dist = 0.0;
        //our direction vector is the target minus our position
        p2d dir = target-p2d(pos2d),last_pos;
        //if distance is smaller than the maximum distance we traveled, we should stop
        while(dir.abs()>max_dist) {
            rotateToVector(drone,dir);
            last_pos = pos2d; //saving last position to calculate distance traveled
            drone.move("forward",20,2);
            drone.goUpDown();
            //try to relocalize if needed
            if(!droneLocalized.load()) {
                relocalize(drone);
                //if failed to relocalize, return
                if(!droneLocalized.load()) return;
            }
            //update max_dist and direction accordingly
            ld dist_trav = (pos2d-last_pos).abs();
            if(dist_trav-epsilon>max_dist) max_dist = dist_trav; 
            dir = target-pos2d;
        }
    }

    //extract map points to global vector map_points
    void extractMapPoints(ORB_SLAM2::System& slam) {
        vector<ORB_SLAM2::MapPoint*> mapPoints = slam.GetMap()->GetAllMapPoints();
        map_points.clear();
        for(auto p : mapPoints) {
            if(p != nullptr) {
                auto point = p->GetWorldPos();
                Eigen::Matrix<double,3,1> v = ORB_SLAM2::Converter::toVector3d(point);
                map_points.push_back({v.x(),v.y(),v.z()});
            }
        }
    }

    //save data for analysis (saveMap saves points, savePositions saves drone's positions)
    void saveMap(ORB_SLAM2::System& slam) {
        cout << "saving map..." << endl;
        //extract using the extraction function
        //and then write points to the stream
        extractMapPoints(slam);
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for (auto v : map_points) pointData << v.x << "," << v.y << "," << v.z << endl;
        pointData.close();
    }
    //save drone's positions, given as a vector of p3d
    void savePositions(vp3d& P)  {
        cout << "saving positions..." << endl;
        std::ofstream posStream("/tmp/dronePosition.csv");
        for(auto p : P) posStream << p.x << ',' << p.y << ',' << p.z << endl;
        posStream.close();
        cout << "done." << endl;
    }

    //global variables to communicate between threads
    //ret means we've read the first image
    //finish means the whole program is done
    int ret = 0, finish = 0;
    cv::Mat im;
    //takePicture is called on a different thread to ensure our system gets the latest image
    void takePicture() {
        //open the drone's camera stream
        VideoCapture capture{"udp://0.0.0.0:11111?fifo_size=100000", cv::CAP_FFMPEG};
        if (!capture.isOpened()) {
            cerr << "camera not opened :(" << endl;
            exit(1);
        }
        //read images until we're done
        while (!finish) {
            capture >> im;
            ret = 1;
        }
    }

    //run find exit given initialized slam system and drone
    int run(ORB_SLAM2::System& SLAM, Drone& drone) {
        if (!drone.init()) return 1;
        thread picture_thread(takePicture), algo_thread;
        drone.takeOff();
        //wait for pictures
        while (!ret) usleep(5000);
        //the i variable helps to syncronize between the main thread
        //and the pictures thread.
        //credits to the group of Daniel Greenhut
        int i = 0;
        //trackRet is the camera pose matrix returned by ORB_SLAM
        cv::Mat trackRet;
        //positions is the drone's positions vector
        vp3d positions;
        //variables to hold whether we've called the scan,
        //and whether we've called the extraction of map points after the scan
        bool scanCalled = false, extractCalled = false;
        while(!algoThreadDone.load()) {
            i++;
            if (i % 10 != 0) usleep(5000);
            if (im.empty()) {
                cerr << endl << "Failed to load image from camera :(" << endl;
                exit(1);
            }
            //TrackMonocular returns an empty matrix if not localized
            //and a 4x4 matrix containing information about rotation and translation
            trackRet = SLAM.TrackMonocular(im, i);
            bool localized = !trackRet.empty();
            droneLocalized.store(localized);
            if(localized) {
                //compute position and rotation only if localized
                pos = positionToVector(trackRet);
                rot = positionToRotation(trackRet);
                pos2d = p2d(pos);
                positions.push_back(pos);
            }
            if (localized && !scanCalled) {
                //algoThread receives a reference of drone,
                //however this is checked at compile time.
                //std::ref "converts" drone to reference at "compile-time"
                algo_thread = thread(algoThread, std::ref(drone));
                scanCalled = 1;
            }
            if(scanFinished.load() && !extractCalled) {
                extractMapPoints(SLAM);
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
        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        //join threads and finish execution.
        picture_thread.join();
        algo_thread.join();
        return 0;
    }

    //run find exit given path to orb slam vocabulary and settings, and drone
    int run(const string& path_to_vocabulary, const string& path_to_settings, Drone& drone) {
        // Create SLAM system. It initializes all system threads and gets ready to
        // process frames.
        ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR, true);
        return run(SLAM,drone);
    }

    //run find exit given path to orb_slam vocabulary and settings
    int run(const string& path_to_vocabulary, const string& path_to_settings) {
        Drone drone;
        return run(path_to_vocabulary,path_to_settings,drone);
    }
    //run find exit given only orb slam (initialize drone ourselves)
    int run(ORB_SLAM2::System& slam) {
        Drone drone;
        return run(slam, drone);
    }

}
