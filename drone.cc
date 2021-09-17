#include "common.cc"
namespace drone_exit_room {
    class Drone {
    private:
        Tello tello;
    public:
        Drone() {}

        bool init() {
            if(!tello.Bind()) return 0;
            tello.SendCommand("streamon");
            return 1;
        }
        
        //send a command to the drone, wait for a response
        //and sleep for dur seconds
        void command(string com, int dur = 0) {
            tello.SendCommand(com);
            while (!tello.ReceiveResponse()) csleep(5);
            if (dur > 0) csleep(dur*1000);
        }
        //dir is one of up,down,left,right,forward,back
        //can also be used with cw,ccw but it is recommended
        //to use the rotate function.
        //dur is how much to sleep in seconds
        void move(string dir, int x, int dur = 1) {
            command(dir + " " + to_string(x), dur);
        }
        //angle in degrees
        //dur is how much to sleep in seconds
        //if the absolute value of angle is more than 45,
        //we will divide it into smaller rotations
        //if go is true, drone will go up and down after every rotation
        void rotate(int angle, bool go, int dur = 0) {
            if(angle==0) return;
            int sgn = angle>0 ? 1 : -1;
            //if angle is outside [-360,360], call rotate again
            //with the matching angle inside the range [-360,360]
            if(abs(angle)>=360) return rotate(sgn*(abs(angle)%360),dur);
            //if the angle is inside [-360,360]
            //but not inside [-180,180]
            //we can turn in the other direction
            //for a smaller angle
            if(abs(angle)>180) { 
                int na = angle > 0 ? (360-angle) : (angle+360);
                return rotate(na, dur);
            }
            string dir = angle>0 ? "ccw" : "cw";
            //if the angle is big, cut it into smaller rotations
            //to avoid losing localization
            if(abs(angle) > 30) {
                move(dir,20,dur);
                if(go) goUpDown();
                return rotate(angle-20*sgn,go,dur);
            }
            //otherwise, do the whole rotation in one go
            move(dir,abs(angle),dur);
            if(go) goUpDown();
        }

        //take off the drone.
        void takeOff() {
            tello.SendCommand("takeoff");
            sleep(1);
            while (!(tello.ReceiveResponse()));
        }

        //land the drone.
        void land() { command("land"); }
        
        //land and turn off the camera stream.
        void turnOff() {
            land();
            tello.SendCommand("streamoff");
        }

        //can help create more points by creating different POVs
        void goUpDown() {
            move("up", svm, 1);
            move("down", svm, 2);
        }

    };

}
