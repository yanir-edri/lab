#include "common.cc"
class Drone {
private:
    Tello tello;
public:
    bool init() {
        if(!tello.Bind()) return 0;
        tello.SendCommand("streamon");
        return 1;
    }
    Drone() {}
    void command(string com, int dur = 0) {
        cout << "sending command " << com << " at time " << time_since_start() << 's' << endl;
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
    void rotate(int angle, bool go, int dur = 0) {
        //if(abs(angle) < 20) return;
        if(angle==0) { cout << "rotate(0) called. not doing anything." << endl; }
        int sgn = angle>0 ? 1 : -1;
        if(abs(angle)>=360) {
            cout << "rotate("<<angle<<") called, calling rotate(" << sgn*(abs(angle)%360) << ") instead" << endl;
            return rotate(sgn*(abs(angle)%360),dur);
        }
        if(abs(angle)>180) { 
            int na = angle > 0 ? (360-angle) : (angle+360);
            cout << "rotate("<<angle<<") called, calling rotate(" << na << ") instead" << endl;
            return rotate(na, dur);
        }
        string dir = angle>0 ? "ccw" : "cw";
        if(abs(angle) > 30) {
            move(dir,20,dur);
            if(go) goUpDown();
            return rotate(angle-20*sgn,go,dur);
        }
        move(dir,abs(angle),dur);
        if(go) goUpDown();
    }
    void takeOff() {
        tello.SendCommand("takeoff");
        sleep(1);
        while (!(tello.ReceiveResponse()));
    }
    void land() { command("land"); }
    
    void turnOff() {
        land();
        tello.SendCommand("streamoff");
    }

    //can help create more points by creating different POVs
    bool state = 0;
    void goUpDown() {
        //if(state) move("down",svm,2), state = 0;
        move("up", svm, 1);
        //state = 1;
        //csleep(2000);
        move("down", svm+2, 2);
        //csleep(2000);
        //state = 0;
    }
};