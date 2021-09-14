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
    void command(string com, double dur = 0.0) {
        cout << "sending command " << com << endl;
        tello.SendCommand(com);
        while (!tello.ReceiveResponse());
        if (dur > epsilon) usleep(int(dur*1000000));

    }
    //dir is one of up,down,left,right,forward,back
    //can also be used with cw,ccw but it is recommended
    //to use the rotate function.
    //dur is how much to sleep in seconds
    void move(string dir, int x, double dur = 0.0) {
        command(dir + " " + to_string(x), dur);
    }
    //angle in degrees
    //dur is how much to sleep in seconds
    //if the absolute value of angle is more than 45,
    //we will divide it into smaller rotations
    void rotate(int angle, double dur = 0.0) {
        //if(abs(angle) < 20) return;
        int sgn = angle>0 ? 1 : -1;
        string dir = angle>0 ? "ccw" : "cw";
        if(abs(angle) > 30) {
            move(dir,20*sgn,dur);
            goUpDown();
            return rotate(angle-20*sgn,dur);
        }
        move(dir,abs(angle),dur);
        goUpDown();
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
    void goUpDown() {
        move("up", svm, 1);
        move("down", svm, 1);
        usleep(1000000);
    }
};