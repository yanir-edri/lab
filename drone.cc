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
    void rotate(ld angle, double dur = 0.0) {
        if(abs(angle) < epsilon) return;
        if(angle>0) move("ccw",angle,dur);
        else move("cw",-angle,dur);
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