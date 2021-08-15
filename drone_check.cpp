#include <iostream>
#include <ctello.h>
using namespace std;
using namespace ctello;
int main() {
	Tello t;
	if(!t.Bind()) {
		return 0;
	}
	t.SendCommand("takeoff");
	while(!(t.ReceiveResponse()));
	t.SendCommand("land");
	while(!(t.ReceiveResponse()));
	return 0;
}
