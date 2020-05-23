#include "PIDComm.h"

PIDComm::PIDComm(int p){
	port = p;
}

void setup(){
    if(udp.connect(IPAddress(192,168,1,100), port)) {
        udp.onPacket([](AsyncUDPPacket packet) {
			double[3] values = packet.data();
            tuneConstants(values);
        });
    }
}
void loop();
void sendData();
void tuneConstants(values){
	//Update DPID constants to values[0-3]
}


