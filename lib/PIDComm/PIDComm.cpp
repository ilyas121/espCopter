#include "PIDComm.h"

PIDComm::PIDComm(int p, Drone* drone){
	port = p;
    pidcontroller = drone;
    setup();
}

void PIDComm::setup(){
    if(udp.listen(port)) {
        udp.onPacket([this](AsyncUDPPacket packet) {
            //Serial.write(packet.data(), packet.length());
            double *data = (double*)packet.data();
            tuneConstants(data);
            //reply to the client
            //packet.printf("Got %u bytes of data", packet.length());
            udp.broadcastTo("Recieved!!", port);
        });
    }
}
void PIDComm::loop(){
    udp.broadcastTo("Anyone here?", port);
}
void PIDComm::sendData(){

}
void PIDComm::tuneConstants(double* values){
	//Update DPID constants to values[0-3]
    //Serial.write(values);
    pidcontroller->setConstants(values[0], values[1], values[2]);
}


