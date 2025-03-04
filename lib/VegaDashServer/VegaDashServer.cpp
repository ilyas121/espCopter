#include "VegaDashServer.h"

VegaDashServer::VegaDashServer(Drone* _drone) {
    drone = _drone;
    webSocket = new WebSocketsServer(9090);
}

void VegaDashServer::setup() {
    WiFi.softAP(ssid, password);
    webSocket->begin();
    webSocket->onEvent(webSocketEvent);
}

void VegaDashServer::loop() {
	webSocket->loop();
}

void VegaDashServer::sendData() {
	
}

void VegaDashServer::webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket->remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
                
                // Send initial data immediately upon connection
                String jsonString = getJSONfromDroneState();
                webSocket->sendTXT(num, jsonString);
            }
            break;
            
        case WStype_TEXT:
            {
                // Print the full payload as a string
                String payloadStr = String((char*)payload);
                Serial.println("Received WebSocket payload: " + payloadStr);
                
                // Parse incoming JSON message
                StaticJsonDocument<200> doc;
                DeserializationError error = deserializeJson(doc, payload);
                
                if (error) {
                    Serial.print("deserializeJson() failed: ");
                    Serial.println(error.c_str());
                    return;
                }

                // Check if this is a PID update message
                if (doc.containsKey("gains")) {
                    JsonArray pidValues = doc["gains"];
                    if (pidValues.size() == 9) {
                        double gains[9];
                        for (int i = 0; i < 9; i++) {
                            gains[i] = pidValues[i];
                        }
                        drone->updateGain(gains);
                        Serial.println("gains");
                        drone->printGains();
                    }
                }
            }
            break;
            
        case WStype_BIN:
            Serial.printf("[%u] Received binary length: %u\n", num, length);
            break;
            
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}

String VegaDashServer::getJSONfromDroneState() {
    doc.clear();
    
    // IMU Array
    JsonArray imu = doc.createNestedArray("IMU");
    imu.add(drone->getRoll());      // degrees
    imu.add(drone->getPitch());     // degrees
    imu.add(drone->getYaw());       // degrees
    imu.add(drone->getRollVelocity());   // degrees/second
    imu.add(drone->getPitchVelocity());  // degrees/second
    imu.add(drone->getYawVelocity());    // degrees/second
    // Add calibration values
    imu.add(drone->getSystemCalibration());  // 0-3
    imu.add(drone->getGyroCalibration());    // 0-3
    imu.add(drone->getAccelCalibration());   // 0-3
    imu.add(drone->getMagCalibration());     // 0-3
    
    // PID Output Array
    JsonArray pidOutput = doc.createNestedArray("PIDOutput");
    pidOutput.add(drone->getRollPIDOutput());   // -100 to 100
    pidOutput.add(drone->getPitchPIDOutput());  // -100 to 100
    pidOutput.add(drone->getYawPIDOutput());    // -100 to 100
    
    // Setpoints Array
    JsonArray setpoints = doc.createNestedArray("Setpoints");
    setpoints.add(drone->getRollSetpoint());    // degrees
    setpoints.add(drone->getPitchSetpoint());   // degrees
    setpoints.add(drone->getYawSetpoint());     // degrees
    
    // PID Gains Array
    JsonArray pidGains = doc.createNestedArray("PIDGains");
    double gains[9];
    drone->getGains(gains);  // Assuming this method exists or needs to be added
    for(int i = 0; i < 9; i++) {
        pidGains.add(gains[i]);
    }
    
    // Receiver Values Array (normalized to 0-100%)
    JsonArray receiverValues = doc.createNestedArray("ReceiverValues");
    receiverValues.add(drone->getReceiverValue(0));  // CH1
    receiverValues.add(drone->getReceiverValue(1));  // CH2
    receiverValues.add(drone->getReceiverValue(2));  // CH3
    receiverValues.add(drone->getReceiverValue(3));  // CH4
    receiverValues.add(drone->getReceiverValue(4));  // CH5
    receiverValues.add(drone->getReceiverValue(5));  // CH6
    
    // Motor Values Array (normalized to 0-100%)
    JsonArray motorValues = doc.createNestedArray("MotorValues");
    double* motors = drone->getMotorValues();  // Assuming this method exists or needs to be added
    motorValues.add(motors[0]);
    motorValues.add(motors[1]);
    motorValues.add(motors[2]);
    motorValues.add(motors[3]);
    
    // Battery Life (you'll need to implement battery monitoring)
    doc["BatteryLife"] = 100;  // Placeholder value, implement actual battery monitoring
    
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
    
}