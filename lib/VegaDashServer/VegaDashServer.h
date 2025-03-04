#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "Drone.h"

const char *ssid = "ESP32 Copter";
const char *password = "12345678";

class VegaDashServer {
    public:
        VegaDashServer(Drone* _drone);
        void setup();
        void loop();
    private:
        unsigned long lastUpdate;
        const int updateInterval = 30; // ~30Hz
        void sendData();

        static WebSocketsServer* webSocket;
        static StaticJsonDocument<1024> doc;
        static Drone* drone;
        static StaticJsonDocument<1024> doc;
        static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
        static String getJSONfromDroneState();
};
