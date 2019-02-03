#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

class Ota
{
	WebServer server(int port = 80);
	const char* host = "esp32";
	const char* ssid = "Hotspot-ilyas121";
	const char* password = "NiZTumeA";
	void begin(void);
	void loop(void);
	Ota(void);
};
