#include "Arduino.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char *ssid = "ESP32 Test";
const char *password = "12345678";
WebSocketsServer webSocket(9090);
StaticJsonDocument<1024> doc;

// Timing variables
unsigned long lastUpdate = 0;
const int UPDATE_INTERVAL = 1000;  // 1 second between updates

// Add this global variable at the top with other globals
int testCounter = 0;
const int MAX_COUNTER = 1500;

void printTestPage() {
    Serial.println("\nTest page HTML (save to file and open in browser):");
    Serial.println("----------------------------------------");
    Serial.println("<!DOCTYPE html>");
    Serial.println("<html>");
    Serial.println("<head>");
    Serial.println("    <title>ESP32 WiFi Test</title>");
    Serial.println("    <style>");
    Serial.println("        body { font-family: Arial, sans-serif; margin: 20px; }");
    Serial.println("        .status { padding: 10px; margin: 10px 0; }");
    Serial.println("        .connected { background: #d4edda; }");
    Serial.println("        .disconnected { background: #f8d7da; }");
    Serial.println("        #data { white-space: pre; font-family: monospace; }");
    Serial.println("    </style>");
    Serial.println("</head>");
    Serial.println("<body>");
    Serial.println("    <h1>ESP32 WiFi Test</h1>");
    Serial.println("    <div id=\"status\" class=\"status disconnected\">Disconnected</div>");
    Serial.println("    <button onclick=\"sendTestMessage()\">Send Test Message</button>");
    Serial.println("    <h2>Received Data:</h2>");
    Serial.println("    <div id=\"data\"></div>");
    Serial.println("    ");
    Serial.println("    <script>");
    Serial.println("        const ws = new WebSocket('ws://192.168.4.1:9090');");
    Serial.println("        ");
    Serial.println("        ws.onopen = () => {");
    Serial.println("            document.getElementById('status').textContent = 'Connected';");
    Serial.println("            document.getElementById('status').className = 'status connected';");
    Serial.println("        };");
    Serial.println("        ");
    Serial.println("        ws.onclose = () => {");
    Serial.println("            document.getElementById('status').textContent = 'Disconnected';");
    Serial.println("            document.getElementById('status').className = 'status disconnected';");
    Serial.println("        };");
    Serial.println("        ");
    Serial.println("        ws.onmessage = (event) => {");
    Serial.println("            const data = JSON.parse(event.data);");
    Serial.println("            document.getElementById('data').textContent = ");
    Serial.println("                JSON.stringify(data, null, 2);");
    Serial.println("        };");
    Serial.println("");
    Serial.println("        function sendTestMessage() {");
    Serial.println("            ws.send('Test message from client: ' + new Date().toISOString());");
    Serial.println("        }");
    Serial.println("    </script>");
    Serial.println("</body>");
    Serial.println("</html>");
    Serial.println("----------------------------------------");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
            
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            }
            break;
            
        case WStype_TEXT:
            Serial.printf("[%u] Received text: %s\n", num, payload);
            // Echo back the received message
            webSocket.sendTXT(num, payload, length);
            break;
    }
}

String formatTestData() {
    doc.clear();
    
    // Increment counter and wrap around
    testCounter = (testCounter + 10) % MAX_COUNTER;
    float normalizedCounter = testCounter / 15.0; // This will give us 0-100 range
    
    // IMU Array - cycling values
    JsonArray imu = doc.createNestedArray("IMU");
    imu.add(normalizedCounter);      // roll degrees
    imu.add(-normalizedCounter);     // pitch degrees
    imu.add(testCounter / 8.33);     // yaw degrees (0-180)
    imu.add(normalizedCounter / 2);  // roll velocity
    imu.add(-normalizedCounter / 2); // pitch velocity
    imu.add(normalizedCounter / 2);  // yaw velocity
    imu.add(testCounter % 4);        // system calibration (0-3)
    imu.add(testCounter % 4);        // gyro calibration
    imu.add(testCounter % 4);        // accel calibration
    imu.add(testCounter % 4);        // mag calibration
    
    // PID Output Array
    JsonArray pidOutput = doc.createNestedArray("PIDOutput");
    pidOutput.add(normalizedCounter);     // roll output
    pidOutput.add(-normalizedCounter);    // pitch output
    pidOutput.add(normalizedCounter / 2); // yaw output
    
    // Setpoints Array
    JsonArray setpoints = doc.createNestedArray("Setpoints");
    setpoints.add(normalizedCounter);     // roll setpoint
    setpoints.add(normalizedCounter);     // pitch setpoint
    setpoints.add(testCounter / 8.33);    // yaw setpoint (0-180)
    
    // PID Gains Array
    JsonArray pidGains = doc.createNestedArray("PIDGains");
    for(int i = 0; i < 9; i++) {
        pidGains.add(normalizedCounter / 100.0); // 0-1 range
    }
    
    // Receiver Values Array (1000-2000 range)
    JsonArray receiverValues = doc.createNestedArray("ReceiverValues");
    int pwmValue = 1000 + testCounter;  // Will cycle 1000-2500
    for(int i = 0; i < 6; i++) {
        receiverValues.add(pwmValue);
    }
    
    // Motor Values Array (1000-2000 range)
    JsonArray motorValues = doc.createNestedArray("MotorValues");
    for(int i = 0; i < 4; i++) {
        motorValues.add(pwmValue);
    }
    
    // Battery Life (0-100)
    doc["BatteryLife"] = testCounter % 101;  // 0-100
    
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 WiFi Test ===");

    // Configure WiFi
    Serial.println("Configuring WiFi AP...");
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);  // Disable power saving
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max power

    // Start AP
    if(WiFi.softAP(ssid, password, 1, 0, 4)) {  // channel 1, open network, max 4 connections
        Serial.println("WiFi AP started successfully");
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("Password: ");
        Serial.println(password);
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("MAC Address: ");
        Serial.println(WiFi.softAPmacAddress());
    } else {
        Serial.println("WiFi AP failed to start!");
    }

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started on port 9090");
    
    // Print the test page HTML
    printTestPage();
}

void loop() {
    webSocket.loop();

    // Print status every second
    if (millis() - lastUpdate >= UPDATE_INTERVAL) {
        Serial.println("\n=== WiFi Status ===");
        Serial.print("Connected clients: ");
        Serial.println(WiFi.softAPgetStationNum());
        Serial.print("TX Power: ");
        Serial.println(WiFi.getTxPower());
        Serial.print("Free heap: ");
        Serial.println(ESP.getFreeHeap());
        Serial.println("=================");

        // Send status to all connected WebSocket clients
        if (WiFi.softAPgetStationNum() > 0) {
            String jsonString = formatTestData();
            webSocket.broadcastTXT(jsonString);
        }
        
        lastUpdate = millis();
    }
} 