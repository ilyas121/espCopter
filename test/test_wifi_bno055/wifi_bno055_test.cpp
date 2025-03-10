#include "Arduino.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// WiFi Configuration
const char *ssid = "ESP32 Copter";
const char *password = "12345678";
WebSocketsServer webSocket(9090);
StaticJsonDocument<1024> doc;

// BNO055 Configuration
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
const int BNO055_RESET_PIN = 15;  // Reset pin for BNO055

// Timing variables
unsigned long lastUpdate = 0;
const int UPDATE_INTERVAL = 1000;  // 1 second between updates

// Add this global variable at the top with other globals
int testCounter = 0;
const int MAX_COUNTER = 1500;

// BNO055 sensor data
float orientation[3] = {0, 0, 0};  // x, y, z (roll, pitch, yaw)
float acceleration[3] = {0, 0, 0}; // x, y, z
float gyro[3] = {0, 0, 0};         // x, y, z
uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;

void printTestPage() {
    Serial.println("\nTest page HTML (save to file and open in browser):");
    Serial.println("----------------------------------------");
    Serial.println("<!DOCTYPE html>");
    Serial.println("<html>");
    Serial.println("<head>");
    Serial.println("    <title>ESP32 WiFi + BNO055 Test</title>");
    Serial.println("    <style>");
    Serial.println("        body { font-family: Arial, sans-serif; margin: 20px; }");
    Serial.println("        .status { padding: 10px; margin: 10px 0; }");
    Serial.println("        .connected { background: #d4edda; }");
    Serial.println("        .disconnected { background: #f8d7da; }");
    Serial.println("        #data { white-space: pre; font-family: monospace; }");
    Serial.println("        .sensor-data { margin: 20px 0; }");
    Serial.println("        .calibration { display: flex; gap: 10px; }");
    Serial.println("        .cal-item { padding: 5px; border: 1px solid #ccc; }");
    Serial.println("        .data-section { margin-bottom: 20px; }");
    Serial.println("    </style>");
    Serial.println("</head>");
    Serial.println("<body>");
    Serial.println("    <h1>ESP32 WiFi + BNO055 Test</h1>");
    Serial.println("    <div id=\"status\" class=\"status disconnected\">Disconnected</div>");
    Serial.println("    <button onclick=\"sendTestMessage()\">Send Test Message</button>");
    Serial.println("    <h2>BNO055 Sensor Data:</h2>");
    Serial.println("    <div class=\"sensor-data\">");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>Orientation (degrees):</h3>");
    Serial.println("            <div id=\"orientation\"></div>");
    Serial.println("        </div>");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>Angular Velocity (rad/s):</h3>");
    Serial.println("            <div id=\"angular-velocity\"></div>");
    Serial.println("        </div>");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>Calibration Status:</h3>");
    Serial.println("            <div class=\"calibration\">");
    Serial.println("                <div class=\"cal-item\">System: <span id=\"sys-cal\">0</span>/3</div>");
    Serial.println("                <div class=\"cal-item\">Gyro: <span id=\"gyro-cal\">0</span>/3</div>");
    Serial.println("                <div class=\"cal-item\">Accel: <span id=\"accel-cal\">0</span>/3</div>");
    Serial.println("                <div class=\"cal-item\">Mag: <span id=\"mag-cal\">0</span>/3</div>");
    Serial.println("            </div>");
    Serial.println("        </div>");
    Serial.println("    </div>");
    Serial.println("    <h2>Other Data:</h2>");
    Serial.println("    <div class=\"sensor-data\">");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>PID Output:</h3>");
    Serial.println("            <div id=\"pid-output\"></div>");
    Serial.println("        </div>");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>Setpoints:</h3>");
    Serial.println("            <div id=\"setpoints\"></div>");
    Serial.println("        </div>");
    Serial.println("        <div class=\"data-section\">");
    Serial.println("            <h3>Battery:</h3>");
    Serial.println("            <div id=\"battery\"></div>");
    Serial.println("        </div>");
    Serial.println("    </div>");
    Serial.println("    <h2>Raw JSON Data:</h2>");
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
    Serial.println("                ");
    Serial.println("            // Update IMU data display");
    Serial.println("            if (data.IMU) {");
    Serial.println("                // Orientation (first 3 values)");
    Serial.println("                document.getElementById('orientation').textContent = ");
    Serial.println("                    `Roll: ${data.IMU[0].toFixed(2)}°, ` +");
    Serial.println("                    `Pitch: ${data.IMU[1].toFixed(2)}°, ` +");
    Serial.println("                    `Yaw: ${data.IMU[2].toFixed(2)}°`;");
    Serial.println("                ");
    Serial.println("                // Angular velocity (next 3 values)");
    Serial.println("                document.getElementById('angular-velocity').textContent = ");
    Serial.println("                    `Roll: ${data.IMU[3].toFixed(2)} rad/s, ` +");
    Serial.println("                    `Pitch: ${data.IMU[4].toFixed(2)} rad/s, ` +");
    Serial.println("                    `Yaw: ${data.IMU[5].toFixed(2)} rad/s`;");
    Serial.println("                ");
    Serial.println("                // Calibration status (last 4 values)");
    Serial.println("                document.getElementById('sys-cal').textContent = data.IMU[6];");
    Serial.println("                document.getElementById('gyro-cal').textContent = data.IMU[7];");
    Serial.println("                document.getElementById('accel-cal').textContent = data.IMU[8];");
    Serial.println("                document.getElementById('mag-cal').textContent = data.IMU[9];");
    Serial.println("            }");
    Serial.println("            ");
    Serial.println("            // Update PID output display");
    Serial.println("            if (data.PIDOutput) {");
    Serial.println("                document.getElementById('pid-output').textContent = ");
    Serial.println("                    `Roll: ${data.PIDOutput[0].toFixed(2)}, ` +");
    Serial.println("                    `Pitch: ${data.PIDOutput[1].toFixed(2)}, ` +");
    Serial.println("                    `Yaw: ${data.PIDOutput[2].toFixed(2)}`;");
    Serial.println("            }");
    Serial.println("            ");
    Serial.println("            // Update setpoints display");
    Serial.println("            if (data.Setpoints) {");
    Serial.println("                document.getElementById('setpoints').textContent = ");
    Serial.println("                    `Roll: ${data.Setpoints[0].toFixed(2)}°, ` +");
    Serial.println("                    `Pitch: ${data.Setpoints[1].toFixed(2)}°, ` +");
    Serial.println("                    `Yaw: ${data.Setpoints[2].toFixed(2)}°`;");
    Serial.println("            }");
    Serial.println("            ");
    Serial.println("            // Update battery display");
    Serial.println("            if (data.BatteryLife !== undefined) {");
    Serial.println("                document.getElementById('battery').textContent = ");
    Serial.println("                    `${data.BatteryLife}%`;");
    Serial.println("            }");
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

void readBNO055Data() {
    // Get orientation data (Euler angles in degrees)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    orientation[0] = euler.x(); // Roll
    orientation[1] = euler.y(); // Pitch
    orientation[2] = euler.z(); // Yaw/Heading
    
    // Get accelerometer data (in m/s^2)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    acceleration[0] = accel.x();
    acceleration[1] = accel.y();
    acceleration[2] = accel.z();
    
    // Get gyroscope data (in rad/s)
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro[0] = gyroscope.x();
    gyro[1] = gyroscope.y();
    gyro[2] = gyroscope.z();
    
    // Get calibration status
    bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
}

String formatSensorData() {
    doc.clear();
    
    // Read sensor data
    readBNO055Data();
    
    // Increment counter and wrap around
    testCounter = (testCounter + 10) % MAX_COUNTER;
    float normalizedCounter = testCounter / 15.0; // This will give us 0-100 range
    
    // IMU Array - real values from BNO055
    JsonArray imu = doc.createNestedArray("IMU");
    imu.add(orientation[0]);      // roll degrees
    imu.add(orientation[1]);      // pitch degrees
    imu.add(orientation[2]);      // yaw degrees (0-180)
    imu.add(gyro[0]);             // roll velocity (rad/s)
    imu.add(gyro[1]);             // pitch velocity (rad/s)
    imu.add(gyro[2]);             // yaw velocity (rad/s)
    imu.add(sys_cal);             // system calibration (0-3)
    imu.add(gyro_cal);            // gyro calibration (0-3)
    imu.add(accel_cal);           // accel calibration (0-3)
    imu.add(mag_cal);             // mag calibration (0-3)
    
    // PID Output Array - dummy values
    JsonArray pidOutput = doc.createNestedArray("PIDOutput");
    pidOutput.add(normalizedCounter);     // roll output
    pidOutput.add(-normalizedCounter);    // pitch output
    pidOutput.add(normalizedCounter / 2); // yaw output
    
    // Setpoints Array - dummy values
    JsonArray setpoints = doc.createNestedArray("Setpoints");
    setpoints.add(normalizedCounter);     // roll setpoint
    setpoints.add(normalizedCounter);     // pitch setpoint
    setpoints.add(testCounter / 8.33);    // yaw setpoint (0-180)
    
    // PID Gains Array - dummy values
    JsonArray pidGains = doc.createNestedArray("PIDGains");
    for(int i = 0; i < 9; i++) {
        pidGains.add(normalizedCounter / 100.0); // 0-1 range
    }
    
    // Receiver Values Array (1000-2000 range) - dummy values
    JsonArray receiverValues = doc.createNestedArray("ReceiverValues");
    int pwmValue = 1000 + testCounter;  // Will cycle 1000-2500
    for(int i = 0; i < 6; i++) {
        receiverValues.add(pwmValue);
    }
    
    // Motor Values Array (1000-2000 range) - dummy values
    JsonArray motorValues = doc.createNestedArray("MotorValues");
    for(int i = 0; i < 4; i++) {
        motorValues.add(pwmValue);
    }
    
    // Battery Life (0-100) - dummy value
    doc["BatteryLife"] = testCounter % 101;  // 0-100
    
    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 WiFi + BNO055 Test ===");

    // Configure BNO055 reset pin
    pinMode(BNO055_RESET_PIN, OUTPUT);
    digitalWrite(BNO055_RESET_PIN, HIGH);  // Keep BNO055 enabled
    
    // Initialize I2C and BNO055
    Serial.println("Initializing BNO055 sensor...");
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055! Check your wiring or I2C address.");
        while (1) {
            Serial.println("BNO055 not detected. Retrying...");
            delay(1000);
        }
    }
    
    Serial.println("BNO055 initialized successfully!");
    delay(1000);
    
    // Set BNO055 to NDOF mode (9-DOF with fusion)
    bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);
    bno.setExtCrystalUse(true);
    
    // Display BNO055 system status
    uint8_t system_status, self_test_results, system_error;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
    Serial.println("BNO055 System Status: " + String(system_status));
    Serial.println("Self Test Results: " + String(self_test_results));
    Serial.println("System Error: " + String(system_error));
    
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
        
        // Read and display BNO055 data
        readBNO055Data();
        
        Serial.println("\n=== BNO055 Data ===");
        Serial.print("Orientation (deg): Roll=");
        Serial.print(orientation[0]);
        Serial.print(", Pitch=");
        Serial.print(orientation[1]);
        Serial.print(", Yaw=");
        Serial.println(orientation[2]);
        
        Serial.print("Angular Velocity (rad/s): Roll=");
        Serial.print(gyro[0]);
        Serial.print(", Pitch=");
        Serial.print(gyro[1]);
        Serial.print(", Yaw=");
        Serial.println(gyro[2]);
        
        Serial.print("Calibration: Sys=");
        Serial.print(sys_cal);
        Serial.print("/3, Gyro=");
        Serial.print(gyro_cal);
        Serial.print("/3, Accel=");
        Serial.print(accel_cal);
        Serial.print("/3, Mag=");
        Serial.print(mag_cal);
        Serial.println("/3");
        Serial.println("=================");

        // Send status to all connected WebSocket clients
        if (WiFi.softAPgetStationNum() > 0) {
            String jsonString = formatSensorData();
            webSocket.broadcastTXT(jsonString);
        }
        
        lastUpdate = millis();
    }
} 