#include "Drone.h"

void Drone::loop() {
	if (esp_timer_get_time() - lastPrint > 500
			|| esp_timer_get_time() < lastPrint // check for the wrap over case
					) {
		switch (state) {
		case Startup:
			setup();
			state = WaitForConnect;
			break;
		case WaitForConnect:
#if defined(USE_WIFI)
			if (manager.getState() == Connected)
#endif
				state = readGame; // begin the main operation loop
			break;
		case readGame:
			runGameControl();
			state = readIMU;
			break;
		case readIMU:
#if defined(USE_IMU)
			sensor->loop();
//			if (PRINTROBOTDATA)
//				sensor->print();
#endif
			state = readIR;
			break;
		case readIR:
#if defined(USE_IR_CAM)
			serverIR->loop();
			if (PRINTROBOTDATA)
				serverIR->print();
#endif
			state = readGame;    // loop back to start of sensors
			break;

		}
		printAll();    // Print some values in a slow loop
		lastPrint = esp_timer_get_time(); // ensure 0.5 ms spacing *between* reads for Wifi to transact
	}
	// If this is run before the sensor reads, the I2C will fail because the time it takes to send the UDP causes a timeout
	fastLoop();    // Run PID and wifi after State machine on all states

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
ExampleRobot::ExampleRobot(String * mn) {
	pidList[0] = &motor1;
	pidList[1] = &motor2;
	state = Startup;
#if defined(	USE_WIFI)
#if defined(USE_IMU)
	sensor = NULL;
#endif
#if defined(USE_IR_CAM)
	serverIR = NULL;
#endif
#endif
	name = mn;
	serverIR=NULL;
	sensor= NULL;
}

ExampleRobot::~ExampleRobot() {
	// TODO Auto-generated destructor stub
}
void ExampleRobot::setupPIDServers(){
#if defined(	USE_WIFI)
	coms.attach(new PIDConfigureSimplePacketComsServer(numberOfPID,pidList));
	coms.attach(new GetPIDData(numberOfPID,pidList));
	coms.attach(new GetPIDConfigureSimplePacketComsServer(numberOfPID,pidList));
#endif

}
void ExampleRobot::setup() {
	if (state != Startup)
		return;
	state = WaitForConnect;
#if defined(USE_WIFI)
	manager.setup();
#else
	Serial.begin(115200);
#endif
	delay(100);
	motor1.attach(MOTOR1_PWM, MOTOR1_DIR, MOTOR1_ENCA, MOTOR1_ENCB);
	motor2.attach(MOTOR2_PWM, MOTOR2_DIR, MOTOR2_ENCA, MOTOR2_ENCB);
	Serial.println("Starting Motors");

	// Create a module to deal with the demo wrist bevel gears
	wristPtr = new GearWrist(&motor1, //right motor
			&motor2, // left motor
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					2.5347 * // Wrist gear stage ratio
					(1.0 / 360.0) * // degrees per revolution
					motor1.encoder.countsMode, // full quadrature, 4 ticks be encoder count, half is 2 and single mode is one
			0.8932); // ratio of second stage to first stage
	// Set up digital servos
	panEyes.setPeriodHertz(330);
	panEyes.attach(SERVO_PAN, 1000, 2000);
	jaw.setPeriodHertz(330);
	jaw.attach(SERVO_JAW, 1000, 2000);
	tiltEyes.setPeriodHertz(330);
	tiltEyes.attach(SERVO_TILT, 1000, 2000);

//	// Create sensors and servers
#if defined(USE_IMU)
	sensor = new GetIMU();
	/* Initialise the sensor */
	if (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print(
				"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(1000);
		while (1)
			;
	}

	delay(1000);
	bno.setExtCrystalUse(true);
	sensor->startSensor(&bno);
#endif

#if defined(USE_IR_CAM)
	myDFRobotIRPosition.begin();
#endif
#if defined(USE_WIFI)
	// Attach coms
#if defined(USE_IMU)
	coms.attach(sensor);
#endif
#if defined(USE_IR_CAM)
	serverIR = new IRCamSimplePacketComsServer(&myDFRobotIRPosition);
	coms.attach(serverIR);
#endif
	coms.attach(new NameCheckerServer(name));
	setupPIDServers();
#endif
#if defined(USE_GAME_CONTOL)
	control.begin();
	control.readData();    // Read inputs and update maps
#endif
}

void ExampleRobot::fastLoop() {
	if (state == Startup)    // Do not run before startp
		return;
#if defined(USE_WIFI)
	manager.loop();
	if (manager.getState() == Connected)
		coms.server();
	else {
		return;
	}
#endif
	wristPtr->loop();

}

void ExampleRobot::runGameControl() {

}
