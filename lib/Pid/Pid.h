class VPID{
	//Tuneable parameters
	float k[3];
	float aggK[3];

	//Output&Input Limits
	float max;
	float min;
	float setpoint;
	float lastError;
 	float kiError;	
	//References to external motors
	float* input;
	float* output;

	void calculatePid();
	VPID(float* in, float* out, float kp, float ki, float kd);
};
