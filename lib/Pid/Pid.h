#ifndef VPID_H
#define VPID_H
class VPID{
	//Tuneable parameters
	double k[3];
	double aggK[3];

	//Output&Input Limits
	double max;
	double min;
	double setpoint;
	double lastError;
 	double kiError;	
	//References to external motors
	double* input;
	double* output;

	void calculate();
	public:
		VPID(double* in, double* out, double kp, double ki, double kd);
};
#endif
