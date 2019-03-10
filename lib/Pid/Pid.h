#ifndef VPID_H
#define VPID_H
class VPID{
	//Tuneable parameters
	double k[3];
	double aggK[3];

	//Output&Input Limits
	double max = 400;
	double min = 400;
	double setpoint = 0;
	double lastError;
 	double kiError;	
	//References to external motors
	double* input;
	double* output;

	public:
		void calculate();
		VPID(double* in, double* out, double kp, double ki, double kd);
};
#endif
