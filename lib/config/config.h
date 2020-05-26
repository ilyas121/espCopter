// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#ifndef CONFIG_H
#define CONFIG_H

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define UPPER_LEFT_MOTOR 18
#define UPPER_RIGHT_MOTOR 16
#define LOWER_LEFT_MOTOR 19
#define LOWER_RIGHT_MOTOR 17

#define USE_IMU true

#define PID_DATA_GET_PORT 1234
#define IMU_EULER GET_PORT 6969

extern const char* loginIndex;
extern const char* serverIndex;
#endif
