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
#define USE_IMU false

extern const char* loginIndex;
extern const char* serverIndex;
#endif
