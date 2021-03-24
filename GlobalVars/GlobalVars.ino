#include <KF.h>
#include <QUADPID.h>
#include <MPU6050.h>
// global variables
float dt;
float ct;
float lt;// last time
float  sensorReturn[6];
boolean started = -1;
float minControl = -400;
float maxControl = 400;
float FREQ = 250;
float period = (1.0 / FREQ) ;
#define m1 4
#define m2 5
#define m3 6
#define m4 7

#define ROLL 0
#define PITCH 1
#define YAW 2
float references[3] = {0, 0, 0};
float throttle = 1000;
QUADPID ctrlr;
KF kf;
MPU6050 mpu;
// kalman initialization
float xroll[2] = {0.1, 0.1};
float Proll[4] = {0.1 , 0, 0, 0.1};
float xpitch[2] = {0.1, 0.1};
float Ppitch[4] = {0.1 , 0, 0, 0.1};
float xyaw = {0};
float  A[4] = {1, dt/1000000, 0 , 1};
float B[2] = {dt/1000000 , 0};
float C[2] = {1, 0};
float Q[4] = {0.001,0, 0, 0.003};
float R = 0.3;

// got this at https://www.rcgroups.com/forums/showthread.php?2478885-ZMR250-Lux-Float-PID-settings-for-3S-and-4S
//float kp[3] = {1.5, 2.1, 6.5};
//float ki[3] = {0.04,0.048,0.045};
//float kd[3] = {16,18,50};

float kp[3] = {2.5, 2.5, 2.5};
float ki[3] = {0.06,0.06,0.05};
float kd[3] = {20,20,20};
