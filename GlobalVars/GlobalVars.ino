#include <KF.h>
#include <QUADPID.h>
#include <MPU6050.h>
// global variables
float sensorReturn[6];
boolean started = -1;
float minControl = -400;
float maxControl = 400;
float FREQ = 250;
float period = (1.0 / FREQ) ;
const int m1 = 4;
const int m2 = 5;
const int m3 = 6;
const int m4 = 7;
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
float  A[4] = {1, -1/250, 0 , 1};
float B[2] = {1/250 , 0};
float C[2] = {1, 0};
float Q[4] = {0.002,0, 0, 0};
float R = 0.0003;
float kp[3] = {1, 1, 1};
float ki[3] = {1, 1, 1};
float kd[3] = {1, 1, 1};