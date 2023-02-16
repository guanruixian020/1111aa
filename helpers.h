/*
 * new
 * pn 003 1
 * pn 097 1
 * pn 098 50
 * pn 109 2
 * pn 112 50
 * **
 * old
 * pn 098 10
 * pn 110 30
 * pn 109 1
 * pn 113 20
 * pn 114 10
*/
/*
 * 856  2047 3238
 * 856  2047 3238 
 * */

#include <math.h>

#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define PAUSECHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"   ////接收按钮值
#define FILTERCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a9"  //接收位置信息
#define POSITIONCHARACTERISTIC_UUID "aeb5483e-36e1-4688-b7f5-ea07361b26a4"  //发送位置信息
#define NAMESPACE "6dofPrefv1"
#define AXIS1_KEY "Axis1"
#define AXIS2_KEY "Axis2"
#define AXIS3_KEY "Axis3"
#define AXIS4_KEY "Axis4"
#define AXIS5_KEY "Axis5"
#define AXIS6_KEY "Axis6"

#define ESTOPPIN 22
#define ESTOPDEBOUNCETIME 10

#define MICRO_INTERVAL_FAST  10 
#define MICRO_INTERVAL_SLOW  20000
#define MICRO_INTERVAL_BLE_SEND  100000
#define ESTOP_RATE_LIMIT_TIME 3000

#define MAX_INPUT 60

extern volatile float arrble[6];  //存储位置信息
extern volatile bool zt[4];
//extern volatile bool bflag;

struct acServo {
  int stepPin;
  int dirPin;
  bool pinState;
  long currentpos;
  long targetpos;  
};

float rateLimit(float currentSample,float lastSample);
float mapfloat(double x, double in_min, double in_max, double out_min, double out_max);
//float minfloat(double x, double in_min, double in_max, double out_min, double out_max);
void  blerawdata();
