

//#define MOTOR_PWM_A 3
//#define MOTOR_PWM_B 11

#define MOTOR_PWM_A 9
#define MOTOR_PWM_B 10

#define LinerSpeedTableMode 0
#define UserSpeedTableMode 1
#define ThreePpointsTableMode 2

typedef struct _MOTOR_PARAM
{
  byte mSpeedMode;
  byte mStartVoltage;
  byte mMaxVoltage;
  byte mMiddleVoltage;
  byte mCV29;
  byte mAccRatio;
  byte mDecRatio;
  byte mBEMFcutoff;
  float mBEMFcoefficient;
  float mKp; 
  float mKi;
  float mKd; 
  unsigned int mSpeedStep[33];
} MOTOR_PARAM;

extern void MOTOR_Init();
extern void MOTOR_SetCV(byte iNo, byte inData);
extern void MOTOR_Ack(void);
extern void MOTOR_Main(byte inSpeedCmd, byte inDirection);
extern void MOTOR_SpeedMode(byte inMode);
extern int MOTOR_GetBEMF();
extern void MOTOR_Sensor();
