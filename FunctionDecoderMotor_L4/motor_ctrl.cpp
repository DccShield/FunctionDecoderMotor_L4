

#include <Arduino.h>
#include "motor_ctrl.h"
//#define SmDecN18
//#define DEBUG             // シリアルにデバックメッセージを出力するときは // を外す
//#define ONBRAKEPWM      // ONブレーキタイプのモータデコーダを使用するときは // を外す　MP6513用


byte gDirection = 128;
int gSpeed_calculated = 0;
int gPWMRef = 0;

MOTOR_PARAM gParam;

//---------------------------------------------------------------------
// MOTOR_SetCV()
//---------------------------------------------------------------------
void MOTOR_SetCV(byte iNo, byte inData)
{
  switch(iNo){
    case 2:
            gParam.mStartVoltage = inData;
            break;
    case 3:
            gParam.mAccRatio = inData;
            break;
    case 4:
            gParam.mDecRatio = inData;
            break;
    case 5:
            gParam.mMaxVoltage = inData;
            break;

    case 6:
            gParam.mMiddleVoltage = inData;
            break;
    
    case 10:
            gParam.mBEMFcutoff = inData;
            break;

    case 54:
            gParam.mBEMFcoefficient = (float)inData / 10.0;
            break;

    case 55:
            gParam.mKp = (float)inData / 100.0;
            break;
  
    case 56:
            gParam.mKi = (float)inData / 100.0;
            break;

    case 57:
            gParam.mKd = (float)inData / 100.0;
            break;
                    
    case 29:
            gParam.mCV29 = inData;
            break;

    default:
            break;
  }
  
  if(iNo >= 67 && iNo <= 94){
    gParam.mSpeedStep[iNo - 67] = (unsigned int)inData;
    return;
  }
}

//---------------------------------------------------------------------
// MOTOR_Iniit()
// Motor control Task (10Hz)
//---------------------------------------------------------------------
void MOTOR_Init()
{
#ifdef SmDecN18
  //D3,D11 PWM キャリア周期:31kHz
  TCCR2B &= B11111000;
  TCCR2B |= B00000001;
  //PWM出力ピン D3,D11を出力にセット
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(MOTOR_PWM_A, OUTPUT);
#else
  //D9,D10 PWM キャリア周期:31kHz
  TCCR1B &= B11111000;
  TCCR1B |= B00000001;
  //PWM出力ピン D9,D10を出力にセット
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(MOTOR_PWM_A, OUTPUT);
#endif
  gParam.mSpeedStep[28]=255;  // SpeedStepは28までなので、32に調整するため255で埋める
  gParam.mSpeedStep[29]=255;
  gParam.mSpeedStep[30]=255;
  gParam.mSpeedStep[31]=255;  
  gParam.mSpeedStep[32]=255;  
}

//byte lerp(byte x0, byte y0, byte x1, byte y1, byte x) {
byte lerp(int x0, int y0, int x1, int y1, int x) {

#if 0
  Serial.print(",x0:");
  Serial.print(x0);
  Serial.print(",y0:");
  Serial.print(y0);
  Serial.print(",x1:");
  Serial.print(x1);
  Serial.print("y1:");
  Serial.print(y1);
  Serial.print(",x:");
  Serial.print(x);
#endif
  
  return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0);
}



byte pid( int spd )
{
  float aP = 0;
  static float aI = 0;
  static float aD = 0;
  static float preP = 0;
  static int pwn = 0;

  aP = spd - gSpeed_calculated; // スロット値 - 誘起電圧（スケーリング済）
  aI = aI + (aP * 0.1);
  aD = ( aP - preP ) / 0.1;
  preP = aP;

  pwn = pwn + (gParam.mKp * aP + gParam.mKi * aI + gParam.mKd * aD);

 #if DEBUG
  Serial.print(",aSpeed:");
  Serial.print(gSpeed_calculated);
  Serial.print(",aP:");
  Serial.print(aP);
  Serial.print(",aI:");
  Serial.print(aI);
  Serial.print(",aD:");
  Serial.print(aD);
#endif

  if(pwn < 0){
    aP = 0;
    aI = 0;
    aD = 0;
    preP = 0;
    pwn = 0;
  }
  return pwn;
}

//---------------------------------------------------------------------
// Motor Main Task (10Hz)
// https://haizairenmei.com/2018/10/27/arduino_noise/  rc
//---------------------------------------------------------------------
void MOTOR_Main(byte inSpeedCmd, byte inDirection)
{  
  byte aPWMRef = 0;
  byte aPidPWMRef = 0;
  static int aNowPWMRef = 0; 
  byte aSubcal = inSpeedCmd >> 3;  // inSpeedCmd/8
  static int f = 0;
  static int aDelay = 0;

//  if(inDirection > 0){          // CV3,CV4の処理
//    aDelay = gParam.mAccRatio;  // 増加率
//  } else {
//    aDelay = gParam.mDecRatio;  // 減少率
//  }

#if DEBUG
  Serial.print("aSubcal:");
  Serial.print((int)aSubcal);
  Serial.print(",inSpeedCmd");
  Serial.print(inSpeedCmd);
#endif

if(f==0){
  Serial.print("gParam.mSpeedMode:");
  Serial.print(gParam.mSpeedMode);
  Serial.print(",gParam.mStartVoltage:");
  Serial.print(gParam.mStartVoltage);
  Serial.print(",gParam.mMiddleVoltage:");
  Serial.print(gParam.mMiddleVoltage);
  Serial.print(",gParam.mMaxVoltage:");
  Serial.println(gParam.mMaxVoltage);
  f=1;
}

//  Serial.print(",gParam.mSpeedStep[aSubcal]");
//  Serial.print(gParam.mSpeedStep[aSubcal]);
//  Serial.print(",gParam.mSpeedStep[aSubcal+1]");
//  Serial.print(gParam.mSpeedStep[aSubcal+1]);

//  Serial.print(",gParam.mSpeedMode:");
//  Serial.print(gParam.mSpeedMode);

  if(gParam.mSpeedMode == LinerSpeedTableMode || gParam.mSpeedMode == UserSpeedTableMode){         // SpeedTable有効
    aPWMRef = lerp(aSubcal*8,gParam.mSpeedStep[aSubcal],(aSubcal+1)*8,gParam.mSpeedStep[aSubcal+1],inSpeedCmd);    

#if 0
  Serial.print(",aSubcal*8:");
  Serial.print(aSubcal*8);
  Serial.print(",gParam.mSpeedStep[aSubcal]:");
  Serial.print(gParam.mSpeedStep[aSubcal]);
  Serial.print(",(aSubcal+1)*8:");
  Serial.print((aSubcal+1)*8);
  Serial.print(",gParam.mSpeedStep[aSubcal+1]:");
  Serial.print(gParam.mSpeedStep[aSubcal+1]);
#endif
  
  } else {                         // 3点SpeedTable有効
  
    if(gParam.mStartVoltage <= inSpeedCmd && gParam.mMiddleVoltage >= inSpeedCmd){  // スタート電圧〜中間電圧間を補間
//      aPWMRef = lerp(aSubcal*8,gParam.mStartVoltage,(aSubcal+1)*8,gParam.mMiddleVoltage,inSpeedCmd);        
      aPWMRef = lerp(1,gParam.mStartVoltage,127,gParam.mMiddleVoltage,inSpeedCmd);    
    } else if(gParam.mMiddleVoltage < inSpeedCmd && gParam.mMaxVoltage >= inSpeedCmd){  // 中間電圧間〜最大電圧間を補間
      aPWMRef = lerp(127,gParam.mMiddleVoltage,255,gParam.mMaxVoltage,inSpeedCmd);  
    }
  }
  
#if DEBUG  
  Serial.print(",aPWMRef:");
  Serial.print(aPWMRef);
  Serial.print(",aNowPWMRef:");
  Serial.print(aNowPWMRef);
#endif

  if(aNowPWMRef < aPWMRef){         // 増加率計算の場合
    if(gParam.mAccRatio != 0 ){
      aDelay++;
      if(aDelay < gParam.mAccRatio){  // Delay時間に達していなかったら抜ける
        return;      
      } else {
        aDelay = 0;                   // Delay時間に達していたら、クリアする。
        aNowPWMRef++; 
        if(aNowPWMRef >= aPWMRef)
          aNowPWMRef = aPWMRef;
      }
    } else {
    aNowPWMRef = aPWMRef;
    }
  }

  if(aNowPWMRef > aPWMRef) {                          // 減少率計算の場合
    if(gParam.mDecRatio != 0 ){
      aDelay++;
      if(aDelay < gParam.mDecRatio){  // Delay時間に達していなかったら抜ける
        return;      
      } else {
        aDelay = 0;                   // Delay時間に達していたら、クリアする。
        aNowPWMRef--;
        if(aNowPWMRef <= aPWMRef)
          aNowPWMRef = aPWMRef;
      }
    } else {
    aNowPWMRef = aPWMRef;
    }
  }
  
 #if 0  
  // 3点スピードテーブルで定義しているからいらない？
   if(gParam.mStartVoltage >= aNowPWMRef){ // CV2 スタート電圧以下はカット
      aNowPWMRef = gParam.mStartVoltage;
    }
    if(gParam.mMaxVoltage <= aNowPWMRef){ // CV5 最大電圧以上はカット
      aNowPWMRef = gParam.mMaxVoltage;
    }
#endif

  aPidPWMRef = pid(aNowPWMRef);

  Serial.print(",aPidPWMRef:");
  Serial.println(aPidPWMRef);
  
  if(inSpeedCmd == 0){  // 速度0の時はPWMも0(ofF)
    aNowPWMRef  = 0;
  }
    //進行方向でPWMのABを切り替える
    if( inDirection > 0){
      #ifdef ONBRAKEPWM
        analogWrite(MOTOR_PWM_B, 255);            //Change  by MP6513.
        analogWrite(MOTOR_PWM_A, 255 - aPidPWMRef);  //Change  by MP6513.
      #else
        analogWrite(MOTOR_PWM_B, 0);
        analogWrite(MOTOR_PWM_A, aPidPWMRef);
      #endif
    } else {
      #ifdef ONBRAKEPWM
        analogWrite(MOTOR_PWM_A, 255);            //Change  by MP6513.
        analogWrite(MOTOR_PWM_B, 255 - aPidPWMRef);  //Change  by MP6513.
      #else
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, aPidPWMRef);
      #endif
    }
  

#ifdef DEBUG1
  Serial.print("Spd:");
  Serial.print(inSpeedCmd);
  Serial.print(",aSubcal:");
  Serial.print(aSubcal);
  Serial.print(",gParam.mS:");
  Serial.print(gParam.mSpeedStep[aSubcal]);
  Serial.print(",aPWMRef:");
  Serial.println(aPWMRef);
#endif

  gDirection = inDirection;
  gPWMRef = aPidPWMRef;
  
}


//---------------------------------------------------------------------
// MOTOR_AckV()
// デコーダーの応答用関数
// PWM_A と PWM_B を交互に出力するように変更
//---------------------------------------------------------------------
void MOTOR_Ack(void)
{
  static char aFwdRev = 0;
  
  #ifdef DEBUG
  Serial.println("notifyCVAck");
  #endif
  
  if(aFwdRev == 0) {
  #if defined ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 30); 
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 255);
  #else
    analogWrite(MOTOR_PWM_B, 0);
    analogWrite(MOTOR_PWM_A, 50);
    delay( 6 );  
    analogWrite(MOTOR_PWM_A, 0);
  #endif
    aFwdRev = 1;
  } else {
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 30);
    analogWrite(MOTOR_PWM_A, 255);
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 255);
  #else
    analogWrite(MOTOR_PWM_B, 50);
    analogWrite(MOTOR_PWM_A, 0);
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 0);
  #endif
    aFwdRev = 0;  
  }
}
//---------------------------------------------------------------------
// MOTOR_SpeedMode(byte inMode)
// 0:リニアスピードテーブル/1:ユーザースピードテーブル/2:3点スピードテーブルかの設定
//---------------------------------------------------------------------
void MOTOR_SpeedMode(byte inMode)
{
  if(inMode >= 3){          // 3以上は無効 
    gParam.mSpeedMode = 0;  
    return;
  }
  gParam.mSpeedMode = inMode;
}


//---------------------------------------------------------------------
// MOTOR_Sensor()
//---------------------------------------------------------------------
void MOTOR_Sensor()
{
  int aBEMF_result = 0 ;           // BEMF enhance by MECY 2017/09/17
  
  //BEMF detection

  aBEMF_result = MOTOR_GetBEMF();
  
   
  gSpeed_calculated = (float)aBEMF_result / gParam.mBEMFcoefficient ;  // 計測したした誘起電圧をCV54の1/10の定数でPWM値に合わせる。

#if 0
  Serial.print(",gPWMRef:");
  Serial.print(gPWMRef);
  Serial.print(",gBEMF:");
  Serial.print(gBEMF);
  Serial.print(",aSpeed_calculated:");
  Serial.println(aSpeed_calculated);
#endif
  
  if(gSpeed_calculated > 255)
  {
    gSpeed_calculated = 255;
  }
  
  //Speed conversion
 // gMotorSpeed = MOTOR_LPF(aSpeed_calculated, 8, &gBEMFLPF_buf);
}

int MOTOR_GetBEMF()
{
//  if( gParam.mBEMFcutoff > 0){
  uint32_t aAve = 0;
  int i;
  
  //Stop PWM
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 255);  //Change  by MP6513.
    analogWrite(MOTOR_PWM_A, 255);  //Change  by MP6513.
  #else
    analogWrite(MOTOR_PWM_B, 0);
    analogWrite(MOTOR_PWM_A, 0);
  #endif
  
  delayMicroseconds(300);
  
  for( i = 0; i < 8; i++){

   aAve = aAve + analogRead(A0);
   delayMicroseconds(30);

  }

  //MaxValueCheck
    if( gPWMRef >= 255)
    {
        gPWMRef = 255;
    }
  
  //進行方向でPWMのABを切り替える
  if( gDirection > 0){
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 255);            //Change  by MP6513.
    analogWrite(MOTOR_PWM_A, 255 - gPWMRef);  //Change  by MP6513.
  #else
    analogWrite(MOTOR_PWM_B, 0);
    analogWrite(MOTOR_PWM_A, gPWMRef);
  #endif
  }
  else
  {
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_A, 255);            //Change  by MP6513.
    analogWrite(MOTOR_PWM_B, 255 - gPWMRef);  //Change  by MP6513.
  #else
    analogWrite(MOTOR_PWM_A, 0);
    analogWrite(MOTOR_PWM_B, gPWMRef);  
  #endif
  }
  
  return (aAve >> 3);
  
//}
}
