//
// 10/18 L3
// 10/18 BEMFを組み合わせた。 実際処理させる aPWMRef = pid(inSpeedCmd); は入れていない
// 10/18 CV3,CV4 を実装するため保留



//--------------------------------------------------------------------------------
// DCC Smile Function Decoder Motor
// [FunctionDecoderMotor.ino]
// スマイルデコーダ シリーズ の、Locomotive decoder sketch R4をベースにコメントを追加
// https://desktopstation.net/wiki/doku.php/ds_smile_decoder_r4
//
// Copyright (c) 2020 Ayanosuke(Maison de DCC) / Desktop Station
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

// DCC Decoder for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
// Debug serial output available on the serial port at baud 115200, aka Tools -> Serial Monitor
//

#include "NmraDcc.h"
#include "motor_ctrl.h"
#include "DccCV.h"
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below

//#define ONBRAKEPWM
#define DEBUG

//各種設定、宣言

#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 9	//if defined enables the ACK pin functionality. Comment out to disable.


#define MOTOR_LIM_MAX 255

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

//Task Scheduleg
unsigned long gPreviousL1 = 0;
unsigned long gPreviousL5 = 0;

//モータ制御関連の変数
uint32_t gPwmLPF_buf = 0;
uint32_t gPwmLPF2_buf = 0;
uint8_t gPwmDir = 128;
uint16_t gSpeedCmd = 0;
uint16_t gPrevSpeed = 0;

uint8_t gDecoderMode = 0; //2018/01/01　nagoden

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

/* Function assigned pins */
const int FunctionPin0 = 3;
const int FunctionPin1 = 4;
const int FunctionPin2 = 5;
const int FunctionPin3 = 6;
const int FunctionPin4 = 7;

struct CVPair{
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},		//The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},	 //XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},	 //YY in the XXYY address
  {CV_29_CONFIG, 128 },	 //Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_BEMFCUTOFF, 1},       // BEMF無効
  {CV_VSTART, 16},
  {CV_ACCRATIO, 1},
  {CV_DECCRATIO, 1},
  {CV_VMAX, 0},
  {CV_VMIDDLE, 125},
  {CV_BEMFcoefficient, 19},   //BEMF効果度合
  {CV_KP, 200},            //BEMF Pパラメータ
  {CV_KI, 100},            //BEMF Iパラメータ
  {CV_KD, 10},             //BEMF Dパラメータ
#if 0
  // メーカーピードテーブル
  {CV_ST1, 1},  //0 SpeedTable CV67-94 28
  {CV_ST2, 6},  //1
  {CV_ST3, 12}, //2
  {CV_ST4, 16}, //3
  {CV_ST5, 20}, //4
  {CV_ST6, 24}, //5
  {CV_ST7, 28}, //6
  {CV_ST8, 32},
  {CV_ST9, 36},
  {CV_ST10, 42},
  {CV_ST11, 48},
  {CV_ST12, 54},
  {CV_ST13, 60},
  {CV_ST14, 69},
  {CV_ST15, 78},
  {CV_ST16, 85},
  {CV_ST17, 92},
  {CV_ST18, 105},
  {CV_ST19, 118},
  {CV_ST20, 127},
  {CV_ST21, 136},
  {CV_ST22, 152},
  {CV_ST23, 168},
  {CV_ST24, 188},
  {CV_ST25, 208},
  {CV_ST26, 219},
  {CV_ST27, 240},
  {CV_ST28, 255},//27
#endif
  // 入替機スピードテーブル
  {CV_ST1, 20}, //0 SpeedTable CV67-94 28
  {CV_ST2, 21}, //1
  {CV_ST3, 22}, //2
  {CV_ST4, 23}, //3
  {CV_ST5, 26}, //4
  {CV_ST6, 29}, //5
  {CV_ST7, 30}, //6
  {CV_ST8, 31},
  {CV_ST9, 33},
  {CV_ST10, 35},
  {CV_ST11, 37},
  {CV_ST12, 39},
  {CV_ST13, 41},
  {CV_ST14, 43},
  {CV_ST15, 45},
  {CV_ST16, 48},
  {CV_ST17, 51},
  {CV_ST18, 55},
  {CV_ST19, 59},
  {CV_ST20, 63},
  {CV_ST21, 66},
  {CV_ST22, 70},
  {CV_ST23, 75},
  {CV_ST24, 82},
  {CV_ST25, 93},
  {CV_ST26, 112},
  {CV_ST27, 130},
  {CV_ST28, 155},//27
  
  };

void(* resetFunc) (void) = 0;  //declare reset function at address 0


uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);


void notifyCVResetFactoryDefault()
{
	//When anything is writen to CV8 reset to defaults. 

	resetCVToDefault();
#ifdef DEBUG	 
	Serial.println("Resetting...");
#endif
	delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

	resetFunc();
};

//------------------------------------------------------------------
// CVをデフォルトにリセット(Initialize cv value)
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  //CVをデフォルトにリセット
#ifdef DEBUG
	Serial.println("CVs being reset to factory defaults");
#endif	
	for (int j=0; j < FactoryDefaultCVIndex; j++ ){
		Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
	}
};

//------------------------------------------------------------------
// CV値が変化した時の処理（特に何もしない）OPSモードで書き込む
//------------------------------------------------------------------
extern void	   notifyCVChange( uint16_t CV, uint8_t Value){
   //CVが変更されたときのメッセージ

#ifdef DEBUG
   Serial.print("CV "); 
   Serial.print(CV); 
   Serial.print(" Changed to "); 
   Serial.println(Value, DEC);
#endif

  switch(CV){
    case 3:
            gCV3_AccRatio = Value;
            MOTOR_SetCV(3, gCV3_AccRatio);
            break;
    case 4:
            gCV4_DecRatio = Value;
            MOTOR_SetCV(4, gCV4_DecRatio);
            break;
    case 10:
            gCV10_BEMF_CutOff = Value;
            MOTOR_SetCV(10, gCV10_BEMF_CutOff);    
            break;
    case 54:
            gCV54_BEMFcoefficient = Value;
            MOTOR_SetCV(54, gCV54_BEMFcoefficient);    
            break;
    case 55:
            gCV55_KP = Value;
            MOTOR_SetCV(55, gCV55_KP);    
            break;
    case 56:
            gCV56_KI = Value;
            MOTOR_SetCV(56, gCV56_KI);    
            break;
    case 58:
            gCV58_KD = Value;
            MOTOR_SetCV(58, gCV58_KD);    
            break;
    default:
            break;
  }
};

//------------------------------------------------------------------
// CV Ackの処理
// そこそこ電流を流さないといけない
// 呼び出されるたびにFWD/REVを切り替える
//------------------------------------------------------------------
void notifyCVAck(void)
{
  MOTOR_Ack();
}


//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
#ifdef DEBUG
	//シリアル通信開始
	Serial.begin(115200);
#endif

	//ファンクションの割り当てピン初期化
	pinMode(FunctionPin0, OUTPUT);
	digitalWrite(FunctionPin0, 0);
	
	pinMode(FunctionPin1, OUTPUT);
	digitalWrite(FunctionPin1, 0);
	
	pinMode(FunctionPin2, OUTPUT);
	digitalWrite(FunctionPin2, 0);	
	
	pinMode(FunctionPin3, OUTPUT);
	digitalWrite(FunctionPin3, 0);
	
	pinMode(FunctionPin4, OUTPUT);
	digitalWrite(FunctionPin4, 0);
	
	
	//DCCの応答用負荷ピン
	
	#if defined(DCCACKPIN)
	//Setup ACK Pin
	//pinMode(DccAckPin,OUTPUT);
	//digitalWrite(DccAckPin, 0);
	#endif
  
   #if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
	if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ){	 //if eeprom has 0xFF then assume it needs to be programmed
	  Serial.println("CV Defaulting due to blank eeprom");
	  notifyCVResetFactoryDefault();
	  
   } else{
	 Serial.println("CV Not Defaulting");
   }
  #else
	 Serial.println("CV Defaulting Always On Powerup");
	 notifyCVResetFactoryDefault();
  #endif 
     
   
  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_MY_ADDRESS_ONLY , 0 ); 

  //Reset task
  gPreviousL1 = gPreviousL5 = millis();
  
  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;
  gCV2_Vstart = Dcc.getCV( CV_VSTART ) ;
  gCV3_AccRatio = Dcc.getCV( CV_ACCRATIO ) ;
  gCV4_DecRatio = Dcc.getCV( CV_DECCRATIO ) ;
  gCV5_VMAX = Dcc.getCV( CV_VMAX ) ;
  gCV6_VMIDDLE = Dcc.getCV( CV_VMIDDLE ) ;
  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );
  gCV29_Vstart = Dcc.getCV( CV_29_CONFIG ) ;
  gCV10_BEMF_CutOff = Dcc.getCV( CV_BEMFCUTOFF );
  gCV54_BEMFcoefficient = Dcc.getCV( CV_BEMFcoefficient );  // 係数
  gCV55_KP = Dcc.getCV(CV_KP);            // 比例制御（P制御）
  gCV56_KI = Dcc.getCV(CV_KI);            // 積分ゲイン(Iゲイン)
  gCV58_KD = Dcc.getCV(CV_KD);            // 微分ゲイン(Dゲイン)

  //cv29 Direction Check
  if(gCV29_Vstart & 0x01) {
    gCV29Direction = 1; //REVをFWDにする
  } else {
    gCV29Direction = 0;//FWDをFWDにする
  }
  if(gCV29_Vstart & 0x10) {
    gCV29SpeedTable = 1; //スピードテーブル有効
  } else {
    gCV29SpeedTable = 0; //スピードテーブル無効
  }
    
  MOTOR_Init();
  MOTOR_SetCV(2, gCV2_Vstart);
  MOTOR_SetCV(3, gCV3_AccRatio);
  MOTOR_SetCV(4, gCV4_DecRatio);
  MOTOR_SetCV(5, gCV5_VMAX);
  MOTOR_SetCV(6, gCV6_VMIDDLE);
  MOTOR_SetCV(10, gCV10_BEMF_CutOff);
  MOTOR_SetCV(29, gCV29_Vstart);
  MOTOR_SetCV(54, gCV54_BEMFcoefficient); // 係数
  MOTOR_SetCV(55, gCV55_KP);            // 比例制御（P制御）
  MOTOR_SetCV(56, gCV56_KI);            // 積分ゲイン(Iゲイン)
  MOTOR_SetCV(58, gCV58_KD);            // 微分ゲイン(Dゲイン)
  

  for( int i = 67 ; i <= 94 ; i++ ){      // SpeedTable EEPROMから読み込み
    if(gCV29SpeedTable == 1){
      MOTOR_SetCV( i, Dcc.getCV( i ));            // ユーザー設定テーブル
    } else {
      MOTOR_SetCV( i, LinerSpeedTable[ i-67 ] );  // メーカー設定テーブル
    }
  }

  if(gCV29SpeedTable == 1){                       // ユーザースピードテーブルが有効？
    MOTOR_SpeedMode(UserSpeedTableMode);          // ユーザースピードテーブルモード
  } else if(gCV2_Vstart < gCV6_VMIDDLE && gCV6_VMIDDLE < gCV5_VMAX){ // CV2,5,6 の関係が成立？
    MOTOR_SpeedMode(ThreePpointsTableMode);       // 三点スピードテーブルモード
  } else {
    MOTOR_SpeedMode(LinerSpeedTableMode);         // リニアスピードテーブルモード
  }
    
#ifdef DEBUG  
  Serial.print("CV1(ShortAddr): ");
  Serial.println(gCV1_SAddr);
  Serial.print("CV17/18(LongAddr): ");
  Serial.println(gCVx_LAddr);
  Serial.print("CV2(Vstart): ");
  Serial.println(gCV2_Vstart);
  Serial.print("CV3(AccRatio): ");
  Serial.println(gCV3_AccRatio);
  Serial.print("CV2(DecRatio): ");
  Serial.println(gCV4_DecRatio);
  Serial.print("gCV29SpeedTable:" );
  Serial.println(gCV29SpeedTable); 
  Serial.print("gCV10_BEMF_CutOff:" );
  Serial.println(gCV10_BEMF_CutOff); 
  Serial.print("gCV54_BEMFcoefficient:" );
  Serial.println(gCV54_BEMFcoefficient);    
  Serial.print("gCV55_KP:" );
  Serial.println(gCV55_KP);  
  Serial.print("gCV56_KI:" );
  Serial.println(gCV56_KI); 
  Serial.print("gCV58_KD:" );
  Serial.println(gCV58_KD);   
  Serial.println("Ready");
#endif
}

//---------------------------------------------------------------------
// Arduino main loop
//---------------------------------------------------------------------
void loop(){
	
	// You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
	Dcc.process();

  if( (millis() - gPreviousL1) >= 10){
    if(gCV10_BEMF_CutOff != 0){             // CV10 = 0 以外だったらBEMF処理のための誘起電圧測定を実行
      MOTOR_Sensor();
    }
    gPreviousL1 = millis();
  }

	if( (millis() - gPreviousL5) >= 100){
		//Motor drive control
		MOTOR_Main(gSpeedCmd, gPwmDir);

		//Reset task
		gPreviousL5 = millis();
	}
}





//---------------------------------------------------------------------
//DCC速度信号の受信によるイベント
//extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
//---------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
	
	uint16_t aSpeedRef = 0;
	
	//速度値の正規化(255を100%とする処理)
	if( Speed >= 1){
    aSpeedRef = ((Speed - 1) * 255) / SpeedSteps;
	}	else {
		aSpeedRef = 0;
	}
	
	//リミッタ
	if(aSpeedRef > 255) {
		aSpeedRef = 255;
	}
	
	gSpeedCmd = aSpeedRef;
	gPwmDir = Dir;

}

//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
//extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
extern void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
	switch(FuncGrp)
	{
		case FN_0_4:	//Function Group 1 F0 F4 F3 F2 F1
			exec_function( 0, FunctionPin0, (FuncState & FN_BIT_00)>>4 );
			exec_function( 1, FunctionPin1, (FuncState & FN_BIT_01));
			exec_function( 2, FunctionPin2, (FuncState & FN_BIT_02)>>1);
			exec_function( 3, FunctionPin3, (FuncState & FN_BIT_03)>>2 );
			exec_function( 4, FunctionPin4, (FuncState & FN_BIT_04)>>3 );
			break;

		case FN_5_8:	//Function Group 1 S FFFF == 1 F8 F7 F6 F5	&  == 0	 F12 F11 F10 F9 F8
			//exec_function( 5, FunctionPin5, (FuncState & FN_BIT_05));
			//exec_function( 6, FunctionPin6, (FuncState & FN_BIT_06)>>1 );
			//exec_function( 7, FunctionPin7, (FuncState & FN_BIT_07)>>2 );
			//exec_function( 8, FunctionPin8, (FuncState & FN_BIT_08)>>3 );
			break;

		case FN_9_12:
			//exec_function( 9, FunctionPin9,	(FuncState & FN_BIT_09));
			//exec_function( 10, FunctionPin10, (FuncState & FN_BIT_10)>>1 );
			//exec_function( 11, FunctionPin11, (FuncState & FN_BIT_11)>>2 );
			//exec_function( 12, FunctionPin12, (FuncState & FN_BIT_12)>>3 );
			break;

		case FN_13_20:	 //Function Group 2 FuncState == F20-F13 Function Control
			//exec_function( 13, FunctionPin13, (FuncState & FN_BIT_13));
			//exec_function( 14, FunctionPin14, (FuncState & FN_BIT_14)>>1 );
			//exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
			//exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
			break;

		case FN_21_28:
			break;
			
	}
}

void exec_function (int function, int pin, int FuncState)
{
	digitalWrite (pin, FuncState);
}
