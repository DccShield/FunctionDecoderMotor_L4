//--------------------------------------------------------------------------------
// DCC Smile Function Decoder Do Nothing
// DCC信号を受信するだけで何もしないデコーダ
// [DccCV.H]
// Copyright (c) 2020 Ayanosuke(Maison de DCC) / Desktop Station
// https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

//#ifndef _DccCV_h_
//#define _DccCV_h_

#pragma once

#define DECODER_ADDRESS 3

#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0

#define CV_VSTART    2           //スタート電圧
#define CV_ACCRATIO   3          //加速率
#define CV_DECCRATIO  4          //減速率
#define CV_VMAX 5                //最高電圧
#define CV_VMIDDLE  6            //中間電圧
#define CV_BEMFCUTOFF 10         //BEMF カットオフ値 0:off 1:〜 on
#define CV_F0_FORWARD 33
#define CV_F0_BACK 34
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_49_F0_FORWARD_LIGHT 49
#define CV_DIMMING_SPEED 50
#define CV_DIMMING_LIGHT_QUANTITY 51
#define CV_ROOM_DIMMING 52

#define CV_BEMFcoefficient  54   //BEMF効果度合
#define CV_KP 55               //BEMF Pパラメータ
#define CV_KI 56               //BEMF Iパラメータ
#define CV_KD 58               //BEMF Dパラメータ

#define CV_ST1 67
#define CV_ST2 68
#define CV_ST3 69
#define CV_ST4 70
#define CV_ST5 71
#define CV_ST6 72
#define CV_ST7 73
#define CV_ST8 74
#define CV_ST9 75
#define CV_ST10 76
#define CV_ST11 77
#define CV_ST12 78
#define CV_ST13 79
#define CV_ST14 80
#define CV_ST15 81 
#define CV_ST16 82
#define CV_ST17 83
#define CV_ST18 84
#define CV_ST19 85
#define CV_ST20 86
#define CV_ST21 87
#define CV_ST22 88
#define CV_ST23 89
#define CV_ST24 90
#define CV_ST25 91
#define CV_ST26 92
#define CV_ST27 93
#define CV_ST28 94



#define MAN_ID_NUMBER 166   // Manufacture ID //
#define MAN_VER_NUMBER  01  // Release Ver CV07 //



//Function State
uint8_t gState_F0 = 0;
uint8_t gState_F1 = 0;
uint8_t gState_F2 = 0;
uint8_t gState_F3 = 0;
uint8_t gState_F4 = 0;
uint8_t gState_F5 = 0;
uint8_t gState_F6 = 0;
uint8_t gState_F7 = 0;
uint8_t gState_F8 = 0;
uint8_t gState_F9 = 0;
uint8_t gState_F10 = 0;
uint8_t gState_F11 = 0; 
uint8_t gState_F12 = 0;
uint8_t gState_F13 = 0;
uint8_t gState_F14 = 0; 
uint8_t gState_F15 = 0;
uint8_t gState_F16 = 0;
uint8_t gState_Function = 0;

//CV related
uint8_t gCV1_SAddr = 3; 
uint8_t gCVx_LAddr = 3;
uint8_t gCV2_Vstart = 16;
uint8_t gCV3_AccRatio = 0;
uint8_t gCV4_DecRatio = 0;
uint8_t gCV5_VMAX = 250;
uint8_t gCV6_VMIDDLE = 125;
uint8_t gCV29_Vstart = 0;
uint8_t gCV29Direction = 0; // bit0
uint8_t gCV29SpeedTable = 0; // bit4
uint8_t gCV49_fx = 20;

// BEMF パラメータ
uint8_t gCV10_BEMF_CutOff = 1;        // 0でBEMF Off 1でBEMF On
uint8_t gCV54_BEMFcoefficient = 64;
uint8_t gCV55_KP = 18;              // Reajust by MECY 2017/10/08
uint8_t gCV56_KI = 96;
uint8_t gCV58_KD = 96;

 
uint8_t LinerSpeedTable[33] = {   1,   6,  12,  16,  20,  24,  28,  32,  36,  42,
                                 48,  54,  60,  69,  78,  85,  92, 105, 118, 127,
                                136, 152, 168, 188, 208, 219, 240, 255 };


  
// #endif
