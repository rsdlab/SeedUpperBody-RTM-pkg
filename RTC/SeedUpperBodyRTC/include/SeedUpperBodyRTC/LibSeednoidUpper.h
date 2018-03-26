//LibSeedonoidUpper.h
#ifndef __LIBSEEDNOIDUPPER_H__
#define __LIBSEEDNOIDUPPER_H__


//_USE_SERIALCOM_CLASS  シリアル通信にSerialComClassを使用する。



//数学定数の使用
#define _USE_MATH_DEFINES

//SerialComClassの使用
//#define _USE_SERIALCOM_CLASS


/********* includeファイル *********/
//#include <cmath>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include "ikfast.h"
#include <fstream>

#ifdef _USE_SERIALCOM_CLASS
#include "SerialCom.h"
#else
#include <sys/types.h>
 #include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#endif

typedef unsigned char uchar;

#define SEND_DATALENGTH 60
#define SEED_FREEDOM 30
#define ARM_FREEDOM 7

//腕関節の可動範囲[deg]
//アーム第1軸モータの可動範囲
#define	Angle1_LimitMax	89
#define	Angle1_LimitMin	-20
//アーム第2軸モータの可動範囲
#define	Angle2_LimitMax	85
#define	Angle2_LimitMin 0
//アーム第3軸モータの可動範囲
#define	Angle3_LimitMax	135
#define	Angle3_LimitMin	-135
//アーム第4軸モータの可動範囲
#define	Angle4_LimitMax	180
#define	Angle4_LimitMin	0
//アーム第5軸モータの可動範囲
#define Angle5_LimitMax 135
#define Angle5_LimitMin -135
//アーム第6軸モータの可動範囲
#define Angle6_LimitMax 25
#define Angle6_LimitMin -25
//アーム第7軸モータの可動範囲
#define Angle7_LimitMax 40
#define Angle7_LimitMin -80
//頭ロール（左右）の可動範囲
#define NeckRoll_LimitMax 20
#define NeckRoll_LimitMin -20
//頭ピッチ（前後）の可動範囲
#define NeckPitch_LimitMax 60
#define NeckPitch_LimitMin -20
//頭ヨー（回転）の可動範囲
#define NeckYaw_LimitMax 90
#define NeckYaw_LimitMin -90
//腰ロール（左右）の可動範囲
#define WaistRoll_LimitMax 8
#define WaistRoll_LimitMin -8
//腰ピッチ（前後）の可動範囲
#define WaistPitch_LimitMax 39
#define WaistPitch_LimitMin -9
//腰ヨー（回転）の可動範囲
#define WaistYaw_LimitMax 90
#define WaistYaw_LimitMin -90

//ハンドの可動範囲
#define HAND_LIMITMAX 0
#define HAND_LIMITMIN 1800

//ジョイントリミットの構造体
typedef struct
{
  short Upper;
  short Lower;
}JLimit;

//ロールピッチヨー
typedef struct
{
  double roll;
  double pitch;
  double yaw;
}Frame;

class Seednoid{
 private:
  
  //private変数の宣言
  int noidJointPos[SEED_FREEDOM];
  JLimit SeedArmJointLimit[7];
  JLimit SeedNeckJointLimit[3];
  JLimit SeedWaistJointLimit[3];
  double ReadAngle[SEED_FREEDOM];
  double RightArmSetJointPos[7];
  double LeftArmSetJointPos[7];
  double NeckSetJointPos[3];
  double WaistSetJointPos[3];
  int NeedCancelScriptRightHand;
  int NeedCancelScriptLeftHand;

#ifndef _USE_SERIALCOM_CLASS
  char *dev;
  int fd;
  struct termios newtio;
#endif
  
 //private関数の定義
 //10進数を16進数に変換
  uchar ConvertDecimalNumberToHexadecimalNumber(int DecimalNumber);
  //16進数を10進数に変換
  unsigned int ConvertHexadecimalNumberToDecimalNumber(uchar HexadecimalNumber);
  void serialWrite(uchar *buf, int length);
  void serialRead(uchar *recv, int length);
  uchar calcCheckSum(uchar *buf, int Datasize);
  //void ReadServoAngle(double ReadAngle[]);
  
 public:
 //コンストラクタ
 Seednoid();
 
 // シリアル通信関数
 int OpenSerialPort(const char *SERIAL_PORT);
 void CloseSerialPort();
 //void serialWrite(uchar *buf, int length);
 //void serialRead(uchar *recv, int length);
 //uchar calcCheckSum(uchar *buf, int Datasize);
 void ReadServoAngle(double ReadAngle[]);

 //変数セット関数
 int setRightJointAngle(double JointPos[]);
 int setLeftJointAngle(double JointPos[]);
 int setNeckJointAngle(Frame pos);
 int setWaistJointAngle(Frame pos); 

 int getRightJointAngle(double JointPos[]);
 int getLeftJointAngle(double JointPos[]);
 Frame getNeckJointAngle();
 Frame getWaistJointAngle(); 

 void readRightJointAngle(double JointPos[]);
 void readLeftJointAngle(double JointPos[]);
 Frame readNeckJointAngle();
 Frame readWaistJointAngle();
 
  //Seednoidに関する関数
  void initPosition();
  void ServoOnOff(int torque);
  void SeedAction(int mtime);
  void OpenRightGripper();
  void CloseRightGripper();
  void MoveRightGripper(int angleRatio);
  void OpenLeftGripper();
  void CloseLeftGripper();
  void MoveLeftGripper(int angleRatio);
  void TransRot(double eerot[],double roll,double pitch,double yaw);
  Frame Solve_Rot(double eerot[]);
  int Solve_RightArmIk(double eerot[9],double eetrans[3], double iksol[]);
  int Solve_LeftArmIk(double eerot[9],double eetrans[3], double iksol[]);
  int Solve_RightArmFK(double eerot[9],double eetrans[3],double joint[]);
  int Solve_LeftArmFK(double eerot[9],double eetrans[3],double joint[]);
  void setRightHandCurrent(int CurrentRate);
  void setLeftHandCurrent(int CurrentRate);
  void cancelScriptRightHand();
  void cancelScriptLeftHand();

};
#ifdef _USE_SERIALCOM_CLASS
extern SerialCom sc;
#endif
extern Seednoid noid;

#endif//__LIBSEEDNOIDUPPER_H__
