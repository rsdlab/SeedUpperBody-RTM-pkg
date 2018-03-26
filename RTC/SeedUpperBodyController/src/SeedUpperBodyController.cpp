// -*- C++ -*-
/*!
 * @file  SeedUpperBodyController.cpp
 * @brief SeedUpperBodyController
 * @date $Date$
 *
 * $Id$
 */

#include "SeedUpperBodyController.h"
#include <math.h>

// Module specification
// <rtc-template block="module_spec">
static const char* seedupperbodycontroller_spec[] =
  {
    "implementation_id", "SeedUpperBodyController",
    "type_name",         "SeedUpperBodyController",
    "description",       "SeedUpperBodyController",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SeedUpperBodyController::SeedUpperBodyController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RightManipulatorCommonInterface_CommonPort("RightManipulatorCommonInterface_Common"),
    m_RightManipulatorCommonInterface_MiddlePort("RightManipulatorCommonInterface_Middle"),
    m_LeftManipulatorCommonInterface_CommonPort("LeftManipulatorCommonInterface_Common"),
    m_LeftManipulatorCommonInterface_MiddlePort("LeftManipulatorCommonInterface_Middle"),
    m_SeedWaistInterfacePort("SeedWaistInterface"),
    m_SeedNeckInterfacePort("SeedNeckInterface"),
    m_LifterPosePort("LifterPose"),
    m_WaustRightArmKinematicsPort("WaustRightArmKinematics"),
    m_WaustLeftArmKinematicsPort("WaustLeftArmKinematics")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SeedUpperBodyController::~SeedUpperBodyController()
{
}



RTC::ReturnCode_t SeedUpperBodyController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_RightManipulatorCommonInterface_CommonPort.registerConsumer("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_RightManipulatorCommonInterface_MiddlePort.registerConsumer("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);
  m_LeftManipulatorCommonInterface_CommonPort.registerConsumer("LeftManipulatorCommonInterface_Common", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common", m_LeftManipulatorCommonInterface_Common);
  m_LeftManipulatorCommonInterface_MiddlePort.registerConsumer("LeftManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle", m_LeftManipulatorCommonInterface_Middle);
  m_SeedWaistInterfacePort.registerConsumer("SeedWaistInterface", "SeedWaist::SeedWaistInterface", m_SeedWaistInterface);
  m_SeedNeckInterfacePort.registerConsumer("SeedNeckInterface", "SeedNeck::SeedNeckInterface", m_SeedNeckInterface);
  m_LifterPosePort.registerConsumer("lifterPose", "SeedNoid_Mobile::LifterInterface", m_lifterPose);
  m_WaustRightArmKinematicsPort.registerConsumer("WaistRightArmIkInterface", "RIGHT_IK_FAST::WaistRightArmIkInterface", m_WaistRightArmIkInterface);
  m_WaustLeftArmKinematicsPort.registerConsumer("WaistLeftArmIkInterface", "LEFT_IK_FAST::WaistLeftArmIkInterface", m_WaistLeftArmIkInterface);
  
  // Set CORBA Service Ports
  addPort(m_RightManipulatorCommonInterface_CommonPort);
  addPort(m_RightManipulatorCommonInterface_MiddlePort);
  addPort(m_LeftManipulatorCommonInterface_CommonPort);
  addPort(m_LeftManipulatorCommonInterface_MiddlePort);
  addPort(m_SeedWaistInterfacePort);
  addPort(m_SeedNeckInterfacePort);
  addPort(m_LifterPosePort);
  addPort(m_WaustRightArmKinematicsPort);
  addPort(m_WaustLeftArmKinematicsPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBodyController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SeedUpperBodyController::onActivated(RTC::UniqueId ec_id)
{
  //プロバイダより遅くActivateするため
  sleep(1);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBodyController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBodyController::onExecute(RTC::UniqueId ec_id)
{
 int c;

  std::cout << "コマンドを選択してください" << std::endl;
  std::cout << "1 : サーボON" << std::endl;
  std::cout << "2 : サーボOFF" << std::endl;
  std::cout << "3 : 右腕の制御" << std::endl;
  std::cout << "4 : 左腕の制御" << std::endl;
  std::cout << "5 : 首の制御" << std::endl;
  std::cout << "6 : 腰の制御" << std::endl;
  std::cout << "7 : 現在角度の取得" << std::endl;
  std::cout << "8 : 先端座標の取得" << std::endl;
  std::cout << "9 : 腰＋右腕の制御" << std::endl;
  std::cout << "10 : 腰＋左腕の制御" << std::endl;
  std::cout << "11 : 終了" << std::endl;


  std::cout << ">>";
  std::cin >> c; 
  std::cout << std::endl; 

  //******************   サーボON   ******************************
  if(c == 1)
    {
      m_rid_right=m_ManipulatorCommonInterface_Common->servoON();
      if(m_rid_right->id != 0){//Error
	std::cout<<"Servo ON ERROR"<<std::endl;
	std::cout<<m_rid_right->comment<<std::endl<<std::endl;
      }
    }

  //******************   サーボOFF   ******************************
  else if(c == 2)
    {
      m_rid_right=m_ManipulatorCommonInterface_Common->servoOFF();
      if(m_rid_right->id != 0){//Error
	std::cout<<"Servo OFF ERROR"<<std::endl;
	std::cout<<m_rid_right->comment<<std::endl<<std::endl;
      }
    }

  //******************   右腕の制御   ******************************
  else if(c == 3)//右腕
    {
      int r; 
      std::cout << "1 : アーム先端の移動" << std::endl;
      std::cout << "2 : アーム関節の回転" << std::endl;
      std::cout << "3 : グリッパ閉" << std::endl;
      std::cout << "4 : グリッパ開" << std::endl;
      std::cout << "5 : グリッパ操作" << std::endl;
	    
      std::cout << ">>";
      std::cin >> r; 
      std::cout << std::endl; 

        //******************   右腕アーム先端の移動   ********************
      if(r==1)
	{   
	  int rp;
	  std::cout << "1 : 直交空間における直線軌道" << std::endl;
	  std::cout << "2 : 関節空間における直線軌道" << std::endl;
	  
	  std::cout << ">>";
	  std::cin >> rp; 
	  std::cout << std::endl; 
	  

	      
	  double x;
	  double y;
	  double z;
	  double roll;
	  double pitch;
	  double yaw;
	  double eerot[9];
	  JARA_ARM::CarPosWithElbow pos;
	  
	  std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
	  std::cout<< "yaw : ";
	  std::cin >> yaw;
	  std::cout << "pitch : ";
	  std::cin >> pitch;
	  std::cout << "roll : ";
	  std::cin >> roll;
	  std::cout << std::endl; 
	  
	  std::cout << "座標を入力してください[mm]" << std::endl;
	  std::cout << "x座標 : "; 
	  std::cin >> x;
	  std::cout << "y座標 : ";
	  std::cin >> y;
	  std::cout << "z座標 : ";
	  std::cin >> z;
	  std::cout << std::endl; 
	  
	  //姿勢の行列の計算
	  TransRot(eerot,roll,pitch,yaw);
	  //1列目
	  pos.carPos[0][0]=eerot[0];
	  pos.carPos[0][1]=eerot[1];
	  pos.carPos[0][2]=eerot[2];
	  
	  //2列目
	  pos.carPos[1][0]=eerot[3];
	  pos.carPos[1][1]=eerot[4];
	  pos.carPos[1][2]=eerot[5];
	  
	  //3列目
	  pos.carPos[2][0]=eerot[6];
	  pos.carPos[2][1]=eerot[7];
	  pos.carPos[2][2]=eerot[8];
	  
	  //4列目
	  pos.carPos[0][3]=x;//[mm]
	  pos.carPos[1][3]=y;
	  pos.carPos[2][3]=z;
	  
	  if(rp==1)
	    {
	      m_rid_right=m_ManipulatorCommonInterface_Middle->moveLinearCartesianAbs(pos);
	      if(m_rid_right->id != 0){//Error
		std::cout<<"movePTPCartesianAbs ERROR"<<std::endl;
		std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	      }
	    }
	  if(rp==2)
	    {
	      m_rid_right=m_ManipulatorCommonInterface_Middle->movePTPCartesianAbs(pos);
	      if(m_rid_right->id != 0){//Error
		std::cout<<"movePTPCartesianAbs ERROR"<<std::endl;
		std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	      }
	    }
	  
	}
      
      //******************   右腕アーム関節の回転   ******************************
      else if(r==2)
	{
	  JARA_ARM::ManipInfo_var minfovar;
	  JARA_ARM::ManipInfo minfo;
	  JARA_ARM::JointPos jointPoint;
	  m_ManipulatorCommonInterface_Common->getManipInfo(minfovar);
	  minfo=minfovar;
	  jointPoint.length(minfo.axisNum);
	  std::cout << "下からグリッパに向かって第1関節〜第"<<minfo.axisNum<<"関節" << std::endl << "関節角度を入力してください　単位[°]" << std::endl;
	  for(int i=0;i<(int)minfo.axisNum;i++)
	    {
	      std::cout << "第"<<i+1<<"関節の角度 : ";
	      std::cin >>jointPoint[i];
	      if(i==(int)minfo.axisNum-1){//最後の関節で1行改行
		std::cout << std::endl; 
	      }
	    }
	  
	  m_rid_right=m_ManipulatorCommonInterface_Middle->movePTPJointAbs(jointPoint);
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"movePTPJointAbs ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	}

      //******************   右腕アームグリッパ閉   ******************************
      else if(r==3)
	{
	  m_rid_right=m_ManipulatorCommonInterface_Middle->closeGripper();
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"closeGripper ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	}

      //******************   右腕アームグリッパ開   ******************************
      else if(r==4)
	{
	  m_rid_right=m_ManipulatorCommonInterface_Middle->openGripper();
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"closeGripper ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	}
      //******************   右腕アーム操作   ******************************
      else if(r==5)
	{
	  JARA_ARM::ULONG angleRatio;
	  std::cout << "開度を入力して下さい" << std::endl;
	  std::cout << ">>";
	  std::cin >> angleRatio;
	  m_rid_right=m_ManipulatorCommonInterface_Middle->moveGripper(angleRatio);
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"moveGripper ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	}
      else 
	std::cout << "無効な値です" << std::endl<<std::endl;
      
    }//else if(c==3)
  
  //******************   左腕の制御   ******************************
  else if(c == 4)
    {
      int l; 
      std::cout << "1 : アーム先端の移動" << std::endl;
      std::cout << "2 : アーム関節の回転" << std::endl;
      std::cout << "3 : グリッパ閉" << std::endl;
      std::cout << "4 : グリッパ開" << std::endl;
      std::cout << "5 : グリッパ操作" << std::endl;

      std::cout << ">>";
      std::cin >> l; 
      std::cout << std::endl; 

      //******************   左腕アーム先端の移動   ******************************
      if(l==1)
	{   
	  int lp;
	  std::cout << "1 : 直交空間における直線軌道" << std::endl;
	  std::cout << "2 : 関節空間における直線軌道" << std::endl;
	  
	  std::cout << ">>";
	  std::cin >> lp; 
	  std::cout << std::endl; 
	  
	  double x;
	  double y;
	  double z;
	  double roll;
	  double pitch;
	  double yaw;
	  double eerot[9];
	  JARA_ARM_LEFT::CarPosWithElbow pos;
	  
	  std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
	  std::cout<< "yaw : ";
	  std::cin >> yaw;
	  std::cout << "pitch : ";
	  std::cin >> pitch;
	  std::cout << "roll : ";
	  std::cin >> roll;
	  std::cout << std::endl; 
	  
	  std::cout << "座標を入力してください[mm]" << std::endl;
	  std::cout << "x座標 : "; 
	  std::cin >> x;
	  std::cout << "y座標 : ";
	  std::cin >> y;
	  std::cout << "z座標 : ";
	  std::cin >> z;
	  std::cout << std::endl; 
	  
	  //姿勢の行列の計算
	  TransRot(eerot,roll,pitch,yaw);
	  //1列目
	  pos.carPos[0][0]=eerot[0];
	  pos.carPos[0][1]=eerot[1];
	  pos.carPos[0][2]=eerot[2];
	  
	  //2列目
	  pos.carPos[1][0]=eerot[3];
	  pos.carPos[1][1]=eerot[4];
	  pos.carPos[1][2]=eerot[5];
	  
	  //3列目
	  pos.carPos[2][0]=eerot[6];
	  pos.carPos[2][1]=eerot[7];
	  pos.carPos[2][2]=eerot[8];
	  
	  //4列目
	  pos.carPos[0][3]=x;//[mm]
	  pos.carPos[1][3]=y;
	  pos.carPos[2][3]=z;
	  
	  if(lp==1)
	    {
	      m_rid_left=m_LeftManipulatorCommonInterface_Middle->moveLinearCartesianAbs(pos);
	      if(m_rid_left->id != 0){//Error
		std::cout<<"movePTPCartesianAbs ERROR"<<std::endl;
		std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	      }
	    }
	  if(lp==2)
	    {
	      m_rid_left=m_LeftManipulatorCommonInterface_Middle->movePTPCartesianAbs(pos);
	      if(m_rid_left->id != 0){//Error
		std::cout<<"movePTPCartesianAbs ERROR"<<std::endl;
		std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	      }
	    }
	  
	}
      
      //******************   左腕アーム関節の移動   ******************************
      else if(l==2)
	{
	  JARA_ARM_LEFT::ManipInfo_var minfovar;
	  JARA_ARM_LEFT::ManipInfo minfo;
	  JARA_ARM_LEFT::JointPos jointPoint;
	  m_LeftManipulatorCommonInterface_Common->getManipInfo(minfovar);
	  minfo=minfovar;
	  jointPoint.length(minfo.axisNum);
	  std::cout << "下からグリッパに向かって第1関節〜第"<<minfo.axisNum<<"関節" << std::endl << "関節角度を入力してください　単位[°]" << std::endl;
	  for(int i=0;i<(int)minfo.axisNum;i++)
	    {
	      std::cout << "第"<<i+1<<"関節の角度 : ";
	      std::cin >>jointPoint[i];
	      if(i==(int)minfo.axisNum-1){//最後の関節で1行改行
		std::cout << std::endl; 
	      }
	    }
	  
	  m_rid_left=m_LeftManipulatorCommonInterface_Middle->movePTPJointAbs(jointPoint);
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"movePTPJointAbs ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	}
      //******************   左腕グリッパ閉   ******************************
      else if(l==3)
	{
	  m_rid_left=m_LeftManipulatorCommonInterface_Middle->closeGripper();
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"closeGripper ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	}
      //******************   左腕グリッパ開   ******************************
      else if(l==4)
	{
	  m_rid_left=m_LeftManipulatorCommonInterface_Middle->openGripper();
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"closeGripper ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	}
     //******************   左腕グリッパ操作   ******************************
      else if(l==5)
	{
	  JARA_ARM_LEFT::ULONG angleRatio;
	  std::cout << "開度を入力して下さい" << std::endl;
	  std::cout << ">>";
	  std::cin >> angleRatio;
	  m_rid_left=m_LeftManipulatorCommonInterface_Middle->moveGripper(angleRatio);
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"moveGripper ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	}
      else 
	std::cout << "無効な値です" << std::endl<<std::endl;
    }

  //******************   首の制御   ******************************
  else if(c == 5)//首の制御
    {
	  SeedNeck::NeckPos neckPoint;
	  neckPoint.length(3);
	  std::cout << "姿勢を変更する角度を入力して下さい" << std::endl;
	  std::cout << "頭ヨー(回転)-90~90 : ";
	  std::cin >> neckPoint[0];
	  std::cout << "頭ピッチ(前後)-20~+60 : ";
	  std::cin >> neckPoint[1];
	  std::cout << "頭ロール(左右)-20~+20 : ";
	  std::cin >> neckPoint[2];
	  
	  m_rid_neck=m_SeedNeckInterface->movePTPNeckAbs(neckPoint);
	  if(m_rid_neck->id != 0){//Error
	    std::cout<<"movePTPNeckAbs ERROR"<<std::endl;
	    std::cout<<m_rid_neck->comment<<std::endl<<std::endl;
	  }
    }  
  
  //******************   腰の制御   ******************************
  else if(c == 6)
    {
	  SeedWaist::WaistPos waistPoint;
	  waistPoint.length(3);
	  std::cout << "姿勢を変更する角度を入力して下さい" << std::endl;
	  std::cout << "腰ヨー(回転)-90~90 : ";
	  std::cin >> waistPoint[0];
	  std::cout << "腰ピッチ(前後)-9~+39 : ";
	  std::cin >> waistPoint[1];
	  std::cout << "腰ロール(左右)-9~+9 : ";
	  std::cin >> waistPoint[2];
	  
	  m_rid_waist=m_SeedWaistInterface->movePTPWaistAbs(waistPoint);
	  if(m_rid_waist->id != 0){//Error
	    std::cout<<"movePTPWaistAbs ERROR"<<std::endl;
	    std::cout<<m_rid_waist->comment<<std::endl<<std::endl;
	  }
    }

  //******************   現在角度の取得   ******************************
  else if(c == 7)
    {
      JARA_ARM::JointPos_var Rightposvar;
      JARA_ARM::JointPos Rightpos;
      JARA_ARM_LEFT::JointPos_var Leftposvar;
      JARA_ARM_LEFT::JointPos Leftpos;
      SeedNeck::NeckPos_var Neckposvar;
      SeedNeck::NeckPos Neckpos;
      SeedWaist::WaistPos_var Waistposvar;
      SeedWaist::WaistPos Waistpos;
      Rightpos.length(7);
      Leftpos.length(7);
      Neckpos.length(3);
      Waistpos.length(3);
      //右腕
      m_rid_right=m_ManipulatorCommonInterface_Common->getFeedbackPosJoint(Rightposvar);
      if(m_rid_right->id != 0){//Error
	std::cout<<"RightgetFeedbackPosJoint ERROR"<<std::endl;
	std::cout<<m_rid_right->comment<<std::endl<<std::endl;
      }
      Rightpos = Rightposvar;
      for(int i=0;i<7;i++){
	std::cout <<"RightArmpos["<<i<<"] = "<<Rightpos[i]<<std::endl;
      }
      //左腕
      m_rid_left=m_LeftManipulatorCommonInterface_Common->getFeedbackPosJoint(Leftposvar);
      if(m_rid_left->id != 0){//Error
	std::cout<<"LeftgetFeedbackPosJoint ERROR"<<std::endl;
	std::cout<<m_rid_left->comment<<std::endl<<std::endl;
      }
      Leftpos = Leftposvar;
      for(int j=0;j<7;j++){
	std::cout <<"LeftArmpos["<<j<<"] = "<<Leftpos[j]<<std::endl;
      }
      //首
      m_rid_neck=m_SeedNeckInterface->getFeedbackPosNeck(Neckposvar);
      if(m_rid_neck->id != 0){//Error
	std::cout<<"NeckgetFeedbackPosNeck ERROR"<<std::endl;
	std::cout<<m_rid_neck->comment<<std::endl<<std::endl;
	  }
      Neckpos = Neckposvar;
      for(int k=0;k<3;k++){
	std::cout <<"Neckpos["<<k<<"] = "<<Neckpos[k]<<std::endl;
      }
      //腰
      m_rid_waist=m_SeedWaistInterface->getFeedbackPosWaist(Waistposvar);
      if(m_rid_waist->id != 0){//Error
	std::cout<<"getFeedbackPosNeck ERROR"<<std::endl;
	std::cout<<m_rid_waist->comment<<std::endl<<std::endl;
      }
      Waistpos = Waistposvar;
      for(int h=0;h<3;h++){
	std::cout <<"Waistpos["<<h<<"] = "<<Waistpos[h]<<std::endl;
      }
    }
  
  //******************   先端座標の取得   ******************************
  else if(c == 8)
    {
      int a;
      std::cout << "1 : 肩を原点とした先端位置" << std::endl;
      std::cout << "2 : 腰を原点とした先端位置" << std::endl;
      
      std::cout << ">>";
      std::cin >> a; 
      std::cout << std::endl; 
      double eerot[9];

      if(a==1)
	{
	  JARA_ARM::CarPosWithElbow rightpos;
	  m_rid_right=m_ManipulatorCommonInterface_Middle->getFeedbackPosCartesian(rightpos);
	  if(m_rid_right->id != 0){//Error
	    std::cout<< "rightgetFeedbackPosCartesian ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	  std::cout << "rightpos.carPos[0][0] = "<<rightpos.carPos[0][0]<<std::endl;
	  std::cout << "rightpos.carPos[0][1] = "<<rightpos.carPos[0][1]<<std::endl;
	  std::cout << "rightpos.carPos[0][2] = "<<rightpos.carPos[0][2]<<std::endl;
	  std::cout << "rightpos.carPos[1][0] = "<<rightpos.carPos[1][0]<<std::endl;
	  std::cout << "rightpos.carPos[1][1] = "<<rightpos.carPos[1][1]<<std::endl;
	  std::cout << "rightpos.carPos[1][2] = "<<rightpos.carPos[1][2]<<std::endl;
	  std::cout << "rightpos.carPos[2][0] = "<<rightpos.carPos[2][0]<<std::endl;
	  std::cout << "rightpos.carPos[2][1] = "<<rightpos.carPos[2][1]<<std::endl;
	  std::cout << "rightpos.carPos[2][2] = "<<rightpos.carPos[2][2]<<std::endl;
	  std::cout << "rightpos.carPos[0][3] = "<<rightpos.carPos[0][3]<<std::endl;
	  std::cout << "rightpos.carPos[1][3] = "<<rightpos.carPos[1][3]<<std::endl;
	  std::cout << "rightpos.carPos[2][3] = "<<rightpos.carPos[2][3]<<std::endl;
	  
	  JARA_ARM_LEFT::CarPosWithElbow leftpos;
	  m_rid_left=m_LeftManipulatorCommonInterface_Middle->getFeedbackPosCartesian(leftpos);
	  if(m_rid_left->id != 0){//Error
	    std::cout<< "leftgetFeedbackPosCartesian ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	  std::cout << "leftpos.carPos[0][0] = "<<leftpos.carPos[0][0]<<std::endl;
	  std::cout << "leftpos.carPos[0][1] = "<<leftpos.carPos[0][1]<<std::endl;
	  std::cout << "leftpos.carPos[0][2] = "<<leftpos.carPos[0][2]<<std::endl;
	  std::cout << "leftpos.carPos[1][0] = "<<leftpos.carPos[1][0]<<std::endl;
	  std::cout << "leftpos.carPos[1][1] = "<<leftpos.carPos[1][1]<<std::endl;
	  std::cout << "leftpos.carPos[1][2] = "<<leftpos.carPos[1][2]<<std::endl;
	  std::cout << "leftpos.carPos[2][0] = "<<leftpos.carPos[2][0]<<std::endl;
	  std::cout << "leftpos.carPos[2][1] = "<<leftpos.carPos[2][1]<<std::endl;
	  std::cout << "leftpos.carPos[2][2] = "<<leftpos.carPos[2][2]<<std::endl;
	  std::cout << "leftpos.carPos[0][3] = "<<leftpos.carPos[0][3]<<std::endl;
	  std::cout << "leftpos.carPos[1][3] = "<<leftpos.carPos[1][3]<<std::endl;
	  std::cout << "leftpos.carPos[2][3] = "<<leftpos.carPos[2][3]<<std::endl;
	} 
      else if(a==2)
	{
	  //現在角度の読み取り
	  JARA_ARM::JointPos_var Rightposvar;
	  JARA_ARM::JointPos Rightpos;
	  JARA_ARM_LEFT::JointPos_var Leftposvar;
	  JARA_ARM_LEFT::JointPos Leftpos;
	  SeedWaist::WaistPos_var Waistposvar;
	  SeedWaist::WaistPos Waistpos;
	  Rightpos.length(7);
	  Leftpos.length(7);
	  Waistpos.length(3);
	  //右腕
	  std::cout << "右腕角度読み取り" << std::endl;
	  m_rid_right=m_ManipulatorCommonInterface_Common->getFeedbackPosJoint(Rightposvar);
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"RightgetFeedbackPosJoint ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	  Rightpos = Rightposvar;
	  //左腕
	  std::cout << "左腕角度読み取り" << std::endl;
	  m_rid_left=m_LeftManipulatorCommonInterface_Common->getFeedbackPosJoint(Leftposvar);
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"LeftgetFeedbackPosJoint ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
	  Leftpos = Leftposvar;
	  //腰
	  std::cout << "腰角度読み取り" << std::endl;
	  m_rid_waist=m_SeedWaistInterface->getFeedbackPosWaist(Waistposvar);
	  if(m_rid_waist->id != 0){//Error
	    std::cout<<"getFeedbackPosNeck ERROR"<<std::endl;
	    std::cout<<m_rid_waist->comment<<std::endl<<std::endl;
	  }
	  Waistpos = Waistposvar;
	  //RightFK====================================
	  RIGHT_IK_FAST::Rotation_var righteerotvar;
	  RIGHT_IK_FAST::FramePos_var righteetransvar;
	  RIGHT_IK_FAST::Rotation rightgeteerot;
	  RIGHT_IK_FAST::FramePos rightgeteetrans;
	  RIGHT_IK_FAST::JointPos rightNowJointPos;
	  Frame rightRotation;

	  rightgeteerot.length(9);
	  rightgeteetrans.length(3);
	  rightNowJointPos.length(10);
	 
	  
	  for(int i=0;i<3;i++){
	    rightNowJointPos[i] = Waistpos[i];
	  }
	 
	  for(int j=0;j<7;j++){
	    rightNowJointPos[j+3] = Rightpos[j];
	      }
	  m_rid_RIK = m_WaistRightArmIkInterface->solveRightArmFk(rightNowJointPos,righteerotvar, righteetransvar);
	  if(m_rid_RIK->id != 0){//Error
	    std::cout<<"solveRightArmFk ERROR"<<std::endl;
	    std::cout<<m_rid_RIK->comment<<std::endl<<std::endl;
	  }
	  rightgeteerot = righteerotvar;
	  rightgeteetrans = righteetransvar;
	  for(int n=0;n<9;n++){
	    std::cout << "rightgeteerot["<<n<<"] = " << rightgeteerot[n] <<std::endl;
	  }
	  for(int m=0;m<3;m++){
	    std::cout << "rightgeteetrans["<<m<<"] = " << rightgeteetrans[m] <<std::endl;
	  }
	  for(int r=0;r<9;r++){
	    eerot[r] = rightgeteerot[r];
	  }
	  rightRotation = Solve_Rot(eerot);
	  std::cout << "rightsol_yaw = " << rightRotation.yaw <<std::endl;
	  std::cout << "rightsol_pitch = " << rightRotation.pitch <<std::endl;
	  std::cout << "rightsol_roll = " << rightRotation.roll <<std::endl;
	  
	  //LeftFK========================================
	  LEFT_IK_FAST::Rotation_var lefteerotvar;
	  LEFT_IK_FAST::FramePos_var lefteetransvar;
	  LEFT_IK_FAST::Rotation leftgeteerot;
	  LEFT_IK_FAST::FramePos leftgeteetrans;
	  LEFT_IK_FAST::JointPos leftNowJointPos;
	  Frame leftRotation;
	  leftgeteerot.length(9);
	  leftgeteetrans.length(3);
	  leftNowJointPos.length(10);
	  
	  for(int q=0;q<3;q++){
	    leftNowJointPos[q] = Waistpos[q];  
	      }	
	  for(int e=0;e<7;e++){
	    leftNowJointPos[e+3] = Rightpos[e];
	      }
	  m_rid_LIK = m_WaistLeftArmIkInterface->solveLeftArmFk(leftNowJointPos,lefteerotvar, lefteetransvar);
	  if(m_rid_LIK->id != 0){//Error
	    std::cout<<"solveLeftArmFk ERROR"<<std::endl;
	    std::cout<<m_rid_LIK->comment<<std::endl<<std::endl;
	  }  
	  leftgeteerot = lefteerotvar;
	  leftgeteetrans = lefteetransvar;
	  for(int k=0;k<9;k++){
	    std::cout << "leftgeteerot["<<k<<"] = " << leftgeteerot[k] <<std::endl;
	  }
	  for(int j=0;j<3;j++){
	    std::cout << "leftgeteetrans["<<j<<"] = " << leftgeteetrans[j] <<std::endl;
	  }
	  for(int t=0;t<9;t++){
	    eerot[t] = leftgeteerot[t];
	  }
	  leftRotation = Solve_Rot(eerot);
	  std::cout << "leftsol_yaw = " << leftRotation.yaw <<std::endl;
	  std::cout << "leftsol_pitch = " << leftRotation.pitch <<std::endl;
	  std::cout << "leftsol_roll = " << leftRotation.roll <<std::endl;
	}
      else
	{
	  std::cout << "入力が違います" << std::endl;
	}
    }
      
      //******************   腰＋右腕の制御   ******************************
  else if(c == 9)
    {
	  int w;
	  std::cout << "1 : 腰の姿勢を指定する" << std::endl;
	  std::cout << "2 : 腰の姿勢を指定しない" << std::endl;
	  
	  std::cout << ">>";
	  std::cin >> w; 
	  std::cout << std::endl; 
	  double x;
	  double y;
	  double z;
	  double roll;
	  double pitch;
	  double yaw;
	  double calceerot[9];
	  RIGHT_IK_FAST::Rotation eerot;
	  RIGHT_IK_FAST::FramePos eetrans;
	  RIGHT_IK_FAST::WaistFrame WaistPos;
	  RIGHT_IK_FAST::JointPos nowJointPos;
	  RIGHT_IK_FAST::JointPos_var solRightJointPosvar;
	  RIGHT_IK_FAST::JointPos solRightJointPos;
	  eerot.length(9);
	  eetrans.length(3);
	  nowJointPos.length(10);
	  solRightJointPos.length(10);
	  JARA_ARM::JointPos jointPoint;
	  jointPoint.length(7);
	  SeedWaist::WaistPos waistPoint;
	  waistPoint.length(3);
	  
	  std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
	  std::cout<< "yaw : ";
	  std::cin >> yaw;
	  std::cout << "pitch : ";
	  std::cin >> pitch;
	  std::cout << "roll : ";
	  std::cin >> roll;
	  std::cout << std::endl; 
	  
	  std::cout << "座標を入力してください[mm]" << std::endl;
	  std::cout << "x座標 : "; 
	  std::cin >> x;
	  std::cout << "y座標 : ";
	  std::cin >> y;
	  std::cout << "z座標 : ";
	  std::cin >> z;
	  std::cout << std::endl; 

	  if(w==1)
	    {
	      std::cout << "姿勢を変更する角度を入力して下さい" << std::endl;
	      std::cout << "腰ロール(左右)-9~+9 : ";
	      std::cin >> WaistPos.roll;
	      std::cout << "腰ピッチ(前後)-9~+39 : ";
	      std::cin >> WaistPos.pitch;
	      std::cout << "腰ヨー(回転)-90~90 : ";
	      std::cin >> WaistPos.yaw;
	    }

	  TransRot(calceerot,roll,pitch,yaw);
	  eetrans[0] = x/1000;//x
	  eetrans[1] = y/1000;//y
	  eetrans[2] = z/1000;//z
	  
	  for(int i=0;i<10;i++){
	    nowJointPos[i] = 0;
	    if(i!=9){
	      eerot[i] = calceerot[i];
	    }
	  }
	  if(w==1)
	    {
	      m_rid_RIK = m_WaistRightArmIkInterface->solveRightArmIkSetWaistJoint(eerot, eetrans, WaistPos, nowJointPos, solRightJointPosvar);
	      if(m_rid_RIK->id != 0){//Error
		std::cout<<"solveRightIkSetWaistJoint ERROR"<<std::endl;
		std::cout<<m_rid_RIK->comment<<std::endl<<std::endl;
		return RTC::RTC_OK;
	      }
	      solRightJointPos = solRightJointPosvar;
	    }
	  if(w==2)
	    {
	      m_rid_RIK = m_WaistRightArmIkInterface->solveRightArmIk(eerot, eetrans, nowJointPos, solRightJointPosvar);
	      if(m_rid_RIK->id != 0){//Error
		std::cout<<"solveRightIk ERROR"<<std::endl;
		std::cout<<m_rid_RIK->comment<<std::endl<<std::endl;
		return RTC::RTC_OK;
	      }
	      solRightJointPos = solRightJointPosvar;
	    }
	  
	  waistPoint[0] = solRightJointPos[0];
	  waistPoint[1] = solRightJointPos[1];
	  waistPoint[2] = solRightJointPos[2];
	  for(int k=0;k<7;k++)
	    {
	      jointPoint[k] = solRightJointPos[k+3];
	    }
	  m_rid_waist=m_SeedWaistInterface->movePTPWaistAbs(waistPoint);
	  if(m_rid_waist->id != 0){//Error
	    std::cout<<"movePTPWaistAbs ERROR"<<std::endl;
	    std::cout<<m_rid_waist->comment<<std::endl<<std::endl;
	  }
	  m_rid_right=m_ManipulatorCommonInterface_Middle->movePTPJointAbs(jointPoint);
	  if(m_rid_right->id != 0){//Error
	    std::cout<<"movePTPJointAbs ERROR"<<std::endl;
	    std::cout<<m_rid_right->comment<<std::endl<<std::endl;
	  }
	 
    }

  //******************   腰＋左腕の制御   ******************************
  else if(c == 10)
    {
	  int w;
	  std::cout << "1 : 腰の姿勢を指定する" << std::endl;
	  std::cout << "2 : 腰の姿勢を指定しない" << std::endl;
	  
	  std::cout << ">>";
	  std::cin >> w; 
	  std::cout << std::endl; 
	  double x;
	  double y;
	  double z;
	  double roll;
	  double pitch;
	  double yaw;
	  double calceerot[9];
	  LEFT_IK_FAST::Rotation eerot;
	  LEFT_IK_FAST::FramePos eetrans;
	  LEFT_IK_FAST::WaistFrame WaistPos;
	  LEFT_IK_FAST::JointPos nowJointPos;
	  LEFT_IK_FAST::JointPos_var solLeftJointPosvar;
	  LEFT_IK_FAST::JointPos solLeftJointPos;
	  eerot.length(9);
	  eetrans.length(3);
	  nowJointPos.length(10);
	  solLeftJointPos.length(10);
	  JARA_ARM_LEFT::JointPos jointPoint;
	  jointPoint.length(7);
	  SeedWaist::WaistPos waistPoint;
	  waistPoint.length(3);
	  
	  std::cout << "姿勢を入力して下さい(yaw,pitch,roll)[deg]"<<std::endl;
	  std::cout<< "yaw : ";
	  std::cin >> yaw;
	  std::cout << "pitch : ";
	  std::cin >> pitch;
	  std::cout << "roll : ";
	  std::cin >> roll;
	  std::cout << std::endl; 
	  
	  std::cout << "座標を入力してください[mm]" << std::endl;
	  std::cout << "x座標 : "; 
	  std::cin >> x;
	  std::cout << "y座標 : ";
	  std::cin >> y;
	  std::cout << "z座標 : ";
	  std::cin >> z;
	  std::cout << std::endl; 

	  if(w==1)
	    {
	      std::cout << "姿勢を変更する角度を入力して下さい" << std::endl;
	      std::cout << "腰ロール(左右)-9~+9 : ";
	      std::cin >> WaistPos.roll;
	      std::cout << "腰ピッチ(前後)-9~+39 : ";
	      std::cin >> WaistPos.pitch;
	      std::cout << "腰ヨー(回転)-90~90 : ";
	      std::cin >> WaistPos.yaw;
	    }

	  TransRot(calceerot,roll,pitch,yaw);
	  eetrans[0] = x/1000;//x
	  eetrans[1] = y/1000;//y
	  eetrans[2] = z/1000;//z
	  
	  for(int i=0;i<10;i++){
	    nowJointPos[i] = 0;
	    if(i!=9){
	      eerot[i] = calceerot[i];
	    }
	  }
	  if(w==1)
	    {
	      m_rid_LIK = m_WaistLeftArmIkInterface->solveLeftArmIkSetWaistJoint(eerot, eetrans, WaistPos, nowJointPos, solLeftJointPosvar);
	      if(m_rid_LIK->id != 0){//Error
		std::cout<<"solveLeftIkSetWaistJoint ERROR"<<std::endl;
		std::cout<<m_rid_LIK->comment<<std::endl<<std::endl;
		return RTC::RTC_OK;
	      }
	      solLeftJointPos = solLeftJointPosvar;
	    }
	  if(w==2)
	    {
	      m_rid_LIK = m_WaistLeftArmIkInterface->solveLeftArmIk(eerot, eetrans, nowJointPos, solLeftJointPosvar);
	      if(m_rid_RIK->id != 0){//Error
		std::cout<<"solveLeftIk ERROR"<<std::endl;
		std::cout<<m_rid_LIK->comment<<std::endl<<std::endl;
		return RTC::RTC_OK;
	      }
	      solLeftJointPos = solLeftJointPosvar;
	    }
	  
	  waistPoint[0] = solLeftJointPos[0];
	  waistPoint[1] = solLeftJointPos[1];
	  waistPoint[2] = solLeftJointPos[2];
	  for(int k=0;k<7;k++)
	    {
	      jointPoint[k] = solLeftJointPos[k+3];
	    }
	  m_rid_waist=m_SeedWaistInterface->movePTPWaistAbs(waistPoint);
	  if(m_rid_waist->id != 0){//Error
	    std::cout<<"movePTPWaistAbs ERROR"<<std::endl;
	    std::cout<<m_rid_waist->comment<<std::endl<<std::endl;
	  }
	 m_rid_left=m_LeftManipulatorCommonInterface_Middle->movePTPJointAbs(jointPoint);
	  if(m_rid_left->id != 0){//Error
	    std::cout<<"movePTPJointAbs ERROR"<<std::endl;
	    std::cout<<m_rid_left->comment<<std::endl<<std::endl;
	  }
    }
  
  //******************   終了   ******************************
  else if(c == 11)
    {
      std::cout << "END" << std::endl;
      std::cout << "システムをDeactivateしてください" << std::endl<<std::endl;
      deactivate(ec_id);
    }
  
  else 
    std::cout << "無効な値です" << std::endl<<std::endl;
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBodyController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/************************************************

	void TransRot(double eerot[],double roll,double pitch,double yaw)
	
	概要：ロールピッチヨー[deg]から先端姿勢の回転行列を求める

	引数：
　　　　　　　double eerot[]・・・計算した行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|
             double roll・・・ロール回転角度
             double pitch・・・ピッチ回転角度
             double yaw・・・ヨー回転角度

	戻り値：なし

*************************************************/
void SeedUpperBodyController::TransRot(double eerot[],double roll,double pitch,double yaw)
{
  //ロールピッチヨー　ロボット工学（遠山茂樹著）p25
  double Rrotx;
  double Rroty;
  double Rrotz;
  Rrotx = yaw * M_PI / 180;
  Rroty = pitch * M_PI / 180;
  Rrotz = roll * M_PI / 180;
  //std::cout << "Rrtx = " << Rrotx << std::endl;
  //std::cout << "Rrty = " << Rroty << std::endl;
  //std::cout << "Rrtz = " << Rrotz << std::endl;
  eerot[0] = cos(Rroty)*cos(Rrotz);
  eerot[1] = cos(Rrotz)*sin(Rroty)*sin(Rrotx)-sin(Rrotz)*cos(Rrotx);
  eerot[2] = cos(Rrotz)*sin(Rroty)*cos(Rrotx)+sin(Rrotz)*sin(Rrotx);
  eerot[3] = sin(Rrotz)*cos(Rroty);
  eerot[4] = sin(Rrotz)*sin(Rroty)*sin(Rrotx)+cos(Rrotz)*cos(Rrotx);
  eerot[5] = sin(Rrotz)*sin(Rroty)*cos(Rrotx)-cos(Rrotz)*sin(Rrotx);
  eerot[6] = -sin(Rroty);
  eerot[7] = cos(Rroty)*sin(Rrotx);
  eerot[8] = cos(Rroty)*cos(Rrotx);
  //for(int j=0;j<9;j++){
  //    printf("eerot[%d] = %f\n",j,eerot[j]);
  //}

}

/************************************************

        Frame Solve_Rot(double eerot[]);
	
	概要：先端姿勢の回転行列から、ロールピッチヨー[deg]を求める

	引数：
　　　　　　　double eerot[]・・・姿勢行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|

	戻り値：姿勢（ロールピッチヨー）

*************************************************/
Frame SeedUpperBodyController::Solve_Rot(double eerot[])
{
  Frame rotation;
  double phi;//φ
  double theta;//θ
  double psi;//ψ

  for(int i=0;i<9;i++)
    {
      if(eerot[i]>1)
	eerot[i] = 1;
      if(eerot[i]<-1)
	eerot[i] = -1;
    }

  phi = atan2(eerot[3], eerot[0]);
  theta =  atan2(-eerot[6], sqrt(eerot[7]*eerot[7]+eerot[8]*eerot[8]));
  psi = atan2(eerot[7], eerot[8]);  

  //phi = atan2(eerot[3], eerot[0]);
  //theta =  asin(-eerot[6]);
  //psi = atan2(eerot[7], eerot[8]);  

  rotation.roll = phi*180/M_PI;
  rotation.pitch = theta*180/M_PI;
  rotation.yaw = psi*180/M_PI;

  return rotation;
}

extern "C"
{
 
  void SeedUpperBodyControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(seedupperbodycontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<SeedUpperBodyController>,
                             RTC::Delete<SeedUpperBodyController>);
  }
  
};


