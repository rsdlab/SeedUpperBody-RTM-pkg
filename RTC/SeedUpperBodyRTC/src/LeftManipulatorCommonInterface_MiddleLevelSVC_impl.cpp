// -*-C++-*-
/*!
 * @file  LeftManipulatorCommonInterface_MiddleLevelSVC_impl.cpp
 * @brief Service implementation code of LeftManipulatorCommonInterface_MiddleLevel.idl
 *
 */

#include "LeftManipulatorCommonInterface_MiddleLevelSVC_impl.h"
#include "LibSeednoidUpper.h"
#include "left_ReturnID.h" 

int LeftCartesianSpeed = 70;
int LeftJointSpeed = 70;

/*
 * Example implementational code for IDL interface JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle
 */
JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra constructor code here.
}


JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::~JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::closeGripper()
{
  std::cout << "closeLeftGripper" << std::endl;
  noid.setLeftHandCurrent(100);
  usleep(1000*10);
  noid.CloseLeftGripper();
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getBaseOffset(JARA_ARM_LEFT::HgMatrix offset)
{
  std::cout<<"getBaseOffset"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getFeedbackPosCartesian(JARA_ARM_LEFT::CarPosWithElbow& pos)
{
  std::cout<<"getFeedbackPosCartesian"<<std::endl;
  
  double JointPos[7];
  noid.readLeftJointAngle(JointPos);
  
  double eerot[9];
  double eetrans[3];
  noid.Solve_LeftArmFK(eerot,eetrans,JointPos);

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
  pos.carPos[0][3]=eetrans[0]*1000;//[mm]
  pos.carPos[1][3]=eetrans[1]*1000;
  pos.carPos[2][3]=eetrans[2]*1000;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedCartesian(JARA_ARM_LEFT::CartesianSpeed& speed)
{
  std::cout<<"getMaxSpeedCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMaxSpeedJoint(JARA_ARM_LEFT::DoubleSeq_out speed)
{
  std::cout<<"getMaxSpeedJoint"<<std::endl;
  speed = new JARA_ARM_LEFT::DoubleSeq;
  speed->length(1);
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeCartesian(::CORBA::Double& aclTime)
{
  std::cout<<"getMinAccelTimeCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getMinAccelTimeJoint(::CORBA::Double& aclTime)
{
  std::cout<<"getMinAccelTimeJoint"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getSoftLimitCartesian(JARA_ARM_LEFT::LimitValue& xLimit, JARA_ARM_LEFT::LimitValue& yLimit, JARA_ARM_LEFT::LimitValue& zLimit)
{
  std::cout<<"getSoftLimitCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveGripper(JARA_ARM_LEFT::ULONG angleRatio)
{
  std::cout << "moveGripper" << std::endl;
  double move;
  
  move = (double)angleRatio;
  
  if (angleRatio>0 && angleRatio <= 100){
    noid.MoveLeftGripper(move);
  }
  else{
    std::cout << "ERROR : angleRatio Wrong Value" << std::endl;
    RETURNID_VALUE_ERR
  }
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
  std::cout<<"moveLinearCartesianAbs"<<std::endl;

  //CartesianSpeed判定
  if((LeftCartesianSpeed <= 0)||(LeftCartesianSpeed > 100)){
    std::cout<<"ERROR : Wrong Cartesian Speed"<<std::endl<<std::endl;
    RETURNID_NG;
  }
  double JointPos[7];
  double startPos[3];
  double viaRot[9];
  double viaPos[3];
  //double viaroll;
  //double viapitch;
  //double viayaw;
  //CartesianSpeedが70%で3秒 100%で30[ms]
  int StrideNum = (int)(-3.3535*LeftCartesianSpeed+336.35);//50*100/(CartesianSpeed);
  double eerot[9];//姿勢
  double start_eerot[9];
  //double via_eerot[9];
  double eetrans[3];//座標
  double iksol[7];
  //Frame start_Rotation;
  //Frame end_Rotation;
  Frame via_Rotation;

  //struct timeval sumstart;
  //struct timeval sumend;
  struct timeval start;
  struct timeval end;
  double sec;
  double micro;
  double duration;
  int first=0;
  double usleeptime;

  //ゴール点が可動範囲か判定
  eerot[0]=carPoint.carPos[0][0];
  eerot[1]=carPoint.carPos[0][1];
  eerot[2]=carPoint.carPos[0][2];
  
  eerot[3]=carPoint.carPos[1][0];
  eerot[4]=carPoint.carPos[1][1];
  eerot[5]=carPoint.carPos[1][2];
  
  eerot[6]=carPoint.carPos[2][0];
  eerot[7]=carPoint.carPos[2][1];
  eerot[8]=carPoint.carPos[2][2];
  
  //座標の入力 関数への入力は[mm]だがikfastは[m]のため変換
  eetrans[0]=carPoint.carPos[0][3]/1000;
  eetrans[1]=carPoint.carPos[1][3]/1000;
  eetrans[2]=carPoint.carPos[2][3]/1000;
  int ret = noid.Solve_LeftArmIk(eerot,eetrans,iksol);
  if(ret){
    std::cout << "ERROR : Wrong Point" << std::endl;
    RETURNID_NG;
  }

  //現在位置の読み取り
  noid.getLeftJointAngle(JointPos);
  noid.Solve_LeftArmFK(start_eerot,startPos,JointPos);
  
  //gettimeofday(&sumstart, NULL);
  for(int i=0; i<=StrideNum ; i++){

    viaRot[0]=(eerot[0]-start_eerot[0])*(double)i/(double)StrideNum+start_eerot[0];
    viaRot[1]=(eerot[1]-start_eerot[1])*(double)i/(double)StrideNum+start_eerot[1];
    viaRot[2]=(eerot[2]-start_eerot[2])*(double)i/(double)StrideNum+start_eerot[2];
    viaRot[3]=(eerot[3]-start_eerot[3])*(double)i/(double)StrideNum+start_eerot[3];
    viaRot[4]=(eerot[4]-start_eerot[4])*(double)i/(double)StrideNum+start_eerot[4];
    viaRot[5]=(eerot[5]-start_eerot[5])*(double)i/(double)StrideNum+start_eerot[5];
    viaRot[6]=(eerot[6]-start_eerot[6])*(double)i/(double)StrideNum+start_eerot[6];
    viaRot[7]=(eerot[7]-start_eerot[7])*(double)i/(double)StrideNum+start_eerot[7];
    viaRot[8]=(eerot[8]-start_eerot[8])*(double)i/(double)StrideNum+start_eerot[8];

    viaPos[0]=(eetrans[0]-startPos[0])*(double)i/(double)StrideNum+startPos[0];
    viaPos[1]=(eetrans[1]-startPos[1])*(double)i/(double)StrideNum+startPos[1];
    viaPos[2]=(eetrans[2]-startPos[2])*(double)i/(double)StrideNum+startPos[2];
    
    via_Rotation = noid.Solve_Rot(viaRot);
    noid.TransRot(viaRot,via_Rotation.roll,via_Rotation.pitch,via_Rotation.yaw);

    for(int j=0;j<9;j++){
      if(viaRot[j]>1)
	viaRot[j] = 1;
      if(viaRot[j]<-1)
	viaRot[j] = -1;
      if(viaRot[j] < 0.0001 && viaRot[j] > -0.0001)
	viaRot[j] = 0;
    }
    /*
    std::cout << "viaroll = " << via_Rotation.roll <<std::endl;
    std::cout << "viapitch = " << via_Rotation.pitch <<std::endl;
    std::cout << "viayaw = " << via_Rotation.yaw <<std::endl;
    for(int l=0;l<9;l++)
      {
	std::cout << "viaRot["<<l<<"] = "<<viaRot[l]<<std::endl;
      }
    std::cout << "viaPos[0] = " << viaPos[0] <<std::endl;
    std::cout << "viaPos[1] = " << viaPos[1] <<std::endl;
    std::cout << "viaPos[2] = " << viaPos[2] <<std::endl;
    */
    int ret = noid.Solve_LeftArmIk(viaRot,viaPos,iksol);
    if(!ret){
      
      noid.setLeftJointAngle(iksol);

      if(first!=0)
	{
	  gettimeofday(&end, NULL);
	  sec = (double)(end.tv_sec - start.tv_sec);
	  micro = (double)(end.tv_usec - start.tv_usec);
	  duration = sec + micro / 1000.0 / 1000.0;
	  std::cout << "duration = " << duration <<std::endl;
	  if(duration<30/1000){
	    usleeptime = 1000*30 - (duration * 1000*1000);
	    usleep(usleeptime);
	  }
	}
      gettimeofday(&start, NULL);
      
      noid.SeedAction(30);
      first = 1;
    }
    else{
      RETURNID_NG;
    }
    
  }
  /*
  gettimeofday(&sumend, NULL);
  sec = (double)(sumend.tv_sec - sumstart.tv_sec);
  micro = (double)(sumend.tv_usec - sumstart.tv_usec);
  duration = sec + micro / 1000.0 / 1000.0; 
  */

  std::cout << "Success" <<std::endl;
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveLinearCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
  std::cout<<"moveLinearCartesianRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
  std::cout<<"movePTPCartesianAbs"<<std::endl;
  
  //JointSpeed判定
  if((LeftJointSpeed <= 0)||(LeftJointSpeed > 100)){
    std::cout<<"ERROR : Wrong Joint Speed"<<std::endl<<std::endl;
    RETURNID_NG;
  }

  double eerot[9];//姿勢
  double eetrans[3];//座標
  double iksol[7];
  int movetime;
  
  //JointSpeedが70%で約3秒
  movetime = (int)(-100.71*LeftJointSpeed + 10101);

  //姿勢の入力
  eerot[0]=carPoint.carPos[0][0];
  eerot[1]=carPoint.carPos[0][1];
  eerot[2]=carPoint.carPos[0][2];
  
  eerot[3]=carPoint.carPos[1][0];
  eerot[4]=carPoint.carPos[1][1];
  eerot[5]=carPoint.carPos[1][2];
  
  eerot[6]=carPoint.carPos[2][0];
  eerot[7]=carPoint.carPos[2][1];
  eerot[8]=carPoint.carPos[2][2];

  for(int j=0;j<9;j++){
    printf("eerot[%d] = %f\n",j,eerot[j]);
  }

  //座標の入力 関数への入力は[mm]だがikfastは[m]のため変換
  eetrans[0]=carPoint.carPos[0][3]/1000;
  eetrans[1]=carPoint.carPos[1][3]/1000;
  eetrans[2]=carPoint.carPos[2][3]/1000;

  for(int i=0;i<3;i++){
    printf("eetrans[%d] = %f\n",i,eetrans[i]);
  }

  int ret = noid.Solve_LeftArmIk(eerot,eetrans,iksol);
  if(!ret){
    noid.setLeftJointAngle(iksol);
    noid.SeedAction(movetime);
  }
  else
    RETURNID_NG;

  std::cout << "Success" <<std::endl;
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPoint)
{
  std::cout<<"movePTPCartesianRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointAbs(const JARA_ARM_LEFT::JointPos& jointPoints)
{
  std::cout << "movePTPJointAbs" << std::endl;

  //JointSpeed判定
  if((LeftJointSpeed <= 0)||(LeftJointSpeed > 100)){
    std::cout<<"ERROR : Wrong Joint Speed"<<std::endl<<std::endl;
    RETURNID_NG;
  }

  double targetJointPos[10];
  int movetime;

  for (int i = 0; i<7; i++){
    std::cout << "JointPoint[" << i << "] = " << jointPoints[i] << std::endl;
  }

  //JointSpeedが70%で約3秒
  movetime = (int)(-100.71*LeftJointSpeed + 10101);
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  targetJointPos[0] = (double)jointPoints[0];
  targetJointPos[1] = (double)jointPoints[1];
  targetJointPos[2] = (double)jointPoints[2];
  targetJointPos[3] = (double)jointPoints[3];
  targetJointPos[4] = (double)jointPoints[4];
  targetJointPos[5] = (double)jointPoints[5];
  targetJointPos[6] = (double)jointPoints[6];
  
  //std::cout << "setJointAngle_Middle" << std::endl;
  noid.setLeftJointAngle(targetJointPos);
  
  //std::cout << "ArmAction_Middle" << std::endl;
  noid.SeedAction(movetime);
  
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::movePTPJointRel(const JARA_ARM_LEFT::JointPos& jointPoints)
{
  std::cout<<"movePTPJointRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::openGripper()
{
  std::cout << "openLeftGripper" << std::endl;
  noid.OpenLeftGripper();
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::pause()
{
  std::cout<<"pause"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::resume()
{
  std::cout<<"resume"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::stop()
{
  std::cout<<"stop"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeCartesian(::CORBA::Double aclTime)
{
  std::cout<<"setAccelTimeCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setAccelTimeJoint(::CORBA::Double aclTime)
{
  std::cout<<"setAccelTimeJoint"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setBaseOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
  std::cout<<"setBaseOffset"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setControlPointOffset(const JARA_ARM_LEFT::HgMatrix offset)
{
  std::cout<<"setControlPointOffset"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedCartesian(const JARA_ARM_LEFT::CartesianSpeed& speed)
{
  std::cout<<"setMaxSpeedCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMaxSpeedJoint(const JARA_ARM_LEFT::DoubleSeq& speed)
{
  std::cout<<"setMaxSpeedJoint"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeCartesian(::CORBA::Double aclTime)
{
  std::cout<<"setMinAccelTimeCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setMinAccelTimeJoint(::CORBA::Double aclTime)
{
  std::cout<<"setMinAccelTimeJoint"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSoftLimitCartesian(const JARA_ARM_LEFT::LimitValue& xLimit, const JARA_ARM_LEFT::LimitValue& yLimit, const JARA_ARM_LEFT::LimitValue& zLimit)
{
  std::cout<<"setSoftLimitCartesian"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedCartesian(JARA_ARM_LEFT::ULONG spdRatio)
{
  std::cout<<"setSpeedCartesian"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    LeftCartesianSpeed = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setSpeedJoint(JARA_ARM_LEFT::ULONG spdRatio)
{
  std::cout<<"setSpeedJoint"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    LeftJointSpeed = spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianAbs(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
  std::cout<<"moveCircularCartesianAbs"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::moveCircularCartesianRel(const JARA_ARM_LEFT::CarPosWithElbow& carPointR, const JARA_ARM_LEFT::CarPosWithElbow& carPointT)
{
  std::cout<<"moveCircularCartesianRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::setHome(const JARA_ARM_LEFT::JointPos& jointPoint)
{
  std::cout<<"setHome"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::getHome(JARA_ARM_LEFT::JointPos_out jointPoint)
{
  std::cout<<"getHome"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM_LEFT::RETURN_ID* JARA_ARM_LEFT_LeftManipulatorCommonInterface_MiddleSVC_impl::goHome()
{
  std::cout<<"goHome"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}



// End of example implementational code



