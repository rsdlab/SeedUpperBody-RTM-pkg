// -*-C++-*-
/*!
 * @file  SeedNeckInterfaceSVC_impl.cpp
 * @brief Service implementation code of SeedNeckInterface.idl
 *
 */

#include "SeedNeckInterfaceSVC_impl.h"
#include "LibSeednoidUpper.h"
#include "neck_ReturnID.h"

int NeckJointSpeed = 70;

/*
 * Example implementational code for IDL interface SeedNeck::SeedNeckInterface
 */
SeedNeck_SeedNeckInterfaceSVC_impl::SeedNeck_SeedNeckInterfaceSVC_impl()
{
  // Please add extra constructor code here.
}


SeedNeck_SeedNeckInterfaceSVC_impl::~SeedNeck_SeedNeckInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::getSoftLimitNeck(SeedNeck::LimitSeq_out softLimit)
{
  std::cout<<"getsoftLimitNeck"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::setSoftLimitNeck(const SeedNeck::LimitSeq& softLimit)
{
  std::cout<<"setsoftLimitNeck"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::getFeedbackPosNeck(SeedNeck::NeckPos_out pos)
{
  std::cout<<"getFeedbackPosNeck"<<std::endl;
  
  Frame Neckpos;
  pos = new SeedNeck::NeckPos;
  pos->length(3);

  Neckpos = noid.readNeckJointAngle();

  (*pos)[0] = Neckpos.yaw;
  (*pos)[1] = Neckpos.pitch;
  (*pos)[2] = Neckpos.roll;
  
  //for (int i = 0; i<3; i++)
  //  {
  //    std::cout << "Neckpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
  //  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::movePTPNeckAbs(const SeedNeck::NeckPos& neckPoints)
{
  std::cout << "movePTPNeckAbs" << std::endl;
  Frame pos;
  int movetime;

  //JointSpeed判定
  if((NeckJointSpeed <= 0)||(NeckJointSpeed > 100)){
    std::cout<<"ERROR : Wrong Neck Speed"<<std::endl<<std::endl;
    RETURNID_NG;
  }
  //JointSpeedが70%で約3秒
  movetime = (int)(-100.71*NeckJointSpeed + 10101);

  std::cout << "neckyaw   = " << neckPoints[0] << std::endl;
  std::cout << "neckpitch = " << neckPoints[1] << std::endl;
  std::cout << "neckroll   = " << neckPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)neckPoints[0];
  pos.pitch = (double)neckPoints[1];
  pos.roll   = (double)neckPoints[2];
  
  //std::cout << "setJointAngle_Middle" << std::endl;
  noid.setNeckJointAngle(pos);
  
  //std::cout << "ArmAction_Middle" << std::endl;
  noid.SeedAction(movetime);

  //sleep(movetime/1000);
  
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::movePTPNeckRel(const SeedNeck::NeckPos& neckPoints)
{
  std::cout<<"movePTPNeckRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedNeck::RETURN_ID* SeedNeck_SeedNeckInterfaceSVC_impl::setSpeedNeck(SeedNeck::ULONG spdRatio)
{
  std::cout<<"setSpeedNeck"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    NeckJointSpeed = (int)spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}



// End of example implementational code



