// -*-C++-*-
/*!
 * @file  SeedWaistInterfaceSVC_impl.cpp
 * @brief Service implementation code of SeedWaistInterface.idl
 *
 */

#include "SeedWaistInterfaceSVC_impl.h"
#include "LibSeednoidUpper.h"
#include "waist_ReturnID.h"

int WaistJointSpeed = 70;

/*
 * Example implementational code for IDL interface SeedWaist::SeedWaistInterface
 */
SeedWaist_SeedWaistInterfaceSVC_impl::SeedWaist_SeedWaistInterfaceSVC_impl()
{
  // Please add extra constructor code here.
}


SeedWaist_SeedWaistInterfaceSVC_impl::~SeedWaist_SeedWaistInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::getSoftLimitWaist(SeedWaist::LimitSeq_out softLimit)
{
  std::cout<<"getsoftLimitWaist"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::setSoftLimitWaist(const SeedWaist::LimitSeq& softLimit)
{
  std::cout<<"setsoftLimitWaist"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::getFeedbackPosWaist(SeedWaist::WaistPos_out pos)
{
  std::cout<<"getFeedbackPosWaist"<<std::endl;

  Frame Waistpos;
  pos = new SeedWaist::WaistPos;
  pos->length(3);
  Waistpos = noid.readWaistJointAngle();

  (*pos)[0] = Waistpos.yaw;
  (*pos)[1] = Waistpos.pitch;
  (*pos)[2] = Waistpos.roll;

  //for (int i = 0; i<3; i++)
  //  {
  //    std::cout << "Waistpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
  //  }

  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::movePTPWaistAbs(const SeedWaist::WaistPos& waistPoints)
{
  std::cout << "movePTPWaistAbs" << std::endl;
  Frame pos;
  int movetime;
  //JointSpeed判定
  if((WaistJointSpeed <= 0)||(WaistJointSpeed > 100)){
    std::cout<<"ERROR : Wrong Waist Speed"<<std::endl<<std::endl;
    RETURNID_NG;
  }
  //JointSpeedが70%で約3秒
  movetime = (int)(-100.71*WaistJointSpeed + 10101);

  std::cout << "waistpitch = " << waistPoints[1] << std::endl;
  std::cout << "waistyaw   = " << waistPoints[0] << std::endl;
  std::cout << "waistroll  = " << waistPoints[2] << std::endl;
  
  //std::cout << "targetJointPos_Middle" << std::endl;
  pos.yaw  = (double)waistPoints[0];
  pos.pitch = (double)waistPoints[1];
  pos.roll   = (double)waistPoints[2];
  
  //std::cout << "setJointAngle_Middle" << std::endl;
  noid.setWaistJointAngle(pos);
  
  //std::cout << "ArmAction_Middle" << std::endl;
  noid.SeedAction(movetime);

  //sleep(movetime/1000);
  
  std::cout << "Success" << std::endl << std::endl;
  RETURNID_OK;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::movePTPWaistRel(const SeedWaist::WaistPos& waistPoints)
{
  std::cout<<"movePTPWaistRel"<<std::endl;
  std::cout<<"ERROR : コマンド未実装"<<std::endl<<std::endl;
  RETURNID_NOT_IMPLEMENTED;
}

SeedWaist::RETURN_ID* SeedWaist_SeedWaistInterfaceSVC_impl::setSpeedWaist(SeedWaist::ULONG spdRatio)
{
  std::cout<<"setSpeedWaist"<<std::endl;
  
  if(spdRatio >= 0 && spdRatio <= 100){
    WaistJointSpeed = (int)spdRatio;
  }else{
    std::cout<<"ERROR : Wrong Value"<<std::endl<<std::endl;
    RETURNID_VALUE_ERR;
  }
  
  std::cout<<"Success"<<std::endl<<std::endl;
  
  RETURNID_OK;
}



// End of example implementational code



