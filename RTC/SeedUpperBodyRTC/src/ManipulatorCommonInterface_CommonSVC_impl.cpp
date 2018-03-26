// -*-C++-*-
/*!
 * @file  ManipulatorCommonInterface_CommonSVC_impl.cpp
 * @brief Service implementation code of ManipulatorCommonInterface_Common.idl
 *
 */

#include "ManipulatorCommonInterface_CommonSVC_impl.h"
#include "LibSeednoidUpper.h"
#include "right_ReturnID.h"

/*
 * Example implementational code for IDL interface JARA_ARM::ManipulatorCommonInterface_Common
 */
JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra constructor code here.
}


JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::~JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::clearAlarms()
{
  std::cout << "clearAlarms" << std::endl;
  std::cout << "ERROR : コマンド未実装" << std::endl << std::endl;
  
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::getActiveAlarm(JARA_ARM::AlarmSeq_out alarms)
{
  std::cout << "getActiveAlarm" << std::endl;
  alarms = new JARA_ARM::AlarmSeq;
  alarms->length(3);
  std::cout << "ERROR : コマンド未実装" << std::endl << std::endl;
  
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::getFeedbackPosJoint(JARA_ARM::JointPos_out pos)
{
  std::cout << "getFeedbackPosJoint" << std::endl;
  
  double JointPos[7];
  pos = new JARA_ARM::JointPos;
  pos->length(7);
  
  noid.readRightJointAngle(JointPos);
  
  for (int i = 0; i<7; i++)
    {
      (*pos)[i] = JointPos[i];
      //std::cout << "Rightpos[" << i << "] = " << (*pos)[i] << "[°]" << std::endl;
    }
  
  std::cout << "Success" << std::endl << std::endl;

  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::getManipInfo(JARA_ARM::ManipInfo_out mInfo)
{
  std::cout << "GetManipInfo" << std::endl;
  
  mInfo = new JARA_ARM::ManipInfo;
  mInfo->manufactur = "Hardware：(株)THK  Software：Meijo University robot systems design laboratory";
  mInfo->type = "Seed_Arm";
  mInfo->cmdCycle = 30;
  mInfo->axisNum = 7;
  mInfo->isGripper = true;

  
  std::cout << " manufactur : " << mInfo->manufactur << std::endl;
  std::cout << " type       : " << mInfo->type << std::endl;
  std::cout << " axisNum    : " << mInfo->axisNum << std::endl;
  std::cout << " cmdCycle   : " << mInfo->cmdCycle << std::endl;
  std::cout << " isGripper  : " << mInfo->isGripper << std::endl;

  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::getSoftLimitJoint(JARA_ARM::LimitSeq_out softLimit)
{
  std::cout << "getSoftLimitJoint" << std::endl;
  //JLimit JointLimit[7];
  softLimit = new JARA_ARM::LimitSeq;
  softLimit->length(7);
  std::cout << "ERROR : コマンド未実装" << std::endl << std::endl;
  
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::getState(JARA_ARM::ULONG& state)
{
  std::cout << "getState" << std::endl;
  std::cout << "ERROR : コマンド未実装" << std::endl << std::endl;
  
  RETURNID_NOT_IMPLEMENTED;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::servoOFF()
{
  int torque = 0;
  std::cout << "ServoOFF (SERVO_OFF)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::servoON()
{
  int torque = 1;
  std::cout << "ServoON (SERVO_ON)" << std::endl;
  
  noid.ServoOnOff(torque);
  
  std::cout << "Success" << std::endl << std::endl;
  
  RETURNID_OK;
}

JARA_ARM::RETURN_ID* JARA_ARM_ManipulatorCommonInterface_CommonSVC_impl::setSoftLimitJoint(const JARA_ARM::LimitSeq& softLimit)
{
  std::cout << "setSoftLimitJoint" << std::endl;
  std::cout << "ERROR : コマンド未実装" << std::endl << std::endl;
  
  RETURNID_NOT_IMPLEMENTED;
}



// End of example implementational code



