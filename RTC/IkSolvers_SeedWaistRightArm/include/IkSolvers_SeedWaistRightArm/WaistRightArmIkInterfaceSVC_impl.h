// -*-C++-*-
/*!
 * @file  WaistRightArmIkInterfaceSVC_impl.h
 * @brief Service implementation header of WaistRightArmIkInterface.idl
 *
 */

#include "BasicDataTypeSkel.h"

#include "WaistRightArmIkInterfaceSkel.h"

#ifndef WAISTRIGHTARMIKINTERFACESVC_IMPL_H
#define WAISTRIGHTARMIKINTERFACESVC_IMPL_H
/********* includeファイル *********/
//数学定数の使用
#define _USE_MATH_DEFINES
//#include <cmath>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include "ikfast.h"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
 
#define ARM_FREEDOM 7
//腕関節の可動範囲[deg]
//アーム第1軸モータの可動範囲
#define	Angle1_LimitMax	89
#define	Angle1_LimitMin	-20
//アーム第2軸モータの可動範囲
#define	Angle2_LimitMax	90
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

/*!
 * @class WaistRightArmIkInterfaceSVC_impl
 * Example class implementing IDL interface RIGHT_IK_FAST::WaistRightArmIkInterface
 */
class RIGHT_IK_FAST_WaistRightArmIkInterfaceSVC_impl
 : public virtual POA_RIGHT_IK_FAST::WaistRightArmIkInterface,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~WaistRightArmIkInterfaceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RIGHT_IK_FAST_WaistRightArmIkInterfaceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RIGHT_IK_FAST_WaistRightArmIkInterfaceSVC_impl();

   // attributes and operations
   RIGHT_IK_FAST::RETURN_ID* solveRightArmIkSetWaistJoint(const RIGHT_IK_FAST::Rotation& eerot, const RIGHT_IK_FAST::FramePos& eetrans, const RIGHT_IK_FAST::WaistFrame& WaistPos, const RIGHT_IK_FAST::JointPos& nowJointPos, RIGHT_IK_FAST::JointPos_out solRightJointPos);
   RIGHT_IK_FAST::RETURN_ID* solveRightArmIk(const RIGHT_IK_FAST::Rotation& eerot, const RIGHT_IK_FAST::FramePos& eetrans, const RIGHT_IK_FAST::JointPos& nowJointPos, RIGHT_IK_FAST::JointPos_out solRightJointPos);
   RIGHT_IK_FAST::RETURN_ID* solveRightArmFk(const RIGHT_IK_FAST::JointPos& RightJointPos, RIGHT_IK_FAST::Rotation_out eerot, RIGHT_IK_FAST::FramePos_out eetrans);

};



#endif // WAISTRIGHTARMIKINTERFACESVC_IMPL_H


