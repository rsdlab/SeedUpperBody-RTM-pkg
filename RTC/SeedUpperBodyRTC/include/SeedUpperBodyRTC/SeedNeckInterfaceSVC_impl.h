// -*-C++-*-
/*!
 * @file  SeedNeckInterfaceSVC_impl.h
 * @brief Service implementation header of SeedNeckInterface.idl
 *
 */

#include "BasicDataTypeSkel.h"

#include "SeedNeckInterfaceSkel.h"

#ifndef SEEDNECKINTERFACESVC_IMPL_H
#define SEEDNECKINTERFACESVC_IMPL_H
 
/*!
 * @class SeedNeckInterfaceSVC_impl
 * Example class implementing IDL interface SeedNeck::SeedNeckInterface
 */
class SeedNeck_SeedNeckInterfaceSVC_impl
 : public virtual POA_SeedNeck::SeedNeckInterface,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~SeedNeckInterfaceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   SeedNeck_SeedNeckInterfaceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~SeedNeck_SeedNeckInterfaceSVC_impl();

   // attributes and operations
   SeedNeck::RETURN_ID* servoOFF();
   SeedNeck::RETURN_ID* servoON();
   SeedNeck::RETURN_ID* getSoftLimitNeck(SeedNeck::LimitSeq_out softLimit);
   SeedNeck::RETURN_ID* setSoftLimitNeck(const SeedNeck::LimitSeq& softLimit);
   SeedNeck::RETURN_ID* getFeedbackPosNeck(SeedNeck::NeckPos_out pos);
   SeedNeck::RETURN_ID* movePTPNeckAbs(const SeedNeck::NeckPos& neckPoints);
   SeedNeck::RETURN_ID* movePTPNeckRel(const SeedNeck::NeckPos& neckPoints);
   SeedNeck::RETURN_ID* setSpeedNeck(SeedNeck::ULONG spdRatio);

};



#endif // SEEDNECKINTERFACESVC_IMPL_H


