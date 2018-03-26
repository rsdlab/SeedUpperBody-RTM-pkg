// -*-C++-*-
/*!
 * @file  SeedWaistInterfaceSVC_impl.h
 * @brief Service implementation header of SeedWaistInterface.idl
 *
 */

#include "BasicDataTypeSkel.h"

#include "SeedWaistInterfaceSkel.h"

#ifndef SEEDWAISTINTERFACESVC_IMPL_H
#define SEEDWAISTINTERFACESVC_IMPL_H
 
/*!
 * @class SeedWaistInterfaceSVC_impl
 * Example class implementing IDL interface SeedWaist::SeedWaistInterface
 */
class SeedWaist_SeedWaistInterfaceSVC_impl
 : public virtual POA_SeedWaist::SeedWaistInterface,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~SeedWaistInterfaceSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   SeedWaist_SeedWaistInterfaceSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~SeedWaist_SeedWaistInterfaceSVC_impl();

   // attributes and operations
   SeedWaist::RETURN_ID* servoOFF();
   SeedWaist::RETURN_ID* servoON();
   SeedWaist::RETURN_ID* getSoftLimitWaist(SeedWaist::LimitSeq_out softLimit);
   SeedWaist::RETURN_ID* setSoftLimitWaist(const SeedWaist::LimitSeq& softLimit);
   SeedWaist::RETURN_ID* getFeedbackPosWaist(SeedWaist::WaistPos_out pos);
   SeedWaist::RETURN_ID* movePTPWaistAbs(const SeedWaist::WaistPos& waistPoints);
   SeedWaist::RETURN_ID* movePTPWaistRel(const SeedWaist::WaistPos& waistPoints);
   SeedWaist::RETURN_ID* setSpeedWaist(SeedWaist::ULONG spdRatio);

};



#endif // SEEDWAISTINTERFACESVC_IMPL_H


