// -*- C++ -*-
/*!
 * @file  IkSolvers_SeedWaistRightArm.cpp
 * @brief IkSolvers_SeedWaistRightArm
 * @date $Date$
 *
 * $Id$
 */

#include "IkSolvers_SeedWaistRightArm.h"

// Module specification
// <rtc-template block="module_spec">
static const char* iksolvers_seedwaistrightarm_spec[] =
  {
    "implementation_id", "IkSolvers_SeedWaistRightArm",
    "type_name",         "IkSolvers_SeedWaistRightArm",
    "description",       "IkSolvers_SeedWaistRightArm",
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
IkSolvers_SeedWaistRightArm::IkSolvers_SeedWaistRightArm(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_WaustRightArmIkPort("WaustRightArmIk")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
IkSolvers_SeedWaistRightArm::~IkSolvers_SeedWaistRightArm()
{
}



RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_WaustRightArmIkPort.registerProvider("WaistRightArmIkInterface", "RIGHT_IK_FAST::WaistRightArmIkInterface", m_WaistRightArmIkInterface);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_WaustRightArmIkPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistRightArm::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void IkSolvers_SeedWaistRightArmInit(RTC::Manager* manager)
  {
    coil::Properties profile(iksolvers_seedwaistrightarm_spec);
    manager->registerFactory(profile,
                             RTC::Create<IkSolvers_SeedWaistRightArm>,
                             RTC::Delete<IkSolvers_SeedWaistRightArm>);
  }
  
};


