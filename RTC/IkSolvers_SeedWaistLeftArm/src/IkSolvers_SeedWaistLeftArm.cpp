// -*- C++ -*-
/*!
 * @file  IkSolvers_SeedWaistLeftArm.cpp
 * @brief IkSolvers_SeedWaistLeftArm
 * @date $Date$
 *
 * $Id$
 */

#include "IkSolvers_SeedWaistLeftArm.h"

// Module specification
// <rtc-template block="module_spec">
static const char* iksolvers_seedwaistleftarm_spec[] =
  {
    "implementation_id", "IkSolvers_SeedWaistLeftArm",
    "type_name",         "IkSolvers_SeedWaistLeftArm",
    "description",       "IkSolvers_SeedWaistLeftArm",
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
IkSolvers_SeedWaistLeftArm::IkSolvers_SeedWaistLeftArm(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_WaustLeftArmIkPort("WaustLeftArmIk")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
IkSolvers_SeedWaistLeftArm::~IkSolvers_SeedWaistLeftArm()
{
}



RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_WaustLeftArmIkPort.registerProvider("WaistLeftArmIkInterface", "LEFT_IK_FAST::WaistLeftArmIkInterface", m_WaistLeftArmIkInterface);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_WaustLeftArmIkPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t IkSolvers_SeedWaistLeftArm::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void IkSolvers_SeedWaistLeftArmInit(RTC::Manager* manager)
  {
    coil::Properties profile(iksolvers_seedwaistleftarm_spec);
    manager->registerFactory(profile,
                             RTC::Create<IkSolvers_SeedWaistLeftArm>,
                             RTC::Delete<IkSolvers_SeedWaistLeftArm>);
  }
  
};


