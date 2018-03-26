// -*- C++ -*-
/*!
 * @file  SeedUpperBodyRTC.cpp
 * @brief SeedUpperBodyRTC
 * @date $Date$
 *
 * $Id$
 */

#include "SeedUpperBodyRTC.h"
#include "LibSeednoidUpper.h"

// Module specification
// <rtc-template block="module_spec">
static const char* seedupperbodyrtc_spec[] =
  {
    "implementation_id", "SeedUpperBodyRTC",
    "type_name",         "SeedUpperBodyRTC",
    "description",       "SeedUpperBodyRTC",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.port_name", "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FT98HKZC-if00-port0",

    // Widget
    "conf.__widget__.port_name", "text",
    // Constraints

    "conf.__type__.port_name", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SeedUpperBodyRTC::SeedUpperBodyRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_RightManipulatorCommonInterface_CommonPort("RightManipulatorCommonInterface_Common"),
    m_RightManipulatorCommonInterface_MiddlePort("RightManipulatorCommonInterface_Middle"),
    m_LeftManipulatorCommonInterface_CommonPort("LeftManipulatorCommonInterface_Common"),
    m_LeftManipulatorCommonInterface_MiddlePort("LeftManipulatorCommonInterface_Middle"),
    m_SeedWaistInterfacePort("SeedWaistInterface"),
    m_SeedNeckInterfacePort("SeedNeckInterface")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SeedUpperBodyRTC::~SeedUpperBodyRTC()
{
}



RTC::ReturnCode_t SeedUpperBodyRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_RightManipulatorCommonInterface_CommonPort.registerProvider("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_RightManipulatorCommonInterface_MiddlePort.registerProvider("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);
  m_LeftManipulatorCommonInterface_CommonPort.registerProvider("LeftManipulatorCommonInterface_Common", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Common", m_LeftManipulatorCommonInterface_Common);
  m_LeftManipulatorCommonInterface_MiddlePort.registerProvider("LeftManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::LeftManipulatorCommonInterface_Middle", m_LeftManipulatorCommonInterface_Middle);
  m_SeedWaistInterfacePort.registerProvider("SeedWaistInterface", "SeedWaist::SeedWaistInterface", m_SeedWaistInterface);
  m_SeedNeckInterfacePort.registerProvider("SeedNeckInterface", "SeedNeck::SeedNeckInterface", m_SeedNeckInterface);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_RightManipulatorCommonInterface_CommonPort);
  addPort(m_RightManipulatorCommonInterface_MiddlePort);
  addPort(m_LeftManipulatorCommonInterface_CommonPort);
  addPort(m_LeftManipulatorCommonInterface_MiddlePort);
  addPort(m_SeedWaistInterfacePort);
  addPort(m_SeedNeckInterfacePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("port_name", m_port_name, "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FT98HKZC-if00-port0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SeedUpperBodyRTC::onActivated(RTC::UniqueId ec_id)
{
  if (!noid.OpenSerialPort(m_port_name.c_str())){
    std::cout << "Connect Error!" << std::endl;
  }

  noid.initPosition();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBodyRTC::onDeactivated(RTC::UniqueId ec_id)
{
  noid.CloseSerialPort();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedUpperBodyRTC::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedUpperBodyRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SeedUpperBodyRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(seedupperbodyrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<SeedUpperBodyRTC>,
                             RTC::Delete<SeedUpperBodyRTC>);
  }
  
};


