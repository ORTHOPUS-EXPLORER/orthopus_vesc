#pragma once

#include "vescpp/comm/can.hpp"
#include "vescpp/vescpp.hpp"

namespace orthopus
{
constexpr vescpp::comm::CAN::Id CAN_RT_DATA_UPSTREAM = 179;
constexpr vescpp::comm::CAN::Id CAN_RT_DATA_DOWNSTREAM = 180;
//constexpr vescpp::comm::CAN::Id CAN_RT_DATA_UPSTREAM   = 181;
constexpr vescpp::comm::CAN::Id CAN_AUX_DATA_DOWNSTREAM = 182;

constexpr unsigned int ORTHOPUS_COMM_RT_POS_SCALE = 5000;  //  0->6.28 rad (0->360 deg)
constexpr unsigned int ORTHOPUS_COMM_RT_VEL_SCALE = 5900;  // -5.5->5.5 rad/S (-50->50 rpm)
constexpr unsigned int ORTHOPUS_COMM_RT_TRQ_SCALE = 600;   // -50->50 Nm
constexpr unsigned int ORTHOPUS_COMM_AUX_SERVO_SCALE = 1000;

constexpr uint16_t ORTHOPUS_CTRL_MODE_OFF = 0x0000;  // 0000 0000 0000
constexpr uint16_t ORTHOPUS_CTRL_MODE_POS = 0x0001;  // 0000 0000 0001
constexpr uint16_t ORTHOPUS_CTRL_MODE_VEL = 0x0002;  // 0000 0000 0010
constexpr uint16_t ORTHOPUS_CTRL_MODE_TRQ = 0x0004;  // 0000 0000 0100
constexpr uint16_t ORTHOPUS_CTRL_MODE_IMP = 0x0007;  // 0000 0000 0111
constexpr uint16_t ORTHOPUS_CTRL_MODE_CST = 0x000F;  // 0000 0000 1111 - custom mode
constexpr uint16_t ORTHOPUS_CTRL_MODE_MSK = 0x000F;  // 0000 0000 1111 - Mode mask

constexpr uint16_t ORTHOPUS_STATE_ERR_NONE = 0x0000;      // 0000 0001 0000
constexpr uint16_t ORTHOPUS_STATE_ERR_POS_STEP = 0x0010;  // 0000 0001 0000
constexpr uint16_t ORTHOPUS_STATE_ERR_VEL_STEP = 0x0020;  // 0000 0010 0000
constexpr uint16_t ORTHOPUS_STATE_ERR_TRQ_STEP = 0x0040;  // 0000 0100 0000
constexpr uint16_t ORTHOPUS_STATE_ERR_OTHER = 0x0080;     // 0000 1000 0000
constexpr uint16_t ORTHOPUS_STATE_ERR_MSK = 0x00F0;       // 0000 1111 0000 - Error mask

constexpr uint16_t ORTHOPUS_STATE_INIT = 0x0000;    // 0000 0000 0000
constexpr uint16_t ORTHOPUS_STATE_IDLE = 0x0100;    // 0001 0000 0000
constexpr uint16_t ORTHOPUS_STATE_ENABLE = 0x0200;  // 0010 0000 0000
constexpr uint16_t ORTHOPUS_STATE_HOLD = 0x0300;    // 0011 0000 0000
constexpr uint16_t ORTHOPUS_STATE_BRAKE = 0x0400;   // 0100 0000 0000
constexpr uint16_t ORTHOPUS_STATE_ESTOP = 0x0500;   // 0101 0000 0000
constexpr uint16_t ORTHOPUS_STATE_MSK = 0x0F00;     // 1111 0000 0000 - State mask

const char* State2Text(uint16_t st);
const char* Err2Text(uint16_t st);
const char* Mode2Text(uint16_t st);

uint16_t f_u16(float v, unsigned int scale);
float u16_f(uint16_t v, unsigned int scale);

enum class JointVariableType
{
  POSITION,
  VELOCITY,
  ACCELERATION,
  EFFORT
};

constexpr const char* JointVariableType_to_string(JointVariableType joint_variable_type)
{
  switch (joint_variable_type)
  {
    case JointVariableType::POSITION:
      return "position";
    case JointVariableType::VELOCITY:
      return "velocity";
    case JointVariableType::ACCELERATION:
      return "acceleration";
    case JointVariableType::EFFORT:
      return "effort";
  }
}

constexpr JointVariableType JointVariableType_from_string(const std::string_view& joint_variable_str)
{
  if (joint_variable_str == "acceleration")
    return JointVariableType::ACCELERATION;
  else if (joint_variable_str == "effort")
    return JointVariableType::EFFORT;
  else if (joint_variable_str == "position")
    return JointVariableType::POSITION;
  else if (joint_variable_str == "velocity")
    return JointVariableType::VELOCITY;
  throw std::out_of_range("Input string is not a valid JointVariableType");
}

typedef union
{
  uint8_t raw[8];
  struct
  {
    uint16_t qm;
    uint16_t dqm;
    uint16_t taum;
    uint16_t status;
  } f;
} RTDataUS;
typedef union
{
  uint8_t raw[8];
  struct
  {
    uint16_t qd;
    uint16_t dqd;
    uint16_t tauf;
    uint16_t ctrl;
  } f;
} RTDataDS;
typedef union
{
  uint8_t raw[2];
  struct
  {
    uint16_t servo;
  } f;
} AuxDataDS;
}  // namespace orthopus
