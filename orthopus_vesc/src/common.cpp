#include "orthopus_vesc/common.hpp"

#include <byteswap.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace orthopus
{

uint16_t f_u16(float v, unsigned int scale)
{
  float scaled = v * (float)scale;
  scaled = std::clamp(scaled, (float)INT16_MIN, (float)INT16_MAX);
  return __bswap_16((int16_t)scaled);
}

float u16_f(uint16_t v, unsigned int scale)
{
  return ((float)(int16_t)(__bswap_16(v))) / (float)scale;
}

const char* state_to_text(uint16_t st)
{
  st &= orthopus::ORTHOPUS_STATE_MSK;
  if (st == orthopus::ORTHOPUS_STATE_INIT)
    return "Init";
  else if (st == orthopus::ORTHOPUS_STATE_IDLE)
    return "Idle";
  else if (st == orthopus::ORTHOPUS_STATE_ENABLE)
    return "Enable";
  else if (st == orthopus::ORTHOPUS_STATE_HOLD)
    return "Hold";
  else if (st == orthopus::ORTHOPUS_STATE_BRAKE)
    return "Brake";
  else if (st == orthopus::ORTHOPUS_STATE_ESTOP)
    return "EStop";
  else
    return "unknown";
};

const char* error_to_text(uint16_t st)
{
  st &= orthopus::ORTHOPUS_STATE_ERR_MSK;
  if (st == orthopus::ORTHOPUS_STATE_ERR_NONE)
    return "";
  else if (st == orthopus::ORTHOPUS_STATE_ERR_POS_STEP)
    return "PosStep";
  else if (st == orthopus::ORTHOPUS_STATE_ERR_VEL_STEP)
    return "VelStep";
  else if (st == orthopus::ORTHOPUS_STATE_ERR_TRQ_STEP)
    return "TrqStep";
  else if (st == orthopus::ORTHOPUS_STATE_ERR_OTHER)
    return "Other";
  else
    return "unknown";
}

const char* mode_to_text(uint16_t st)
{
  st &= orthopus::ORTHOPUS_CTRL_MODE_MSK;
  if (st == orthopus::ORTHOPUS_CTRL_MODE_CST)
    return "custom";
  else if (st == orthopus::ORTHOPUS_CTRL_MODE_IMP)
    return "impedance";
  else if (st == orthopus::ORTHOPUS_CTRL_MODE_TRQ)
    return "effort";
  else if (st == orthopus::ORTHOPUS_CTRL_MODE_VEL)
    return "velocity";
  else if (st == orthopus::ORTHOPUS_CTRL_MODE_POS)
    return "position";
  else if (st == orthopus::ORTHOPUS_CTRL_MODE_OFF)
    return "off";
  else
    return "unknown";
};
}  // namespace orthopus
