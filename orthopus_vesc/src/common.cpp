#include "orthopus_vesc/common.hpp"
#include <byteswap.h>

namespace orthopus
{

uint16_t f_u16(float v, unsigned int scale)
{
    return __bswap_16((int16_t)(v*scale));
}

float u16_f(uint16_t v, unsigned int scale)
{
    return ((float)(int16_t)(__bswap_16(v)))/(float)scale;
}

}
