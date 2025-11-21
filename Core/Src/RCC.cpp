//
// Created by Sesar on 2025/10/25.
//

#include "RCC.h"
float Rcc::linearMapping(float input, float input_min, float input_max, float output_min, float output_max)
{
    return (input - input_min) * (output_max - output_min) / (input_max - input_min) + output_min;
}
void Rcc::handle()
{
    rx_data = rx_buffer_;
    int8_t tmp = (rx_buffer_[5]<<2)&0xC0;
    tmp = (tmp>>6)&0x03;
    switch (tmp)
    {
    case 1:read_remote.S1_toggle = read_remote.S1_UP;break;
    case 2:read_remote.S1_toggle = read_remote.S1_DOWN;break;
    case 3:read_remote.S1_toggle = read_remote.S1_MID;break;
    default:read_remote.S1_toggle = read_remote.S1_DEFAULT;
    }
    tmp = (rx_buffer_[5]>>6)&0x03;
    switch (tmp)
    {
    case 1:read_remote.S2_toggle = read_remote.S2_UP;break;
    case 2:read_remote.S2_toggle = read_remote.S2_DOWN;break;
    case 3:read_remote.S2_toggle = read_remote.S2_MID;break;
    default:read_remote.S2_toggle = read_remote.S2_DEFAULT;
    }
    uint16_t tmp1 = ((rx_buffer_[1]<<8)&0x0700) + rx_buffer_[0];
    read_remote.channel_0 = linearMapping(tmp1, 364, 1684, -1.0f, 1.0f);
    if (read_remote.channel_0 < 0.005f && read_remote.channel_0 > -0.005f)
    {
        read_remote.channel_0 = 0.0f;
    }
    tmp1 = ((rx_buffer_[2]<<5)&0x07E0) + ((rx_buffer_[1]>>3)&0x001F);
    read_remote.channel_1 = linearMapping(tmp1, 364, 1684, -1.0f, 1.0f);
    if (read_remote.channel_1 < 0.005f && read_remote.channel_1 > -0.005f)
    {
        read_remote.channel_1 = 0.0f;
    }
    tmp1 = ((rx_buffer_[4]<<10)&0x0400) + ((rx_buffer_[3]<<2)&0x03FC) + ((rx_buffer_[2]>>6)&0x0003);
    read_remote.channel_2 = linearMapping(tmp1, 364, 1684, -1.0f, 1.0f);
    if (read_remote.channel_2 < 0.005f && read_remote.channel_2 > -0.005f)
    {
        read_remote.channel_2 = 0.0f;
    }
    tmp1 = ((rx_buffer_[5]<<7)&0x0780) + ((rx_buffer_[4]>>1)&0x007F);
    read_remote.channel_3 = linearMapping(tmp1, 364, 1684, -1.0f, 1.0f);
    if (read_remote.channel_3 < 0.005f && read_remote.channel_3 > -0.005f)
    {
        read_remote.channel_3 = 0.0f;
    }
}

void Rcc::init()
{
    connected = false;
    read_remote.S1_toggle = read_remote.S1_DEFAULT;
    read_remote.S2_toggle = read_remote.S2_DEFAULT;
    read_remote.channel_0 = 0.0f;
    read_remote.channel_1 = 0.0f;
    read_remote.channel_2 = 0.0f;
    read_remote.channel_3 = 0.0f;
}

Rcc::~Rcc()
{
    delete[] rx_buffer_;
    delete[] rx_data;
}




