//
// Created by Sesar on 2025/10/25.
//

#ifndef USART_RCC_H
#define USART_RCC_H

#include <stdint.h>
#include <stdbool.h>
    class Rcc
    {
    private:
        float linearMapping(float input, float input_min, float input_max, float output_min, float output_max);
        uint8_t *rx_data;
    public:
        uint32_t tick;
        uint32_t last_tick;
        uint8_t *rx_buffer_;
        bool connected;
        struct read_remote_
        {
            float channel_0;
            float channel_1;
            float channel_2;
            float channel_3;
            enum
            {
                S1_DEFAULT,
                S1_UP,
                S1_MID,
                S1_DOWN,
            }S1_toggle;
            enum
            {
                S2_DEFAULT,
                S2_UP,
                S2_MID,
                S2_DOWN,
            }S2_toggle;
        }read_remote;
        void handle();
        void init();
        ~Rcc();
    };


#endif //USART_RCC_H
