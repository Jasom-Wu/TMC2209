#pragma once
#include "stm32h7xx_hal.h"
#include <cstdint>

namespace BitBangUART {

    class GPIO_BitBangUART {
    public:
        GPIO_BitBangUART(GPIO_TypeDef *port, uint16_t pin, uint32_t baud);

        void setBaud(uint32_t baud);
        void sendByte(uint8_t data);
        uint8_t receiveByte(uint8_t *out, uint32_t timeout_ms);
        void sendBytes(const uint8_t *buf, uint32_t len);
        void sendSyncNibble(uint8_t nibble);

    private:
        GPIO_TypeDef *port;
        uint16_t pin;
        uint32_t baud;
        uint32_t bit_time_us;

        static void gpio_set_output(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
        static void gpio_set_input_pullup(GPIO_TypeDef *port, uint16_t pin);
    };

// 工具函数：必须先初始化 DWT 延时
    void dwt_delay_init();
    void delay_us(uint32_t us);

} // namespace BitBangUART
