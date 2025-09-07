#include "gpio_bitbang_uart.hpp"

namespace BitBangUART {

// ----------------- DWT 延时 -----------------
    void dwt_delay_init() {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
      DWT->CYCCNT = 0;
    }

    void delay_us(uint32_t us) {
      uint32_t start = DWT->CYCCNT;
      uint32_t ticks = (SystemCoreClock / 1000000UL) * us;
      while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
      }
    }

// ----------------- GPIO helper -----------------
    void GPIO_BitBangUART::gpio_set_output(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
      GPIO_InitTypeDef cfg = {0};
      cfg.Pin = pin;
      cfg.Mode = GPIO_MODE_OUTPUT_PP;
      cfg.Pull = GPIO_NOPULL;
      cfg.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(port, &cfg);
      HAL_GPIO_WritePin(port, pin, state);
    }

    void GPIO_BitBangUART::gpio_set_input_pullup(GPIO_TypeDef *port, uint16_t pin) {
      GPIO_InitTypeDef cfg = {0};
      cfg.Pin = pin;
      cfg.Mode = GPIO_MODE_INPUT;
      cfg.Pull = GPIO_PULLUP;
      cfg.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(port, &cfg);
    }

// ----------------- 类成员实现 -----------------
    GPIO_BitBangUART::GPIO_BitBangUART(GPIO_TypeDef *port, uint16_t pin, uint32_t baud)
            : port(port), pin(pin)
    {
      setBaud(baud);
      gpio_set_input_pullup(port, pin);
    }

    void GPIO_BitBangUART::setBaud(uint32_t baud) {
      this->baud = baud;
      this->bit_time_us = 1000000UL / baud;
    }

    void GPIO_BitBangUART::sendByte(uint8_t data) {
      gpio_set_output(port, pin, GPIO_PIN_SET);
      delay_us(2);

      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // start bit
      delay_us(bit_time_us);

      for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(port, pin, (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay_us(bit_time_us);
      }

      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); // stop bit
      delay_us(bit_time_us);
      delay_us(bit_time_us);

      gpio_set_input_pullup(port, pin);
    }

    uint8_t GPIO_BitBangUART::receiveByte(uint8_t *out, uint32_t timeout_ms) {
      gpio_set_input_pullup(port, pin);
      uint32_t start_tick = HAL_GetTick();

      while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start_tick) >= timeout_ms) return 1;
      }

      delay_us(bit_time_us / 2);

      if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_RESET) return 2;

      uint8_t data = 0;
      for (int i = 0; i < 8; i++) {
        delay_us(bit_time_us);
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
          data |= (1 << i);
        }
      }

      delay_us(bit_time_us);
      if (HAL_GPIO_ReadPin(port, pin) != GPIO_PIN_SET) return 3;

      *out = data;
      return true;
    }

    void GPIO_BitBangUART::sendBytes(const uint8_t *buf, uint32_t len) {
      for (uint32_t i = 0; i < len; i++) {
        sendByte(buf[i]);
        delay_us(bit_time_us * 2);
      }
    }

    void GPIO_BitBangUART::sendSyncNibble(uint8_t nibble) {
      sendByte(nibble & 0x0F);
    }

} // namespace BitBangUART
