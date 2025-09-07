#pragma once
#include "stm32h7xx_hal.h"
#include "gpio_bitbang_uart.hpp"
#include <cstdint>


#define TMCDelayms(xms) HAL_Delay(xms)
#define ML_PER_STEP 0.000006
namespace TMC2209 {

    class Motor {
    public:
        Motor(BitBangUART::GPIO_BitBangUART* uart,
              TIM_HandleTypeDef* timer,
              uint32_t channel,
              GPIO_TypeDef* step_port, uint16_t step_pin,
              GPIO_TypeDef* dir_port, uint16_t dir_pin,
              GPIO_TypeDef* enn_port, uint16_t enn_pin,
              uint8_t id = 0,
              uint8_t address = 0x00);

        // 电机控制方法
        void start();
        void stop();
        void step(uint32_t steps,GPIO_PinState dir=GPIO_PIN_SET);
        void stepms(uint32_t Xms, GPIO_PinState dir=GPIO_PIN_SET);
        void stepml(uint16_t Xml, GPIO_PinState dir=GPIO_PIN_SET);
        void setDirection(GPIO_PinState state);
        void enableDriver(GPIO_PinState state);
        void setSpeed(uint32_t stepFrequency);
        void setSpreadCycle(bool enable);
        void setMicrosteppingResolution(uint16_t resolution);
        void setIRUN(uint8_t irun_value);
        void configureGCONF();
        void resetSteps();


        BitBangUART::GPIO_BitBangUART* uart;
        TIM_HandleTypeDef* htim;
        uint32_t step_channel;
        GPIO_TypeDef* step_port;
        uint16_t step_pin;
        GPIO_TypeDef* dir_port;
        uint16_t dir_pin;
        GPIO_TypeDef* enn_port;
        uint16_t enn_pin;

        uint8_t id;
        uint8_t address;
        uint32_t stepsTaken;
        uint32_t nextTotalSteps;
        bool isStepping;
    private:
        void countSteps(uint32_t totalSteps);
        void writeRegister(uint8_t regAddress, int32_t value) const;



    };
    constexpr uint8_t SYNC = 0x05; // 示例
    constexpr uint8_t TMC_WRITE_DATAGRAM_SIZE = 8;
    constexpr uint8_t TMC2209_REG_GCONF = 0x00;
    constexpr uint8_t TMC2209_REG_CHOPCONF = 0x6C;
    constexpr uint8_t TMC2209_REG_IHOLD_IRUN = 0x10;
    constexpr uint8_t TMC2209_EN_SPREADCYCLE_POS = 2;
// 全局电机数组
    constexpr int MAX_MOTORS = 3;
    extern Motor *motors[MAX_MOTORS];

// 初始化函数
    void initializeMotors();

} // namespace TMC2209
