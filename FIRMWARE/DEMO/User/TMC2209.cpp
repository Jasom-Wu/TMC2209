#include "TMC2209.hpp"
#include "main.h"
#include "tim.h"
#include <cstring>
#include <cstdio>

namespace TMC2209 {

    Motor *motors[MAX_MOTORS] = {nullptr};
// 静态 UART 对象
    static BitBangUART::GPIO_BitBangUART moto_uart1(M1UART_GPIO_Port, M1UART_Pin, 115200);
    static BitBangUART::GPIO_BitBangUART moto_uart2(M2UART_GPIO_Port, M2UART_Pin, 115200);
    static BitBangUART::GPIO_BitBangUART moto_uart3(M3UART_GPIO_Port, M3UART_Pin, 115200);
    static uint8_t calculateCRC(const uint8_t *datagram, size_t length) {
      uint8_t crc = 0;
      for(size_t i=0;i<length;i++){
        uint8_t currentByte = datagram[i];
        for(int j=0;j<8;j++){
          if((crc>>7) ^ (currentByte & 0x01)){
            crc = (crc<<1) ^ 0x07;
          } else {
            crc <<= 1;
          }
          currentByte >>= 1;
        }
      }
      return crc;
    }

    void initializeMotors() {
      motors[0] = new Motor(&moto_uart1, &htim4, TIM_CHANNEL_3,
                            M1STEP_GPIO_Port, M1STEP_Pin,
                            M1DIR_GPIO_Port, M1DIR_Pin,
                            M1EN_GPIO_Port, M1EN_Pin, 1, 0x00);
      motors[1] = new Motor(&moto_uart2, &htim3, TIM_CHANNEL_1,
                            M2STEP_GPIO_Port, M2STEP_Pin,
                            M2DIR_GPIO_Port, M2DIR_Pin,
                            M2EN_GPIO_Port, M2EN_Pin, 2, 0x00);
      motors[2] = new Motor(&moto_uart3, &htim23, TIM_CHANNEL_2,
                            M3STEP_GPIO_Port, M3STEP_Pin,
                            M3DIR_GPIO_Port, M3DIR_Pin,
                            M3EN_GPIO_Port, M3EN_Pin, 3, 0x00);
      for (int i = 0; i < MAX_MOTORS; ++i) {
        motors[i]->setDirection(GPIO_PIN_SET);
        motors[i]->setSpeed(190000);
        motors[i]->configureGCONF();
        motors[i]->setMicrosteppingResolution(1);
        motors[i]->setSpreadCycle(false);
        motors[i]->setIRUN(31);
      }
    }

    Motor::Motor(BitBangUART::GPIO_BitBangUART* uart,
                 TIM_HandleTypeDef* timer,
                 uint32_t channel,
                 GPIO_TypeDef* step_port, uint16_t step_pin,
                 GPIO_TypeDef* dir_port, uint16_t dir_pin,
                 GPIO_TypeDef* enn_port, uint16_t enn_pin,
                 uint8_t id_, uint8_t address_)
            : uart(uart), htim(timer), step_channel(channel),
              step_port(step_port), step_pin(step_pin),
              dir_port(dir_port), dir_pin(dir_pin),
              enn_port(enn_port), enn_pin(enn_pin),
              id(id_), address(address_),
              stepsTaken(0), nextTotalSteps(0), isStepping(false)
    {}

// ==================== 电机控制 ====================
    void Motor::start() {
      enableDriver(GPIO_PIN_RESET);
      HAL_TIM_PWM_Start_IT(htim, step_channel);
      isStepping = true;
    }

    void Motor::stop() {
      HAL_TIM_PWM_Stop_IT(htim, step_channel);
      enableDriver(GPIO_PIN_SET);
      isStepping = false;
    }

    void Motor::step(uint32_t steps, GPIO_PinState dir) {
      setDirection(dir);
      start();
      countSteps(steps);
      stop();
    }
    void Motor::stepms(uint32_t Xms, GPIO_PinState dir) {
      setDirection(dir);
      start();
      TMCDelayms(Xms);
      stop();
    }
    void Motor::stepml(uint16_t Xml, GPIO_PinState dir) {
      uint32_t totalstep = Xml/ML_PER_STEP;
      step(totalstep);
    }

    void Motor::setDirection(GPIO_PinState state) {
      HAL_GPIO_WritePin(dir_port, dir_pin, state);
    }

    void Motor::enableDriver(GPIO_PinState state) {
      HAL_GPIO_WritePin(enn_port, enn_pin, state);
    }

    void Motor::setSpeed(uint32_t stepFrequency) {
      uint32_t prescaler = htim->Init.Prescaler;
      uint32_t timerClock = HAL_RCC_GetHCLKFreq() / (prescaler+1);
      uint32_t ARR = (timerClock / stepFrequency) - 1;
      __HAL_TIM_SET_AUTORELOAD(htim, ARR);
      __HAL_TIM_SET_COMPARE(htim, step_channel, ARR / 2);
    }

// ==================== 内部步骤计数 ====================
    void Motor::countSteps(uint32_t totalSteps) {
      nextTotalSteps = totalSteps;
      stepsTaken = 0;
      while(stepsTaken < nextTotalSteps);
      nextTotalSteps = 0;
    }

// ==================== UART 写寄存器 ====================
    void Motor::writeRegister(uint8_t regAddress, int32_t value) const {
      uint8_t command[8];
      command[0] = SYNC;
      command[1] = address;
      command[2] = regAddress | 0x80;
      command[3] = (value >> 24) & 0xFF;
      command[4] = (value >> 16) & 0xFF;
      command[5] = (value >> 8) & 0xFF;
      command[6] = (value) & 0xFF;
      command[7] = calculateCRC(command, 7);
      uart->sendBytes(command, 8);
      HAL_Delay(2);
    }

// ==================== 功能方法 ====================
    void Motor::setSpreadCycle(bool enable) {
      uint32_t gconf = 0x000000C0; // pdn_disable=1, mstep_reg_select=1
      if(enable) gconf |= (1<<TMC2209_EN_SPREADCYCLE_POS);
      else gconf &= ~(1<<TMC2209_EN_SPREADCYCLE_POS);
      writeRegister(TMC2209_REG_GCONF, gconf);
    }

    void Motor::setMicrosteppingResolution(uint16_t resolution) {
      uint32_t newMRES = 0x00;
      switch(resolution){
        case 256: newMRES=0x00; break;
        case 128: newMRES=0x01; break;
        case 64:  newMRES=0x02; break;
        case 32:  newMRES=0x03; break;
        case 16:  newMRES=0x04; break;
        case 8:   newMRES=0x05; break;
        case 4:   newMRES=0x06; break;
        case 2:   newMRES=0x07; break;
        case 1:   newMRES=0x08; break;
        default:  newMRES=0x00; break;
      }
      newMRES |= 1<<29;
      writeRegister(TMC2209_REG_CHOPCONF, newMRES);
    }

    void Motor::setIRUN(uint8_t irun_value) {
      if(irun_value>31) irun_value=31;
      uint32_t regVal = 0x00;
      regVal &= ~(0x1F << 8);
      regVal |= (irun_value << 8);
      writeRegister(TMC2209_REG_IHOLD_IRUN, regVal);
    }

    void Motor::configureGCONF() {
      uint32_t gconf = 0x000000C0;
      writeRegister(TMC2209_REG_GCONF, gconf);
      HAL_Delay(1);
    }

    void Motor::resetSteps() {
      stepsTaken = 0;
      nextTotalSteps = 0;
      isStepping = false;
    }

} // namespace TMC2209

// ==================== HAL Callback ====================
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  using namespace TMC2209;
  for(int i=0;i<MAX_MOTORS;i++){
    if(motors[i] && htim->Instance==motors[i]->htim->Instance){
      motors[i]->stepsTaken++;
    }
  }
}
