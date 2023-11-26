/*
  PCA9533 - Library for esp8266
  Copyright (c) 2017 Max Schmid. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if 1
__asm volatile ("nop");
#endif

#ifndef PCA9533_h
#define PCA9533_h
//#endif

//#if !defined(ARDUINO_ARCH_AVR)
//#error “The PCA9533 library only supports AVR processors.”
//#endif

#include <Arduino.h>
#include "Wire.h"

namespace Pca9533 {
    const byte PCA9533_DEV_ADDR_CHIP01 = 0x62;
    const byte PCA9533_DEV_ADDR_CHIP02 = 0x63;
    const byte COM_SUCCESS = 0x00;


    typedef enum :byte {
        REG_INPUT = 0x00,     // Default :: INPUT REGISTER
        REG_PSC0 = 0x11,     // FREQUENCY PRESCALER 0
        REG_PWM0 = 0x12,     // PWM REGISTER 0
        REG_PSC1 = 0x13,     // FREQUENCY PRESCALER 1
        REG_PWM1 = 0x14,     // PWM REGISTER 1
        REG_LED = 0x15      // LED SELECTOR
    } pca9533_reg_ptr_t;

    typedef enum :byte {
        PIN_HIGH = 0,  // 00 - Output is set Hi-Z (LED off – default)
        PIN_LOW = 1,  // 01 - Output is set LOW (LED on)
        LED_MODE_OFF = 0,  // 00 - Output is set Hi-Z (LED off – default)
        LED_MODE_ON = 1,  // 01 - Output is set LOW (LED on)
        LED_MODE_PWM0 = 2,  // 10 - Output blinks at PWM0 rate
        LED_MODE_PWM1 = 3   // 11 - Output blinks at PWM1 rate
    } pca9533_led_out_mode_t;

    typedef enum :byte {
        IO0 = 0,
        IO1 = 2,
        IO2 = 4,
        IO3 = 6
    } pca9533_pin_t;

    typedef enum :byte {
        ALL_ON = 0x55,
        ALL_OFF = 0x00
    } pca9533_mode_t;

    typedef enum :byte {
        IO_LOW = 0,
        IO_HIGH = 1
    } pca9533_state_t;

    typedef enum :byte {
        IO_PWM0 = 0x12,
        IO_PWM1 = 0x14
    } pca9533_pwm_t;

    typedef enum :byte {
        PCA9533_CHIP_TYPE01 = 1,
        PCA9533_CHIP_TYPE02 = 2
    } pca9533_chip_type_t;


    class PCA9533 {
    public:
        PCA9533(pca9533_chip_type_t chipType = PCA9533_CHIP_TYPE01);
        ~PCA9533();
        byte port_setting = B00001110;
        byte ping();
        bool init();
        void setMode(pca9533_pin_t pin, pca9533_led_out_mode_t newMode);
        void setMode(pca9533_mode_t newMode);
        void setPWM(pca9533_reg_ptr_t pwmPort, int pwmValue);
        void setPSC(pca9533_reg_ptr_t pscPort, int pscValue);
        bool digitalRead(pca9533_pin_t pin);

    private:
        byte PCA9533_DEV_ADDR; // can be selected in Constructor
        byte _comBuffer;
        void setReg(pca9533_reg_ptr_t ptr, byte newSetting);
        byte readReg(pca9533_reg_ptr_t regPtr);
        void initCall(pca9533_reg_ptr_t regPtr);
        void endCall();
    };
}

// using namespace Pca9533;

#endif
