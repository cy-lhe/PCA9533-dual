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

#include "PCA9533.h"

//    CONSTRUCTOR
Pca9533::PCA9533::PCA9533(pca9533_chip_type_t chipType) {
  PCA9533_DEV_ADDR = chipType == PCA9533_CHIP_TYPE01 ? PCA9533_DEV_ADDR_CHIP01 : PCA9533_DEV_ADDR_CHIP02;
}

//    DESTRUCTOR
Pca9533::PCA9533::~PCA9533() {}

//    PING (0 = Success, other = Error code) details in ReadMe
byte Pca9533::PCA9533::ping() {
  Wire.beginTransmission(PCA9533_DEV_ADDR);
  return Wire.endTransmission();
}

bool Pca9533::PCA9533::init() {
  setReg(REG_PSC0, 0x00);
  setReg(REG_PSC1, 0x00);
  setReg(REG_PWM0, 0x00);
  setReg(REG_PWM1, 0x00);
  setReg(REG_LED, port_setting);
  return true;
}

//    SET REGISTER DATA
void Pca9533::PCA9533::setReg(pca9533_reg_ptr_t regPtr, byte newSetting) {
  if (regPtr > 0) {
    // log_d("Writing register: 0x%02x value: 0x%02x", regPtr, newSetting);
    initCall(regPtr);
    Wire.write(newSetting);
    endCall();
  }
}
//    READ REGISTER DATA
byte Pca9533::PCA9533::readReg(pca9533_reg_ptr_t regPtr) {
  byte ret = 0;
  Wire.beginTransmission(PCA9533_DEV_ADDR);
  Wire.write(regPtr);
  Wire.endTransmission();
  Wire.requestFrom(PCA9533_DEV_ADDR, (uint8_t)1);
  ret = Wire.read();
  // log_d("Read from register: 0x%02x read(1): 0x%02x", regPtr, ret);
  return ret;
}

//    INITIATE I2C COMMUNICATION
void Pca9533::PCA9533::initCall(pca9533_reg_ptr_t regPtr) {
  Wire.beginTransmission(PCA9533_DEV_ADDR);
  Wire.write(regPtr);
}

//    SET MODE - Requires pin and mode to set.
// 	  Parameters IOx & LED_MODE_x (See header file)
void Pca9533::PCA9533::setMode(pca9533_pin_t pin, pca9533_led_out_mode_t newMode) {
  byte bit_mask;
  bit_mask = (3 << pin);
  port_setting = (port_setting & (~bit_mask)) | (newMode << pin);
  setReg(REG_LED, port_setting);
}

//    SET MODE FOR ALL PINS :: Parameters ALL_ON | ALL_OFF
void Pca9533::PCA9533::setMode(pca9533_mode_t newMode) {
  setReg(REG_LED, newMode);
}

//    SET PWM Registers :: Parameters REG_PWM0 | REG_PWM1
void Pca9533::PCA9533::setPWM(pca9533_reg_ptr_t pwmPort, int pwmValue) {
  setReg(pwmPort, pwmValue);
}

//    SET PSC Registers :: Parameters REG_PSC0 | REG_PSC1
//       PCS0/1 (Frequency Prescaler 0/1) is used to program the period of the PWM output.
//       Blink Period in seconds = (pscValue + 1) / 152)
//       Max freq of 152Hz is set when pscValue is 0
void Pca9533::PCA9533::setPSC(pca9533_reg_ptr_t pscPort, int pscValue) {
  setReg(pscPort, pscValue);
}


bool Pca9533::PCA9533::digitalRead(pca9533_pin_t pin) {
  byte inputs = readReg(REG_INPUT);
  bool ret = false;
  switch (pin) {
  case IO0:
    ret = inputs & 0x01;
  case IO1:
    ret = inputs & 0x02;
  case IO2:
    ret = inputs & 0x04;
  case IO3:
    ret = inputs & 0x08;
  }
  // log_d("Read pin: %d as: %d", pin == IO0 ? 0 : pin == IO1 ? 1 : pin == IO2 ? 2 : pin == IO3 ? 3 : -1, ret);
  return ret;

}

//    STOP I2C COMMUNICATION
void Pca9533::PCA9533::endCall() {
  _comBuffer = Wire.endTransmission();
}
