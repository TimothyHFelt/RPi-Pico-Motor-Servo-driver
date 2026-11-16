/**@credits
PCA9685.h: Pulled from Adafruit-PWM-Servo-Driver-Library.
CommsI2C.h: Pulled from Adafruit; modified for RPI Pico W (RP2040).
**Software License Agreement (BSD License)
Copyright (c) 2012, Adafruit Industries All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.**
*/

#ifndef MY_I2C_DRIVER_H
#define MY_I2C_DRIVER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "PCA9685.h"
#include <stdbool.h>
#include "hardware/i2c.h"

extern uint8_t ServoUpDegree;    
extern uint8_t ServoDownDegree; 

/**
 * @brief Initializes the I2C peripheral on the Pico and sets up the pins.
 */
void setUpI2C();


/**
 * @brief Checks for a response from the I2C device.
 * * @return true if communication is successful.
 * @return false if communication failed (device not connected/addressed).
 */
bool TestConnection(); 


/**
 * @brief Sends an I2C command to a determined register
 * * @param regAddress the address of the register
 * @param data The data to send to the motor
 * @return int The number of bytes written, or a negative error code on failure.
 */
int WriteToReg(uint8_t regAddress, uint8_t data);


/**
* @brief reads a register and stores the data in the returnData memory location
* * @param regAddress the address of the register 
* @param returnData the address the data will be stored in.
* @return the number of bytes that were read. A -1 if there was a read error or a -2 if there was a problem finding the register.
*/
int ReadReg(uint8_t regAddress, uint8_t *returnData);


/**
 * @brief this function was written by Lee Jackson and uploaded to github for the PCA8965 servo driver board. 
 * This function sets the board up with the desired frequency.
 * @param frequency the desired frequency you want the servos to run at.
 */
void PCA9685_setPWMFreq(float freq);



void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off);
void setServoPulse(uint8_t n, double pulse);
void setServoDegree(uint8_t n, uint8_t Degree);
int ServoDegreeIncrease(uint8_t Channel, uint8_t Step);
int ServoDegreeDecrease(uint8_t Channel, uint8_t Step);

#endif // MY_I2C_DRIVER_H
