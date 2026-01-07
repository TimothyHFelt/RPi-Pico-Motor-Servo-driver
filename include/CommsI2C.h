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