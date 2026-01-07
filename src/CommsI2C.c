#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "CommsI2C.h"
#include "hardware/gpio.h"
#include "PCA9685.h"
#include "hardware/i2c.h"

static int fd = 0;
uint8_t ServoUpDegree = 90;    //Servo Up Angle
uint8_t ServoDownDegree =90;   //Servo Down Angle


/**
 * @brief Initializes the I2C0 hardware interface for the RP2040.
 * Sets up GPIO 16 (SDA) and 17 (SCL) and configures the I2C clock 
 * speed to 400kHz (Fast Mode).
 */
void setUpI2C(){
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_set_function(16, GPIO_FUNC_I2C);
    i2c_init(i2c0, 400000);

}

/**
 * @brief Validates the connection to the PCA9685 hardware address.
 * Attempts a zero-byte write to the I2C address to check for an ACK.
 * @return true if device is detected, false otherwise.
 */
bool TestConnection(){
    int returnNum = i2c_write_blocking(i2c0, I2C_ADDR, NULL, 0, false);
    if (returnNum == PICO_ERROR_GENERIC){
        return false;
    }
    return true;
    
}

/**
 * @brief Writes bytes of data to a specific PCA9685 register.
 * @param regAddress The target hardware register.
 * @param data The 8-bit value to write.
 * @return Number of bytes written, or PICO_ERROR_GENERIC on failure.
 */
int WriteToReg(uint8_t regAddress, uint8_t data){
    uint8_t buff[2];
    buff[0] = regAddress;
    buff[1] = data;
    int written = i2c_write_blocking(i2c0, I2C_ADDR, buff, 2, false);
    return written;
}

/**
 * @brief Reads a single byte of data from a specific PCA9685 register.
 * Uses a repeated start condition to transition from write to read.
 * @param regAddress The register to read from.
 * @param returnData Pointer to store the resulting 8-bit value.
 * @return Number of bytes read, or error code.
 */
int ReadReg(uint8_t regAddress, uint8_t *returnData){
    uint8_t writtenData = i2c_write_blocking(i2c0, I2C_ADDR, &regAddress, 1, true);
    if (writtenData == -1){
        return -2;
    }
    uint8_t myNum = i2c_read_blocking(i2c0, I2C_ADDR, returnData, 1, false);
    return myNum;
}

/**
 * @brief Sets the PWM frequency for the PCA9685.
 * Includes frequency correction for the internal oscillator (0.8449) 
 * and handles the Mode1 register sleep/wake cycle required for prescale changes.
 * @param freq Target frequency in Hz (typically 50-60Hz for servos).
 */
void PCA9685_setPWMFreq(float freq)
{
  freq *= 0.8449;                     // Correct the overshoot of the frequency setting
  float prescaleval = 25000000;       // 25MHz
  prescaleval /= 4096;                // 4096 PWM cycles
  prescaleval /= freq;                // Calculate the frequency division value
  prescaleval -= 1;
  float prescale = (float)(prescaleval + 0.5);
  uint8_t oldmode;
  ReadReg(PCA9685_MODE1, &oldmode);
  uint8_t newmode = (oldmode&0x7F) | 0x10; 
  WriteToReg(PCA9685_MODE1, newmode);              // sleep_ms
  WriteToReg(PCA9685_PRESCALE, (uint8_t)prescale); // Set predivider
  WriteToReg(PCA9685_MODE1, oldmode & 0xEF);
  sleep_us(500);
  ReadReg(PCA9685_MODE1, &oldmode);
  WriteToReg(PCA9685_MODE1, oldmode | 0xa0);       // Set the MODE1 register to turn on autoincrement
}

/**
 * @brief Sets the ON and OFF timing for a specific PWM channel.
 * Directly writes to the 4 registers (ON_L, ON_H, OFF_L, OFF_H) required 
 * to define the PWM duty cycle for a single channel.
 * @param num PCA9685 channel index (0-15).
 * @param on 12-bit timing value for the start of the pulse.
 * @param off 12-bit timing value for the end of the pulse.
 */
void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  WriteToReg(LED0_ON_L+4*num, on);
  WriteToReg(LED0_ON_L+4*num + 1, (on>>8));
  WriteToReg(LED0_ON_L+4*num + 2, off);
  WriteToReg(LED0_ON_L+4*num + 3, off>>8);
}

/**
 * @brief Translates a microsecond pulse width into a 12-bit PWM value.
 * Calculates the pulse length based on a 60Hz frequency and 4096-step 
 * resolution before updating the PCA9685 hardware.
 * @param n PCA9685 channel index.
 * @param pulse Pulse width in microseconds (typically 500us to 2500us).
 */
void setServoPulse(uint8_t n, double pulse)
{
   double pulselength;
   pulselength = 1000000;   // 1,000 ms per second 
   pulselength /= 60;    // 60 Hz
   pulselength /= 4096;         //ms
   double pwmVal = pulse/pulselength;
   PCA9685_setPWM(n, 0, (uint16_t)pwmVal);
}

/**
 * @brief Maps a 0-180 degree angle to a specific PWM pulse width.
 * Performs linear interpolation between MIN_PULSE_US and MAX_PULSE_US.
 * @param n PCA9685 channel index.
 * @param Degree Targeted angle (clamped 0-180).
 */
void setServoDegree(uint8_t n, uint8_t Degree)
{
    if(Degree >= 180)
    {
      Degree = 180;
      
    }
    else if(Degree <= 0)
    {
      Degree = 0;
    }
    double pulse = MIN_PULSE_US + ((MAX_PULSE_US - MIN_PULSE_US) / 180.0) * (double)Degree;
    setServoPulse(n, pulse);
}

/**
 * @brief Increments the current servo angle by a specified step.
 * Checks against SERVO_MAX constants to prevent over-travel and updates 
 * the persistent global tracking variables (ServoUpDegree/ServoDownDegree).
 * @param Channel Target servo channel (UP or DOWN).
 * @param Step Number of degrees to add to current position.
 * @return 0 on success, 1 if channel is invalid.
 */
int ServoDegreeIncrease(uint8_t Channel, uint8_t Step){
    switch(Channel){
        case SERVO_UP_CH:
            if(ServoUpDegree >= SERVO_UP_MAX){
                ServoUpDegree = SERVO_UP_MAX;
                setServoDegree(Channel, ServoUpDegree);
            }
            else{
                ServoUpDegree += Step;
                setServoDegree(Channel, ServoUpDegree); 
            }
            break;
        case SERVO_DOWN_CH:
            if(ServoDownDegree >= SERVO_DOWN_MAX){
                ServoDownDegree = SERVO_DOWN_MAX;
                setServoDegree(Channel, ServoDownDegree); 
            }
            else{
                ServoDownDegree += Step;
                setServoDegree(Channel, ServoDownDegree); 
            }
            break;
        default:
            return 1;
            break;
    }
    return 0;
}


/**
 * @brief Decrements the current servo angle by a specified step.
 * Checks against SERVO_MIN constants to prevent under-travel and 
 * updates global tracking variables.
 * @param Channel Target servo channel (UP or DOWN).
 * @param Step Number of degrees to subtract from current position.
 * @return 0 on success, 1 if channel is invalid.
 */
int ServoDegreeDecrease(uint8_t Channel, uint8_t Step){
    switch(Channel){
        case SERVO_UP_CH:
            if(ServoUpDegree <= SERVO_UP_MIN + Step){
                ServoUpDegree = SERVO_UP_MIN;
                setServoDegree(Channel, ServoUpDegree);
            }
            else{
                ServoUpDegree -= Step;
                setServoDegree(Channel, ServoUpDegree); 
            }
            break;
        case SERVO_DOWN_CH:
            if(ServoDownDegree <= SERVO_DOWN_MIN + Step){
                ServoDownDegree = SERVO_DOWN_MIN;
                setServoDegree(Channel, ServoDownDegree); 
            }
            else{
                ServoDownDegree -= Step;
                setServoDegree(Channel, ServoDownDegree); 
            }
            break;
        default:
            return 1;
            break;
    }
    return 0;
}



