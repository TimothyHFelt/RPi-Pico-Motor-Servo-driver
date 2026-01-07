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

void setUpI2C(){
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_set_function(16, GPIO_FUNC_I2C);
    i2c_init(i2c0, 400000);

}

bool TestConnection(){
    int returnNum = i2c_write_blocking(i2c0, I2C_ADDR, NULL, 0, false);
    if (returnNum == PICO_ERROR_GENERIC){
        return false;
    }
    return true;
    
}

int WriteToReg(uint8_t regAddress, uint8_t data){
    uint8_t buff[2];
    buff[0] = regAddress;
    buff[1] = data;
    int written = i2c_write_blocking(i2c0, I2C_ADDR, buff, 2, false);
    return written;
}

int ReadReg(uint8_t regAddress, uint8_t *returnData){
    uint8_t writtenData = i2c_write_blocking(i2c0, I2C_ADDR, &regAddress, 1, true);
    if (writtenData == -1){
        return -2;
    }
    uint8_t myNum = i2c_read_blocking(i2c0, I2C_ADDR, returnData, 1, false);
    return myNum;
}

//Following Functions uploaded to github for the PCA9865 by Lee Jackson. Modified for RPi Pico Microcontroller.
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

void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  WriteToReg(LED0_ON_L+4*num, on);
  WriteToReg(LED0_ON_L+4*num + 1, (on>>8));
  WriteToReg(LED0_ON_L+4*num + 2, off);
  WriteToReg(LED0_ON_L+4*num + 3, off>>8);
}

void setServoPulse(uint8_t n, double pulse)
{
   double pulselength;
   pulselength = 1000000;   // 1,000 ms per second 
   pulselength /= 60;    // 60 Hz
   pulselength /= 4096;         //ms
   double pwmVal = pulse/pulselength;
   PCA9685_setPWM(n, 0, (uint16_t)pwmVal);
}

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



