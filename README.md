/*******************************************************************************
 * @file        MotorControl.c
 * @author      Timothy Felt
 * @project     RPI Surveillance Tank
 * * @description 
 * This code is used to control 2 servos and motors for an RPI surveillance tank. 
 * It reads UART data and populates a Queue to drive the Servos and Motors. 
 * Includes a Voltage Monitor task to:
 * 1. Monitor system voltage via ADC.
 * 2. Adjust motor duty cycle for steady power (Voltage Compensation).
 * 3. Execute emergency shutdown if low voltage is sensed.
 * * @credits
 * - PCA9685.h: Pulled from Adafruit-PWM-Servo-Driver-Library.
 * - CommsI2C.h: Pulled from Adafruit; modified for RPI Pico W (RP2040).
   - **Software License Agreement (BSD License)

    Copyright (c) 2012, Adafruit Industries
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.**
 * * @hardware    RPI Pico W (RP2040)
 * @language    C
 * @libraries   FreeRTOS, PCA9685, CommsI2C, hardware/pwm, hardware/adc, hardware/i2c
 *******************************************************************************/
