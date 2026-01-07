/*******************************************************************************
 * @file        MotorControl.c
 * @author      Timothy Felt
 * @coauthor    Gemini AI
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
 * - Developed in collaboration with Google Gemini AI.
 * * @hardware    RPI Pico W (RP2040)
 * @language    C
 * @libraries   FreeRTOS, PCA9685, CommsI2C, hardware/pwm, hardware/adc, hardware/i2c
 *******************************************************************************/