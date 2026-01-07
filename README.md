This code was made in collaboration with Google Gemini AI.
The PCA9685.h file was pulled from Adafruit-PWM-Servo-Driver-Library github
The CommsI2C file was also pulled from the library and modified to work on the RPI Pico W board 
Author: Timothy Felt
CoAuthors: Gemini AI 

Description: This code is used to control 2 servos and motors for a RPI surveillance tank. It reads a UART and puts that into a Queue used to tell the Servos and Motors what to do. The Voltage Monitor task is used to monitor the system voltage, adjust the motor duty cycle to create steady power as the voltage decreases, and to shut down the code if the voltage is sensed as low.

Language: C
Hardware: RPI Pico W RP2040 chip
key libraries: FreeRTOSConfig.h, PCA9685.h, CommsI2C.h, pico/stdlib.h, hardware/pwm.h, hardware/adc.h, hardware/i2c.h, pthread.h, 
Main Src: src/MotorControl.c