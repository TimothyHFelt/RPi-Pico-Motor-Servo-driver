#include "FreeRTOS.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "task.h"
#include "CommsI2C.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "PCA9685.h"
#include "hardware/regs/pwm.h"
#include "hardware/structs/pwm.h"
#include "pico/util/datetime.h"
#include "queue.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include <math.h>
#include "semphr.h"


#define DCMOTOR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define UART_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define VOLTAGE_TASK_PRIORITY (tskIDLE_PRIORITY + 3UL)
#define VOLTAGE_TASK_STACK_SIZE 2048
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define UART_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define DCMOTOR_TASK_STACK_SIZE 2048
#define SERVO1_DIRECTION 0
#define SERVO2_DIRECTION 2
#define MOTOR1_DIRECTION 4
#define MOTOR2_DIRECTION 6
#define MOTOR1_SPEED 5
#define MOTOR2_SPEED 7
#define UART_TIMEOUT_MS 2000

absolute_time_t last_packet_time;
uint8_t myRead = 0;
uint8_t *regContents = &myRead;
QueueHandle_t xMotorQueue;
volatile uint8_t myMotors[8] = {1,1,1,1,1,1,1,1}; //[Servo1 Direction, Servo1 speed, Servo2 Direction, Servo2 Speed, Motor1 Direction, Motor1 Speed, Motor2 Direction, Motor 2 Speed]
volatile double systemVoltage = 1.0;
volatile bool isSafe = true;
SemaphoreHandle_t xSafeMutex;

void StopAllMotors();
void motor_safety_check();
void setSafety(bool safe);
bool getSystemSafety();
void ServoDirectionHandler(int motorSelection);


/**
 * @brief Immediate emergency stop for all propulsion and gimbal actuators.
 * * This function performs two critical actions:
 * 1. Resets the software direction flags in the myMotors array.
 * 2. Directly clears the Counter Compare (CC) registers for PWM Slices 7 and 2 
 * on the RP2040 hardware, ensuring all power output to motors and servos 
 * is cut regardless of the current task state.
 * * @note This is called by the Voltage Monitor task if a low-voltage 
 * condition is detected.
 */
void StopAllMotors(){
    myMotors[MOTOR1_DIRECTION] = 1;
    myMotors[MOTOR2_DIRECTION] = 1;
    myMotors[SERVO1_DIRECTION] = 1;
    myMotors[SERVO2_DIRECTION] = 1;

    pwm_hw->slice[7].cc = 0;
    pwm_hw->slice[2].cc = 0;
}

/**
 * @brief Thread-safe update of the global system safety status.
 * * This function attempts to take the safety mutex to update the 'isSafe' flag.
 * It is primarily used by the Voltage Monitor to lock out motor controls 
 * if battery levels are critical, or by the UART task to resume operation.
 * * @param safe Boolean value: 'true' for operational, 'false' for safety-halt.
 * * @note If the mutex cannot be acquired within 5 ticks (5ms), the update 
 * is skipped to maintain RTOS task timing.
 */
void setSafety(bool safe){
    if (xSemaphoreTake(xSafeMutex, pdMS_TO_TICKS(5))==true){
        isSafe = safe;
        xSemaphoreGive(xSafeMutex);
    }
}

/**
 * @brief Thread-safe retrieval of the system safety state.
 * * Accesses the global 'isSafe' variable using a mutex. This ensures that the 
 * Motor Task does not read a stale or transitioning state while the Voltage 
 * Monitor is updating health status.
 * * @return true  System is healthy; operations allowed.
 * @return false System is in a safety-halt state OR the mutex timed out.
 * * @note Defaulting to 'false' on mutex timeout is a fail-safe design 
 * to prevent unintended movement during high CPU contention.
 */
bool getSystemSafety(){
    bool status;
    if (xSemaphoreTake(xSafeMutex, pdMS_TO_TICKS(5))==true){
        status = isSafe;
        xSemaphoreGive(xSafeMutex);
        return status;
    }
    return false;
}

/**
 * @brief Interprets directional flags to update servo positions.
 * * This handler acts as the execution layer for servo commands stored in the 
 * myMotors array. It determines the target hardware channel and invokes 
 * the appropriate incremental movement function (Increase/Decrease) 
 * based on the direction flag.
 * * @param motorSelection The index in myMotors representing the target servo 
 * (Expected: SERVO1_DIRECTION or SERVO2_DIRECTION).
 * * @note Speed is pulled from myMotors[motorSelection + 1], treating the 
 * array as paired (Direction, Speed) values.
 */
void ServoDirectionHandler(int motorSelection){
    int direction = myMotors[motorSelection];
    int speed = myMotors[motorSelection+1];
    int channel = 0;
    if (motorSelection == SERVO1_DIRECTION){
        channel = SERVO_UP_CH;
    }
    else if (motorSelection == SERVO2_DIRECTION){
        channel = SERVO_DOWN_CH;
    }
    if (direction == 2)
    {
        ServoDegreeIncrease(channel, speed);
        //printf("Increasing servo angle.");
    }
    else if (direction == 0)
    {
        ServoDegreeDecrease(channel, speed);
        //printf("Decreasing Servo angle.");
    }
}

/**
 * @brief FreeRTOS Task: High-level management of motors, servos, and power scaling.
 * * This task runs indefinitely to perform the following:
 * 1. Hardware Setup: Configures RP2040 PWM Slices 2 and 7 and initializes PCA9685.
 * 2. Voltage Compensation: Adjusts the 'motorValue' PWM threshold based on current 
 * systemVoltage to provide consistent physical performance.
 * 3. Queue Processing: Drains xMotorQueue to update the myMotors state array.
 * 4. Actuation: Writes to hardware PWM registers for DC motors and invokes 
 * servo handlers for the gimbal.
 * 5. Safety: Integrates with system safety flags and refreshes the hardware watchdog.
 * * @param params Unused FreeRTOS task parameters.
 */
void MotorControl(__unused void *params){
    uint8_t myQueue[8];
    //setup PWM pins on GPIO
    gpio_set_function(21, GPIO_FUNC_PWM); //right motor slice 2 
    gpio_set_function(20, GPIO_FUNC_PWM);
    gpio_set_function(14, GPIO_FUNC_PWM); //left motor slice 7
    gpio_set_function(15, GPIO_FUNC_PWM);
    pwm_hw->slice[7].top=1561;
    pwm_hw->slice[7].div=0b01000000;
    pwm_hw->slice[7].csr=1;
    pwm_hw->slice[2].top=1561;
    pwm_hw->slice[2].div=0b01000000;
    pwm_hw->slice[2].csr=1;

    bool myBool = TestConnection();
    assert(myBool == true);
    int myReturn = WriteToReg(MODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(500));
    PCA9685_setPWMFreq(60);
    setServoDegree(SERVO_UP_CH, 90);
    setServoDegree(SERVO_DOWN_CH, 90);

    while(1){
        if(getSystemSafety()){
            int motorValue = (13*1561)/systemVoltage;
            if (motorValue > 1561){
            motorValue = 1561;
            }
            while (xQueueReceive(xMotorQueue, myQueue, 0) == pdTRUE){
                for(int i = 0; i < 8; i++) {
                    myMotors[i] = myQueue[i];
                }
            }
            if (myMotors[MOTOR1_DIRECTION] == 2){
                pwm_hw->slice[7].cc = 0 | motorValue << 16;

            }
            else if (myMotors[MOTOR1_DIRECTION] == 0){
                pwm_hw->slice[7].cc = motorValue | 0 << 16;
            }
            else if (myMotors[MOTOR1_DIRECTION] ==1){
                pwm_hw->slice[7].cc = 0;
            }
            if (myMotors[MOTOR2_DIRECTION] == 2){
                pwm_hw->slice[2].cc = 0 | motorValue << 16;
            }
            else if (myMotors[MOTOR2_DIRECTION] == 0){
                pwm_hw->slice[2].cc = motorValue | 0 << 16;
            }
            else if (myMotors[MOTOR2_DIRECTION] == 1){
                pwm_hw->slice[2].cc = 0;
            }
            else {
                pwm_hw->slice[7].cc = 0; // Emergency stop if data is weird
                }
            ServoDirectionHandler(SERVO1_DIRECTION);
            ServoDirectionHandler(SERVO2_DIRECTION);
        }
        else{
            pwm_hw->slice[2].cc = 0;
            pwm_hw->slice[7].cc = 0;
        }
        motor_safety_check();
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}


/**
 * @brief Heartbeat monitor for incoming control signals.
 * * Checks the elapsed time since the last valid UART command was received.
 * If the duration exceeds the defined safety window (UART_TIMEOUT_MS), 
 * the hardware is placed into a safe state to prevent unintended movement.
 * * @note This function uses the RP2040's 64-bit microsecond timer for 
 * high-precision delta calculation.
 */
void motor_safety_check() {
    // Calculate how long it has been since the last packet
    int64_t diff = absolute_time_diff_us(last_packet_time, get_absolute_time());
    
    // If more than 500,000 microseconds (500ms) have passed
    if (diff > (UART_TIMEOUT_MS * 1000)) {
        // EMERGENCY STOP: Turn off all motors
        StopAllMotors(); 
    }
}

/**
 * @brief FreeRTOS Task: High-speed UART parser and packet synchronizer.
 * * This task monitors the UART1 hardware buffer for a specific protocol:
 * 1. Synchronizes on a Start-of-Frame byte (0xFF).
 * 2. Reads a fixed 8-byte payload representing motor directions and speeds.
 * 3. Offloads the payload to xMotorQueue for processing by the Motor task.
 * 4. Resets the communication watchdog timer (last_packet_time).
 * * @note Operates with a 5ms polling interval to balance responsiveness 
 * with CPU efficiency.
 * @param params Unused FreeRTOS task parameters.
 */
void UartMessage(__unused void *params)
{
    uint8_t myQueue[8];
    last_packet_time = get_absolute_time();
    while(true)
    {
        if(getSystemSafety()){
            if(uart_is_readable(uart1))
            {
                //printf("readable");
                uint8_t raw_byte = uart_getc(uart1);
                // Print the byte in Hex so you can see exactly what arrived
                //printf("Received Byte: 0x%02X\n", raw_byte);
                if(raw_byte == 0xFF)
                {
                    uart_read_blocking(uart1, (uint8_t *)myQueue, 8);
                    //printf("UART RAW: %d %d %d %d\n", myQueue[0], myQueue[1], myQueue[2], myQueue[3]);
                    xQueueSend(xMotorQueue, myQueue, 0);
                    last_packet_time = get_absolute_time();
                }
                else{

                }
            }
        }
    vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief FreeRTOS Task: Monitors battery health and manages safety states.
 * * This task executes every 100ms to perform the following:
 * 1. ADC Sampling: Reads 5 samples to provide a stable averaged voltage.
 * 2. Voltage Conversion: Scales the raw 12-bit ADC value to real-world Voltage.
 * 3. Health Logic:
 * - Updates global systemVoltage for motor power compensation.
 * - Disables movement if voltage falls below 12.2V (Safety Cutoff).
 * - Enables movement if voltage is above 13.0V (Healthy State).
 * * @note Uses a 6.05x multiplier, assuming a hardware voltage divider circuit 
 * is connected to the ADC pin.
 */
void voltCheck(__unused void *params)
{
    while(1)
    {
        double sum = 0;
        for(int i = 0; i < 5; i ++){
            double myADC = adc_read();
            double myVolt = (myADC/4095)*3.3;
            sum += myVolt;
        }
        systemVoltage = (sum/5)*6.05;
        if(systemVoltage < 1){
        systemVoltage = 1;
    }
    if(systemVoltage < 13){
        //warning low battery to osd;
    }
    if(systemVoltage < 12.2){
        setSafety(false);
        StopAllMotors();
        //warning: Voltage regulator failed. Low voltage is detected. Stopping all motion.
    } else if(systemVoltage > 13){
        setSafety(true);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/// @brief 
/// @return 
int main()
{
    timer_hw->dbgpause = 0;
    char i2c_dev[32];
    stdio_init_all();
    setUpI2C();
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    adc_read();
    watchdog_enable(2000, 1);
    double myADC = adc_read();
    systemVoltage = (myADC / 4095.0) * 3.3 * 6.05;
    printf("System Voltage at boot: %.2fV\n", systemVoltage);
    uart_init(uart1, 115200);
    sleep_ms(10);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    xSafeMutex = xSemaphoreCreateMutex();
    TaskHandle_t xUartHandle = NULL;
    TaskHandle_t xMotorHandle = NULL;
    TaskHandle_t xVoltHandle = NULL;
    xMotorQueue = xQueueCreate(5, 8 * sizeof(uint8_t));
    if (xMotorQueue == NULL) {
    // Handle error: not enough RAM to create the mailbox
    //printf("Critical Error: Queue could not be created!\n");
    }
    xTaskCreate(UartMessage, "Uart Handler", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, &xUartHandle);
    xTaskCreate(MotorControl, "Motor Handler", DCMOTOR_TASK_STACK_SIZE, NULL, DCMOTOR_TASK_PRIORITY, &xMotorHandle);
    xTaskCreate(voltCheck, "Voltage Handler", VOLTAGE_TASK_STACK_SIZE, NULL, VOLTAGE_TASK_PRIORITY, &xVoltHandle);
    vTaskStartScheduler();
    

}
