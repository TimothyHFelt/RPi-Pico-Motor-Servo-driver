#define I2C_ADDR           0x40  //IIC address 

#define MODE1              0x00
#define MODE2              0x01
#define SUBADR1            0x02
#define SUBADR2            0x03

#define MIN_PULSE_US       500.0  
#define MAX_PULSE_US       2500.0

#define PCA9685_SUBADR1    0x2
#define PCA9685_SUBADR2    0x3
#define PCA9685_SUBADR3    0x4

#define PCA9685_MODE1      0x0
#define PCA9685_PRESCALE   0xFE

#define LED0_ON_L          0x6
#define LED0_ON_H          0x7
#define LED0_OFF_L         0x8
#define LED0_OFF_H         0x9

#define ALLLED_ON_L        0xFA
#define ALLLED_ON_H        0xFB
#define ALLLED_OFF_L       0xFC
#define ALLLED_OFF_H       0xFD

#define SERVO_UP_MAX       180    
#define SERVO_UP_MIN       0     
#define SERVO_DOWN_MAX     200 
#define SERVO_DOWN_MIN     20      
#define STEP               1             
#define STEP_DELAY         2        
#define SERVO_UP_CH        0       
#define SERVO_DOWN_CH      1     

#define I2C_SLAVE_FORCE    0x0706

#define SERVO_UP_DEGREE 0
#define SERVO_DOWN_DEGREE 0