#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include <cmath>
#include "imu/I2Cdev.h"


/* =========================PWM Pin======================
 *  |  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |  |
 *  |0 |1 |2 |3 |4 |5 |6 |7 |8 |9 |10 |11 |12 |13 |14 |15|
 *  |  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |  |
 *  |  |  |  |  |  |  |  |  |  |  |   |   |v  |v  |v  |v |
 */

#define PCA9685_addr      0x40
#define PCA9685_SUBADR1   0x02
#define PCA9685_SUBADR2   0x03
#define PCA9685_SUBADR3   0x04

#define PCA9685_MODE1     0x00
#define PCA9685_PRESCALE  0xFE

// PWM_0
#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09
// PWM_1
#define LED1_ON_L         0x0A
#define LED1_ON_H         0x0B
#define LED1_OFF_L        0x0C
#define LED1_OFF_H        0x0D
// PWM_2
#define LED2_ON_L         0x0E
#define LED2_ON_H         0x0F
#define LED2_OFF_L        0x10
#define LED2_OFF_H        0x11
// PWM_3
#define LED3_ON_L         0x12
#define LED3_ON_H         0x13
#define LED3_OFF_L        0x14
#define LED3_OFF_H        0x15
// PWM_4
#define LED4_ON_L         0x16
#define LED4_ON_H         0x17
#define LED4_OFF_L        0x18
#define LED4_OFF_H        0x19
// PWM_5
#define LED5_ON_L         0x1A
#define LED5_ON_H         0x1B
#define LED5_OFF_L        0x1C
#define LED5_OFF_H        0x1D
// PWM_6
#define LED6_ON_L         0x1E
#define LED6_ON_H         0x1F
#define LED6_OFF_L        0x20
#define LED6_OFF_H        0x21
// PWM_7
#define LED7_ON_L         0x22
#define LED7_ON_H         0x23
#define LED7_OFF_L        0x24
#define LED7_OFF_H        0x25
// PWM_8
#define LED8_ON_L         0x26
#define LED8_ON_H         0x27
#define LED8_OFF_L        0x28
#define LED8_OFF_H        0x29
// PWM_9
#define LED9_ON_L         0x2A
#define LED9_ON_H         0x2B
#define LED9_OFF_L        0x2C
#define LED9_OFF_H        0x2D
// PWM_10
#define LED10_ON_L        0x2E
#define LED10_ON_H        0x2F
#define LED10_OFF_L       0x30
#define LED10_OFF_H       0x31
// PWM_11
#define LED11_ON_L        0x32
#define LED11_ON_H        0x33
#define LED11_OFF_L       0x34
#define LED11_OFF_H       0x35
// PWM_12
#define LED12_ON_L        0x36
#define LED12_ON_H        0x37
#define LED12_OFF_L       0x38
#define LED12_OFF_H       0x39
// PWM_13
#define LED13_ON_L        0x3A
#define LED13_ON_H        0x3B
#define LED13_OFF_L       0x3C
#define LED13_OFF_H       0x3D
// PWM_14
#define LED14_ON_L        0x3E
#define LED14_ON_H        0x3F
#define LED14_OFF_L       0x40
#define LED14_OFF_H       0x41
// PWM_15
#define LED15_ON_L        0x42
#define LED15_ON_H        0x43
#define LED15_OFF_L       0x44
#define LED15_OFF_H       0x45
// control all PWM at once
#define ALLLED_ON_L       0xFA
#define ALLLED_ON_H       0xFB
#define ALLLED_OFF_L      0xFC
#define ALLLED_OFF_H      0xFD



class PCA9685
{
public:
    PCA9685();
    void initialize();
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    //void setPin(uint8_t num, uint16_t val, bool invert=false);
    void getMode1();
    void setMode1(uint8_t num);
    void getPrescale();
    void setPrescale(uint8_t val);
    void setSleep(uint8_t mode);
    void restart();
    void getPWM(uint8_t num);
    void setMotor(uint8_t num, uint16_t val);
private:
    uint8_t addr;
};

#endif // PCA9685_H
