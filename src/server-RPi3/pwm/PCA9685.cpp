#include "PCA9685.h"


int dec2bin(uint8_t data){
    int tmp=data;
    //std::cout<<tmp<<std::endl;
    int b[8]={0, 2, 4, 8, 16, 32, 64, 128};
    int pos=0;
    int inc = 0;
    int output[8]={0,0,0,0,0,0,0,0};
    int bin_v;


    for(int i=7; i>=0; i--){
        //std::cout<<b[i]<<std::endl;
        if(tmp>=b[i]){
            pos=i;
            //std::cout<<"pos: "<<pos<<std::endl;
            break;
        }
    }
    for(;pos>=0;pos--){
        //std::cout<<pos<<std::endl;
        if(b[pos] > tmp){
            continue;
        }
        if(pos==0){
            output[7-pos] = tmp%2;
        }else{
            output[7-pos] = tmp/b[pos];
        }
        tmp-=b[pos];
        //std::cout<<output[7-pos]<<std::endl;
    }

    bin_v = (output[0]*pow(10,7))+(output[1]*pow(10,6))+(output[2]*pow(10,5))+(output[3]*pow(10,4))
            +(output[4]*pow(10,3))+(output[5]*pow(10,2))+(output[6]*10)+output[7];

    return bin_v;
}

PCA9685::PCA9685(){
    addr=PCA9685_addr;
}

void PCA9685::initialize()
{
    setSleep(1);
    //usleep(500);
    setPrescale(30);
    //usleep(500);
    setSleep(0);
    usleep(500);
    //getPrescale();
    uint8_t* b0;
    //I2Cdev::readBit(addr, PCA9685_MODE1, 0, b0);
    //std::cout<<"ALLCALL: "<<b0<<std::endl;
    std::cout<<"PCA9685 initialization successful"<<std::endl;
}

void PCA9685::getMode1()
{
    uint8_t* data;
    I2Cdev::readByte(addr, PCA9685_MODE1, data);
    std::cout<<"Mode1: "<<dec2bin(*data)<<std::endl;
}

void PCA9685::setMode1(uint8_t num)
{
    I2Cdev::writeByte(addr, PCA9685_MODE1, num);
}

void PCA9685::getPrescale()
{
    uint8_t *data;
    I2Cdev::readByte(addr, PCA9685_PRESCALE, data);
    std::cout<<int(*data)<<std::endl;
    int b=dec2bin(*data);
    std::cout<<"Prescale: "<<b<<std::endl;

}

void PCA9685::setPrescale(uint8_t val)
{
    I2Cdev::writeByte(addr, PCA9685_PRESCALE, val);
}

void PCA9685::setSleep(uint8_t mode)
{
    I2Cdev::writeBits(addr, PCA9685_MODE1, 4, 1, mode);
}

void PCA9685::restart()
{
    uint8_t* b7;
    I2Cdev::readBits(addr, PCA9685_MODE1, 7, 1, b7); // read bit 7
    if(*b7==1){
        setSleep(0); //clear sleep bit
        usleep(500);
    }
    I2Cdev::writeBits(addr, PCA9685_MODE1, 7, 1, 1);
}

void PCA9685::setMotor(uint8_t num, uint16_t val)
{
    setPWM(num, 0, val);
}

void PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    I2Cdev::writeByte(addr, LED0_ON_L+(num*4), on);     // on_ low byte
    I2Cdev::writeByte(addr, LED0_ON_H+(num*4), on>>8);  // on_ high byte
    I2Cdev::writeByte(addr, LED0_OFF_L+(num*4), off);
    I2Cdev::writeByte(addr, LED0_OFF_H+(num*4), off>>8);
}
