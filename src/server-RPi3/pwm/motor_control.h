#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "PCA9685.h"
#ifndef IOSTR_H
#define IOSTR_H
#include <stdio.h>
#include <string>
#include <iostream>
#endif
/* Objective: Provide functions for motor control and PID algorithm.
 * Author: Peter Wang
 * Date: 2017/05/17
 */
//view from top
uint8_t pin_fl=12;
uint8_t pin_fr=13;
uint8_t pin_rl=14;
uint8_t pin_rr=15;
//macro
const uint16_t th_min = 819;  // no thrust
const uint16_t th_max = 1800; //1638
uint16_t h_inc = 2;  // increasement of height
//uint16_t th_stack = th_min;
uint16_t inc0 = 20;  // tilt increasement
int16_t mtFRinc, mtFLinc, mtRRinc, mtRLinc;

PCA9685 pwmCtrl;

extern float pid_saturator;
extern float pid_roll_max;
extern float pid_roll_min;
extern bool flightState;

float pidLimit(float v, float u, float d){
    if(u==0 && d==0){
        return v;
    }else{
        if(v>u){
            v=u;
        }else if(v<d){
            v=d;
        }
        return v;
    }
}

float pidCal(float kp, float ki, float kd, float e, float prev_e, float dt, float &I_term,float sat_u, float sat_l){
    float output;
    float P_term, D_term;

    P_term = kp*e;
    if(flightState){
        I_term += ki*e*dt;  // first, I_term use as integration part, then multiple ki become I_term
    }else if(!flightState){
        I_term = 0;
        return 0;
    }
    if(sat_u!=0 && sat_l!=0){
        if(I_term > sat_u){ I_term = sat_u; }
        else if(I_term < sat_l){ I_term = sat_l;}
    }
    //I_term *= ki;
    D_term = kd*(e - prev_e)/dt;
    output = P_term + I_term + D_term;
    output = pidLimit(output, pid_roll_max, pid_roll_min);
    return output;
}

void standby(){
    pwmCtrl.setMotor(pin_fl, th_min);
    pwmCtrl.setMotor(pin_fr, th_min);
    pwmCtrl.setMotor(pin_rl, th_min);
    pwmCtrl.setMotor(pin_rr, th_min);
}

void motorSetup(){
    pwmCtrl.initialize();
    pwmCtrl.setMode1(0x00);
    standby();
}

void motor_FL(uint16_t val){
    pwmCtrl.setMotor(pin_fl, val);
}
void motor_FR(uint16_t val){
    pwmCtrl.setMotor(pin_fr, val);
}
void motor_RL(uint16_t val){
    pwmCtrl.setMotor(pin_rl, val);
}
void motor_RR(uint16_t val){
    pwmCtrl.setMotor(pin_rr, val);
}

void attChange(std::string att_cmd, uint16_t &att){
    if(att_cmd=="10"){
        att += 50;
    }
    if(att_cmd=="01"){
        att -= 50;
    }
}
uint16_t valChk(uint16_t val){
    if(val>th_max){
        val=th_max;
    }
    if(val<th_min){
        val=th_min;
    }

    return val;


}
/*
void dirChange(std::string mo_cmd, std::string att_cmd){
    if(mo_cmd=="1000"){
       attChange(att_cmd);
       motor_FL(th_stack);
       motor_FR(th_stack);
       motor_RL(th_stack+inc0);
       motor_RR(th_stack+inc0);
    }
    if(mo_cmd=="0100"){
       attChange(att_cmd);
       motor_FL(th_stack);
       motor_FR(th_stack+inc0);
       motor_RL(th_stack);
       motor_RR(th_stack+inc0);
    }
    if(mo_cmd=="0010"){
       attChange(att_cmd);
       motor_FL(th_stack+inc0);
       motor_FR(th_stack+inc0);
       motor_RL(th_stack);
       motor_RR(th_stack);
    }
    if(mo_cmd=="0001"){
       attChange(att_cmd);
       motor_FL(th_stack+inc0);
       motor_FR(th_stack);
       motor_RL(th_stack+inc0);
       motor_RR(th_stack);
    }
}
*/

#endif // MOTOR_CONTROL_H
