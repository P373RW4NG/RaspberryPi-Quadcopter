#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <wiringPi.h>
#include <ctime>
#include <unistd.h>

int trig=27;
int echo=28;
int dur, r;
float dist;
clock_t str=0;
clock_t end=0;

void SR04setup(){
    wiringPiSetup();
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}
void height(){
    digitalWrite(trig, HIGH);
    usleep(10);
    digitalWrite(trig, LOW);

    while(digitalRead(echo)==0){
        str=clock();
    }
    while(digitalRead(echo)==1){
        end=clock();
        if(str==0){
            //std::cout<<"reset***************************************"<<std::endl;
            return;
        }
    }
    dur=end-str; //micro sec.
    dist=0.5*0.034321*dur;

    //std::cout<<dist<<" cm"<<std::endl;

    //reset
    str=0;
    end=0;

    usleep(100000);  // repeat after 100ms
}

#endif // ULTRASONIC_H
