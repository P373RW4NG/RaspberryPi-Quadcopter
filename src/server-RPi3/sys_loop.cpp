#include <array>
#include <cstdint>
#include <stdio.h>
#include <string>
#include <iostream>

#include "transmission.h"
#include "imu/I2Cdev.h"
#include "imu/MPU6050_6Axis_MotionApps20.h"
#include "algorithm/filter.h"
#include "algorithm/MadgwickAHRS.h"
#include "pwm/motor_control.h"

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include  <raspicam/raspicam_cv.h>

#include "ultrasonic.h"
#define _USE_MATH_DEFINES
#define complementaryFilter 1

/* accUserOffset and angleOffset are both use for calibrate roll and pitch angles.*/
#define accUserOffset 1
#define angleOffset 0
#define vid 1
#define webcam 0
#define raspicam 1
#define img_w 480
#define img_h 360

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
bool setupRdy = false;
bool caliRdy = false;
bool sr04Rdy = false;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0, mx=0, my=0, mz=0; //temp , send to client
//volatile float q0, q1, q2, q3;
uint16_t mtFL=819, mtFR=819, mtRL=819, mtRR=819, th_level=819;

int16_t userOffset_ax = -215;
int16_t userOffset_ay = -100;
int16_t userOffset_az = 0;

int16_t axBias=0;
int16_t ayBias=0;
int16_t azBias=0;

int16_t gxBias=0;
int16_t gyBias=0;
int16_t gzBias=0;

// orientation/motion vars
Quaternion qv;
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float roll=0, pitch=0, yaw=0;
float rollOffset= -0.011, pitchOffset= -0.026;

float att=0;
float m1[2]={1/sqrt(2), 1/sqrt(2)};
float m2[2]={-1/sqrt(2), 1/sqrt(2)};
float m3[3]={-1/sqrt(2), -1/sqrt(2)};
float m4[4]={1/sqrt(2), -1/sqrt(2)};
float unv[3]; // unit normal vector
float m1p, m2p, m3p, m4p;
uint16_t zsc;

std::string usrcmd = "00000000";
std::string mo_cmd = "0000";
std::string att_cmd= "00";

float Ax, Ay, Az;
float Gx, Gy, Gz;
float Mx, My, Mz;

const float accRes = 9.81/8192.0;
const float gyroRes = 1.0/65.536;
const float magRes = 8192.0/2400.0;

float ftr_dt;
clock_t curr_t, ftr_curr_t;
clock_t prev_t = 0, ftr_prev_t = 0;

//PID
float HTol = 3;
float balAngTol = 1.75;
float WzTol = 0.3;

float desired_H, desired_roll, desired_pitch, desired_Wz;
float prev_H = th_min;
float e_H, e_roll, e_pitch, e_Wz;
float prev_e_H=0;
float prev_e_roll=0, prev_e_pitch=0;
float prev_e_Wz=0;
float dt;

float pid_saturator = 300;
float pid_roll_max = 400;
float pid_roll_min = -400;

float I_e_H, I_e_roll, I_e_pitch, I_e_Wz;
float D_e_H, D_e_roll, D_e_pitch, D_e_Wz;

float Kp_H=0, Ki_H=0, Kd_H=0;
float Kp_roll=3.3, Ki_roll=0, Kd_roll=0;
float Kp_pitch=3.3, Ki_pitch=0, Kd_pitch=0;
float Kp_Wz=3, Ki_Wz=0, Kd_Wz=0;

float roll_I_term=0, pitch_I_term=0, Wz_I_term=0;

float u1, u2, u3, u4;

const float Ixx=0.0746;
const float Iyy=0.08283;
const float Izz=0.14445;

const float coe1=1, coe2=1, coe3=1;

float prev_roll=0;
float prev_pitch=0;
float prev_Gz=0;

float droll, dpitch, dGz;

bool flightState = false;
//

void sendMsg(tcp::socket *sock, std::string msg){
    msg.resize(50);
    write(*sock, buffer(msg), boost::asio::transfer_all());
}
void calibration(tcp::socket* sock, VectorInt16* accel, VectorInt16* gyro){ // only gyro
    int sampleCtr = 0;
    int n=0;
    int gyroSum[3] = {0,0,0};
    int acclSum[3] = {0,0,0};

    while(sampleCtr<1000){
        std::ostringstream ostr;
        if (!dmpReady) return;
            fifoCount = mpu.getFIFOCount();
        if (fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            printf("FIFO overflow!\n");
        }
        else if (fifoCount >= 42) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.getMotion6(&accel->x, &accel->y, &accel->z,
                           &gyro->x, &gyro->y, &gyro->z);
            printf("gx: %7d, gy: %7d, gz: %7d \n", gyro->x, gyro->y, gyro->z);
        gyroSum[0] += gyro->x;
        gyroSum[1] += gyro->y;
        gyroSum[2] += gyro->z;
        acclSum[0] += accel->x;
        acclSum[1] += accel->y;
        acclSum[2] += accel->z;
       
        sampleCtr++;
        }
    }
    printf("gx sum= %7d, gy sum=%7d, gz sum=%7d \n", gyroSum[0], gyroSum[1], gyroSum[2]);
    usleep(3000000);

    gxBias = gyroSum[0]/sampleCtr;
    gyBias = gyroSum[1]/sampleCtr;
    gzBias = gyroSum[2]/sampleCtr;
    axBias = acclSum[0]/sampleCtr;
    ayBias = acclSum[1]/sampleCtr;
    azBias = acclSum[2]/sampleCtr - 8192;
}

void setup(tcp::socket *sock, VectorInt16* accel, VectorInt16* gyro) {

    //sendMsg(sock, "Initializing ultrasonic sensor...\n");
    //SR04setup();

    // initialize device
    //printf("Initializing I2C devices...\n");
    sendMsg(sock, "Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    //printf("Testing device connections...\n");
    sendMsg(sock, "Testing device connections...\n");
    //printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    bool mpuTestCn=mpu.testConnection();
    if(mpuTestCn==true){
        sendMsg(sock, "MPU6050 connection successful\n");
    }else{
        sendMsg(sock, "MPU6050 connection failed\n");
    }
    // load and configure the DMP
    printf("Initializing DMP...\n");
    sendMsg(sock, "Initializing DMP...\n");

    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        sendMsg(sock, "Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        sendMsg(sock, "DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        sendMsg(sock, "DMP Initialization failed\n");
    }


    //dmpReady = true;
    sendMsg(sock, "Initialization: OK\n");
    sendMsg(sock, "Calibrating IMU ...\n");
    //mpu.resetFIFO();

    calibration(sock, accel, gyro);

    sendMsg(sock, "Calibration: OK\t\n");

    caliRdy = true;
    //sr04Rdy = true;
    setupRdy = true;
    //std::cout<<std::to_string(mpu.getFullScaleAccelRange())<<std::endl;
    //std::cout<<std::to_string(mpu.getFullScaleGyroRange())<<std::endl;

}
void readIMUdata(Quaternion* q, VectorInt16* accel, VectorInt16* gyro, VectorInt16* magneto, VectorFloat* gravity){
    while(setupRdy==false){} //wait for the IMU setup to complete.
    while(true){

    //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

    if (!dmpReady) return;
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");
    }
    else if (fifoCount >= 42) {

        if(caliRdy==true){
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //mpu.dmpGetQuaternion(q, fifoBuffer);

        mpu.getMotion9(&accel->x, &accel->y, &accel->z,
                       &gyro->x, &gyro->y, &gyro->z,
                       &magneto->x, &magneto->y, &magneto->z);
        
#if(accUserOffset==1)
        ax=accel->x - userOffset_ax;
        ay=accel->y - userOffset_ay;
        az=accel->z - userOffset_az;
#else
        ax=accel->x;
        ay=accel->y;
        az=accel->z;
#endif
        gx=gyro->x - gxBias;
        gy=gyro->y - gyBias;
        gz=gyro->z - gzBias;
        mx=magneto->x;
        my=magneto->y;
        mz=magneto->z;

        // MKS unit
        Ax=ax*accRes; // m/(s*s)
        Ay=ay*accRes;
        Az=az*accRes;
        Gx=gx*gyroRes;  // degrees/s
        Gy=gy*gyroRes;
        Gz=gz*gyroRes;
        Mx=mx*magRes;  // uT
        My=my*magRes;
        Mz=mz*magRes;

        ftr_curr_t = clock();
        if(ftr_prev_t != 0){
            ftr_dt = float(ftr_curr_t - ftr_prev_t) /CLOCKS_PER_SEC; // sec
            //ftr_dt = 1 ;
        }
        ftr_prev_t = ftr_curr_t;

        mpu.dmpGetQuaternion(&qv, fifoBuffer);
        mpu.getYPR(ypr, Ax, Ay, Az);

        //std::cout<< qv.w <<" "<< qv.x<<" "<< qv.y<< " "<< qv.z<<std::endl;
        //std::cout<< q->w <<" "<< q->x<<" "<< q->y<< " "<< q->z<<std::endl;
        //std::cout<<std::endl;
        yaw=ypr[0];
#if (complementaryFilter==1)

        roll = complFtr(roll, ypr[2], Gx*M_PI/180, ftr_dt, 0.9);
        pitch = complFtr(pitch, ypr[1], Gy*M_PI/180, ftr_dt, 0.9);

#else

        pitch=ypr[1];
        roll=ypr[2];

#endif
#if (angleOffset==1)
        roll -= rollOffset;
        pitch -= pitchOffset;
#endif
        }
    }
    }

}
void sendIMUdata(tcp::socket* sock){

    std::vector<int> imudata={ax, ay, az, gx, gy, gz, mx, my, mz};
    imudata.resize(9);
    write(*sock, buffer(imudata), boost::asio::transfer_all());
    //sock->write_some(buffer(imudata));
    //std::cout<<Ax<<" "<<Ay<<" "<<Az<<" "<<Gx<<" "<<Gy<<" "<<Gz<<std::endl;
}
void sendYPR(tcp::socket* sock){
    std::vector<float> yprdata={yaw, pitch, roll};
    yprdata.resize(3);
    write(*sock, buffer(yprdata), boost::asio::transfer_all());
}
void sendThrust(tcp::socket* sock){
    std::vector<int> thru4={mtFL, mtFR, mtRL, mtRR};
    thru4.resize(4);
    write(*sock, buffer(thru4), boost::asio::transfer_all());
}
void sendHeight(tcp::socket* sock){
    std::vector<float> h_cm={dist/100.0};
    h_cm.resize(1);
    write(*sock, buffer(h_cm), boost::asio::transfer_all());
}
void readPIDvars(tcp::socket* sock){
    std::array<float, 12> ks;
    size_t ks_len = read(*sock, buffer(ks), boost::bind(len_chk, 12, _1, ks_len));

    Kp_H = ks.at(0);
    Ki_H = ks.at(1);
    Kd_H = ks.at(2);
    Kp_roll = ks.at(3);
    Ki_roll = ks.at(4);
    Kd_roll = ks.at(5);
    Kp_pitch = ks.at(6);
    Ki_pitch = ks.at(7);
    Kd_pitch = ks.at(8);
    Kp_Wz = ks.at(9);
    Ki_Wz = ks.at(10);
    Kd_Wz = ks.at(11);
}
#if (vid==1)
void readCam(cam_t* cap, cv::Mat* frame , unsigned long* fcount, int* chn){
    bool re;
#if (webcam==1)
    re=cap->usbcam.read(*frame);
#endif
#if(raspicam==1)
    //cv::Mat frameTmp;
    re=cap->rpicam.grab();
    cap->rpicam.retrieve(*frame);
    //cv::flip(frameTmp, frameTmp, 0);
    //cv::flip(frameTmp, *frame, 1);
#endif
    *fcount++;
    *chn=frame->channels();
    if(!re){
        std::cout<<"error: can not create video frame."<<std::endl;
    }
}

void vid_send(tcp::socket* vid_sock, cv::Mat frame, cv::Mat* nframe, std::vector<int>* param, double* fps, uchar* idx)
{
    if(usrcmd[6]=='1'){

        if(frame.isContinuous()){
            //std::cout<<"frame accepted."<<std::endl;
            sendFPS(vid_sock, *fps, *idx);
            cv::resize(frame, *nframe, nframe->size(), 0, 0,CV_INTER_AREA);
            sendframe(vid_sock, *nframe, *param); //send length, then frame
            //std::cout<<"frame sent. "<<std::endl;
        }
        else{
            std::cout<<"error: frame pixels are not continuous. ";
        }
    }
}
#endif
void cmd_data_rw(tcp::socket* sock, VectorInt16* accel, VectorInt16* gyro){  //refer to GUIClient MainWindow::loop()
    setup(sock, accel, gyro); //setup imu, pwm pca9685
    //mpu.initialize();
    for(;;){
        //read user command
        char buff[1024];
        int bytes=read(*sock, buffer(buff), boost::bind(read_complete, buff, _1, _2));
        std::string msg(buff, bytes);
        usrcmd=msg.substr(0, msg.size()-1); //remove \n
        std::string moCmd(usrcmd, 0, 4);
        std::string attCmd(usrcmd, 4, 2);
        mo_cmd=moCmd;
        att_cmd=attCmd;
        //std::cout<<usrcmd<<' '<<mo_cmd<<' '<<att_cmd<<"   FL: "<<mtFL<<" FR: "<<mtFR<<" RL: "<<mtRL<<" RR: "<<mtRR<<std::endl;
        //std::cout<<msg;
        sock->write_some(buffer(msg));
        // send IMU data to client
        sendIMUdata(sock);
        sendYPR(sock);
        sendThrust(sock);
        //sendHeight(sock);
        readPIDvars(sock);
    }
}

int main(){

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 accel;         // [x, y, z]            accel sensor measurements
    VectorInt16 gyro;
    VectorInt16 magneto;
    VectorFloat gravity;

    cam_t cam;
    double w,h,fps;
    int chn;
    unsigned long fcount=0;
    uchar idx=0;
    cv::Mat frame, nframe(img_h, img_w, CV_8UC3, cv::Scalar::all(0));
    std::vector<int> param{CV_IMWRITE_JPEG_QUALITY, 50};
#if(vid==1 && webcam==1)
    cam.usbcam=cv::VideoCapture(0);
    cam.usbcam.set(cv::CAP_PROP_FOURCC, CV_FOURCC('H','2','6','4'));
    w=cam.usbcam.get(cv::CAP_PROP_FRAME_WIDTH);
    h=cam.usbcam.get(cv::CAP_PROP_FRAME_HEIGHT);
    fps=cam.usbcam.get(cv::CAP_PROP_FPS);
#endif
#if(vid==1 && raspicam==1)
    cam.rpicam.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    cam.rpicam.set(cv::CAP_PROP_FPS, 60);
    cam.rpicam.set(cv::CAP_PROP_FRAME_WIDTH, 480);
    cam.rpicam.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
    w=cam.rpicam.get(cv::CAP_PROP_FRAME_WIDTH);
    h=cam.rpicam.get(cv::CAP_PROP_FRAME_HEIGHT);
    fps=cam.rpicam.get(cv::CAP_PROP_FPS);
    cam.rpicam.open();
    if (!cam.rpicam.isOpened()) {std::cerr<<"Error opening the RPI camera"<<std::endl;}
#endif
#if(vid==1)
    readCam(&cam, &frame, &fcount, &chn); //initialize, for check continuous in vid_send
#endif
    //initialize communication
    io_service service, vid_service;
    tcp::socket sock(service);
    tcp::socket vid_sock(vid_service);

    tcp::acceptor vid_acceptor(vid_service, tcp::endpoint(tcp::v4(), 2017));
    tcp::acceptor acceptor(service, tcp::endpoint(tcp::v4(), 2018));

    acceptor.accept(sock);
    vid_acceptor.accept(vid_sock);

    std::string prev_mo_cmd = "0000";
    std::string prev_att_cmd = "00";

    motorSetup();

    boost::thread thd2([&](){cmd_data_rw(&sock, &accel, &gyro);});
    boost::thread thd3([&](){readIMUdata(&q, &accel, &gyro, &magneto, &gravity);});
#if(vid==1)
    boost::thread thd4([&](){while(true){readCam(&cam, &frame, &fcount, &chn);}});
    boost::thread thd1([&, frame](){for(;;){vid_send(&vid_sock, frame, &nframe, &param ,&fps, &idx);}});
#endif
    //boost::thread thd5([&](){while(true){if(sr04Rdy==true){ height(); att=dist;}}});


    while(true){


        curr_t = clock();
        if(prev_t !=0){
            dt = float(curr_t - prev_t)/CLOCKS_PER_SEC; // sec
            //dt = 1;
        }
        //shut down all motors
        if(mo_cmd=="1111"){
            mtFL=th_min;
            mtFR=th_min;
            mtRL=th_min;
            mtRR=th_min;
            prev_H = th_min;
            u1 = th_min;
            prev_e_H = 0;
            prev_e_roll = 0;
            prev_e_pitch = 0;
            prev_e_Wz = 0;
        }else{


            desired_H = prev_H;
            desired_pitch = 0;
            desired_roll = 0;
            desired_Wz = 0;

            //forward
            if(mo_cmd=="1000"){
                desired_pitch = 15;
                desired_roll = 0;
                desired_Wz = 0;
            }
            //tilt left
            else if(mo_cmd=="0100" && usrcmd[7]=='0' ){
                desired_pitch = 0;
                desired_roll = 15;
                desired_Wz = 0;
            }
            //backward
            else if(mo_cmd=="0010"){
                desired_pitch = -15;
                desired_roll = 0;
                desired_Wz = 0;
            }
            //tilt right
            else if(mo_cmd=="0001" && usrcmd[7]=='0' ){
                desired_pitch = 0;
                desired_roll = -15;
                desired_Wz = 0;
            }


            if(att_cmd == "10"){
                desired_H += h_inc;
            }
            else if(att_cmd == "01"){
                desired_H -= h_inc;
            }

            // turn left, c.c.w. spin
            if(mo_cmd[1]=='1' && usrcmd[7]=='1'){
                desired_Wz = 30;
            }// turn right, c.w. spin
            else if(mo_cmd[3]=='1' && usrcmd[7]=='1'){
                desired_Wz = -30;
            }


        e_H = desired_H - (mtFL+mtFR+mtRL+mtRR);
        e_roll = desired_roll - (roll*180/M_PI);
        e_pitch = desired_pitch - (pitch*180/M_PI);
        e_Wz = desired_Wz - Gz;

        //if( (fabs(double(e_roll))>balAngTol) || (fabs(double(e_pitch))>balAngTol)){
        // throttle(H) part
        //I_e_H += e_H*dt;
        //D_e_H = (e_H - prev_e_H)/dt;
        //u1 = Kp_H*e_H + Ki_H*I_e_H + Kd_H*D_e_H;
        u1 = desired_H;
            
        // roll part
        //u2 = pidCal(Kp_roll, Ki_roll, Kd_roll ,e_roll, prev_e_roll, dt, roll_I_term, pid_saturator, -pid_saturator);
        u2 = pidCal_v2(Kp_roll, Ki_roll, Kd_roll ,e_roll, prev_e_roll, dt, roll_I_term, pid_saturator, -pid_saturator);
            
        //pitch part
        //u3 = pidCal(Kp_pitch, Ki_pitch, Kd_pitch, e_pitch, prev_e_pitch, dt, pitch_I_term, pid_saturator, -pid_saturator);
        u3 = pidCal_v2(Kp_pitch, Ki_pitch, Kd_pitch, e_pitch, prev_e_pitch, dt, pitch_I_term, pid_saturator, -pid_saturator);
            
        //yaw angular velocity (Wz) part
        u4 = pidCal(Kp_Wz, Ki_Wz, Kd_Wz, e_Wz, prev_e_Wz, dt, Wz_I_term, pid_saturator, -pid_saturator);

        prev_e_H = e_H;
        prev_e_roll = e_roll;
        prev_e_pitch = e_pitch;
        prev_e_Wz = e_Wz;

        if(u1<830){
            flightState = false;
            mtFL = th_min;
            mtFR = th_min;
            mtRR = th_min;
            mtRL = th_min;
        }else if(u1>=830){

            flightState = true;
            mtFL = (u1/coe1) - ((u2+u3)/coe2) + (u4/coe3);
            mtFR = (u1/coe1) + ((u2-u3)/coe2) - (u4/coe3);
            mtRR = (u1/coe1) + ((u2+u3)/coe2) + (u4/coe3);
            mtRL = (u1/coe1) - ((u2-u3)/coe2) - (u4/coe3);
        }
        //std::cout<<u2 << " "<< u3 <<std::endl;
        }

        prev_H = u1;
        prev_t = curr_t;

        mtFL=valChk(mtFL);
        mtFR=valChk(mtFR);
        mtRL=valChk(mtRL);
        mtRR=valChk(mtRR);
        //std::cout<<usrcmd<<' '<<mo_cmd<<' '<<att_cmd<<"   FL: "<<mtFL<<" FR: "<<mtFR<<" RL: "<<mtRL<<" RR: "<<mtRR <<" u1: "<<u1<<" u2: "<<u2 << " u3: "<< u3 <<" u4: "<<u4<< " dt: "<< dt<<" ftr_dt: "<<ftr_dt<<std::endl;
        motor_FL(mtFL);
        motor_FR(mtFR);
        motor_RL(mtRL);
        motor_RR(mtRR);
        //usleep(5000);
    }

}


