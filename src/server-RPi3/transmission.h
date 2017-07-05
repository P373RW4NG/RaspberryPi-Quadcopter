#ifndef CONNECTION_H
#define CONNECTION_H

#include <time.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <raspicam/raspicam_cv.h>



using namespace boost::asio;
using namespace boost::asio::ip;




size_t len_chk(int n, const boost::system::error_code& error, size_t bytes){
    return bytes==n? 0:1;
}

size_t read_complete(char* buf, const boost::system::error_code& err, size_t bytes){
    if(err) return 0;
    bool found = std::find(buf, buf+bytes, '\n') < buf + bytes;
    return found? 0:1;
}



void sendFPS(tcp::socket* vid_sock, double fps, uchar& idx){
    if(idx==0){ //send FPS
        std::string info=std::to_string(fps);
        //std::cout<<"Frame per second: "<<info<<std::endl;
        info.resize(8);
        write(*vid_sock, buffer(info), boost::asio::transfer_all());
        idx++;
    }
}
void sendframe(tcp::socket* vid_sock, cv::Mat& frame, std::vector<int>& param){
    std::vector<uchar> vidBuff;
    //frame.copyTo(vid);
    unsigned long betm=clock();
    cv::imencode(".jpg", frame, vidBuff, param);
    //cv::imencode(".avi", frame, vidBuff, CV_FOURCC('M','J','P','G'));
    unsigned long eetm=clock();
    double t_itrv=(eetm-betm)/ (CLOCKS_PER_SEC/1000.0);
    std::string headlen(std::to_string(vidBuff.size()));  //length buffer
    headlen.resize(16);
    //std::cout<<"encoded buffer size: "<<vidBuff.size()<<std::endl;
    write(*vid_sock, buffer(headlen), boost::asio::transfer_all()); //send length

    write(*vid_sock, buffer(vidBuff), boost::asio::transfer_all()); //send frame
    //printf("coding time: %2.4f ms\t", t_itrv);
}

class cam_t{
public:
    cv::VideoCapture usbcam;
    raspicam::RaspiCam_Cv rpicam;
};

/*
void vid_send(tcp::socket* vid_sock){
    cv::VideoCapture cap("/home/pi/Desktop/bike.mp4");
    //cv::VideoCapture cap("/Users/Peter/Downloads/SIRO-2365/SIRO-2365.mp4");
    //cv::VideoCapture cap("/Users/Peter/Downloads/BTLG/[Beautyleg]2016-01-19_No.616_Jennifer_4K/616Jennifer.mp4");
    //cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FOURCC, CV_FOURCC('H','2','6','4'));
    double w=cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double h=cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps=cap.get(cv::CAP_PROP_FPS);
    int chn;
    unsigned long fcount=0;
    uchar idx=0;
    cv::Mat frame, nframe(405, 720, CV_8UC3, cv::Scalar::all(0));
    //std::string vid;

    std::vector<int> param=std::vector<int>(2);
    param[0]=CV_IMWRITE_JPEG_QUALITY;
    param[1]=50;

    while(1){

        bool re=cap.read(frame);
        fcount++;
        chn=frame.channels();
        if(!re){
            std::cout<<"error: can not create video frame."<<std::endl;
            break;
        }

        if(frame.isContinuous()){
            std::cout<<"frame accepted."<<std::endl;
            sendFPS(vid_sock, fps, idx);
            cv::resize(frame, nframe, nframe.size(), 0, 0,CV_INTER_AREA);
            sendframe(vid_sock, nframe, param);
            std::cout<<"frame sent. "<<std::endl;
        }
        else{
            std::cout<<"error: frame pixels are not continuous. ";
            break;
        }
        //vid_sock.close();
    }
}

void cmmd_rw(tcp::socket* sock){
    while(true){


        char buff[1024];
        int bytes = read(*sock, buffer(buff), boost::bind(read_complete, buff, _1, _2));
        std::string msg(buff, bytes);
        //if(msg=="w\n"){std::cout<<'*';}
        std::cout<<msg;
        //std::cout<<bytes;
        sock->write_some(buffer(msg));
        //sock.close();
    }
}
*/






#endif // CONNECTION_H
