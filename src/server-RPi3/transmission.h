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

#endif // CONNECTION_H
