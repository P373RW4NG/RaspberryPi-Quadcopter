#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QWidget>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QTextCursor>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <unistd.h>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include <fstream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define _USE_MATH_DEFINES
#define vid 1

using namespace boost::asio;
using namespace boost::asio::ip;

template<class T>
void convertFromStr(const std::string str, T &d){
    std::stringstream ss(str);
    ss >> d;
}

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *krevent);
    std::string keyHold="00000000"; //Key: W, A, S, D, Up, Down, I, Left/Right

private:
    Ui::MainWindow *ui; 
    int readIMUSetup();
    void loop();
    void sendKeyStat();
    void readIMUdata();
    void readYPR();
    float yaw, pitch, roll;
    void readThrust();
    void readHeight();
    void sendKvalues();
    void recVid();
    int p=0;
    int idx[5]={0, 0, 0, 0, 0}; // state: loop, vid, lock, body detection, face detection
    double fps;
    cv::Mat frame = cv::Mat::zeros(360, 480, CV_8UC3), frame_gray;
    cv::Mat gyroFig = cv::Mat::zeros(130, 130, CV_8UC3);
    QImage Qimg, QimgGyro;
    QTextCursor curs;
    io_service service, vid_service;
    //tcp::endpoint vid_ep=tcp::endpoint(address::from_string("10.42.0.184"), 2017);
    //tcp::endpoint ep=tcp::endpoint(address::from_string("10.42.0.184"), 2018);
    tcp::endpoint vid_ep=tcp::endpoint(address::from_string("192.168.1.3"), 2017);
    tcp::endpoint ep=tcp::endpoint(address::from_string("192.168.1.3"), 2018);
    tcp::socket sock=tcp::socket(service);
    tcp::socket vid_sock=tcp::socket(vid_service);

    void gyroPlot();
    std::string strtmp;
    std::vector<std::string> strs;
    void loadSettings();

    // detection data
    std::string body_cascade_name ="/Users/Peter/OpenCV/opencv-3.2.0/data/haarcascades/haarcascade_fullbody.xml";
    std::string face_cascade_name = "/Users/Peter/OpenCV/opencv-3.2.0/data/haarcascades/haarcascade_frontalface_alt.xml";
    std::string eyes_cascade_name = "/Users/Peter/OpenCV/opencv-3.2.0/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

    cv::CascadeClassifier body_cascade;
    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;

    void cascadeBodyDetection();
    void cascadeFaceDetection();
    void HOGPeopleDetection();

signals:
    void valChange(int v);
    void getAx(int v);
    void getAy(int v);
    void getAz(int v);
    void getGx(int v);
    void getGy(int v);
    void getGz(int v);
    void getMx(int);
    void getMy(int);
    void getMz(int);
    void getYaw(float);
    void getPitch(float);
    void getRoll(float);
    void getThrustFL(int);
    void getThrustFR(int);
    void getThrustRL(int);
    void getThrustRR(int);
    void valChange2(QString s);
    void showImg(QPixmap m);
    void showGyro(QPixmap m);
    void imgResz();
    void showStr(QString);
    void detStr();

private slots:
    void labResz();
    void setAx(int);
    void setAy(int);
    void setAz(int);
    void setGx(int);
    void setGy(int);
    void setGz(int);
    void setMx(int);
    void setMy(int);
    void setMz(int);
    void setYaw(float);
    void setPitch(float);
    void setRoll(float);
    void setText1();
    void setText2();
    void sockConnection();
    void setLock();
    void saveSettings();
    void detln();
    void on_faceButton_clicked();
    void on_bodyButton_clicked();
};

#endif // MAINWINDOW_H
