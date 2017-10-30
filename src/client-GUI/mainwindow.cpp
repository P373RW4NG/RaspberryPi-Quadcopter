#include "mainwindow.h"
#include "ui_mainwindow.h"


size_t read_complete(char* buf, const boost::system::error_code& err, size_t bytes){
    if(err) return 0;
    bool found = std::find(buf, buf+bytes, '\n') < buf + bytes;
    return found? 0:1;
}

size_t len_check(int x ,const boost::system::error_code& err, size_t bytes){
    return bytes==x? 0:1;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //sock.connect(ep);
    //vid_sock.connect(vid_ep);
    ui->setupUi(this);

    //ui->gyro->setPixmap(QPixmap::fromImage(QImage((const unsigned char*)(gyroFig.data), gyroFig.cols, gyroFig.rows, QImage::Format_RGB888)));
    ui->vidLabel->setPixmap(QPixmap::fromImage(QImage((const unsigned char*)(frame.data), frame.cols, frame.rows, QImage::Format_RGB888)));
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::setText1 );
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::sockConnection);
    connect(ui->vidButton, &QPushButton::clicked, this, &MainWindow::setText2 );
    connect(ui->lockButton, &QPushButton::clicked, this, &MainWindow::setLock );
    connect(ui->saveButton, &QPushButton::clicked, this, &MainWindow::saveSettings );
    loadSettings();

    if(!body_cascade.load(body_cascade_name)){
        std::cout<<"error: body loading failed"<<std::endl;
    }
    if(!face_cascade.load(face_cascade_name)){
        std::cout<<"error: face loading failed"<<std::endl;
    }
    if(!eyes_cascade.load(eyes_cascade_name)){
        std::cout<<"error: eyes loading failed"<<std::endl;
    }

    //ui->statusLabel->setText("");
    ui->textBrowser->setFocusPolicy(Qt::NoFocus);  //allow keys not be captured or interrupted by text browser


    ui->pgBar_ax->setRange(-32768, 32767);
    ui->pgBar_ay->setRange(-32768, 32767);
    ui->pgBar_az->setRange(-32768, 32767);
    ui->pgBar_gx->setRange(-32768, 32767);
    ui->pgBar_gy->setRange(-32768, 32767);
    ui->pgBar_gz->setRange(-32768, 32767);
    ui->pgBar_mx->setRange(-4096, 4095);
    ui->pgBar_my->setRange(-4096, 4095);
    ui->pgBar_mz->setRange(-4096, 4095);
    ui->pgBar_yaw->setRange(-314, 314);
    ui->pgBar_pitch->setRange(-314, 314);
    ui->pgBar_roll->setRange(-314, 314);
    //connect(this, SIGNAL(valChange(int)), ui->progressBar, SLOT(setValue(int)) );

    connect(this, &MainWindow::showImg, ui->vidLabel, &QLabel::setPixmap);
    connect(this, &MainWindow::imgResz, this, &MainWindow::labResz);

    connect(this, &MainWindow::showGyro, ui->gyro, &QLabel::setPixmap);

    connect(this, &MainWindow::showStr, ui->textBrowser, &QTextBrowser::append);
    //connect(this, &MainWindow::detStr, this, &MainWindow::detln);
    //curs = ui->textBrowser->textCursor();
    connect(this, &MainWindow::getAx, ui->pgBar_ax, &QProgressBar::setValue);
    connect(this, &MainWindow::getAy, ui->pgBar_ay, &QProgressBar::setValue);
    connect(this, &MainWindow::getAz, ui->pgBar_az, &QProgressBar::setValue);
    connect(this, &MainWindow::getGx, ui->pgBar_gx, &QProgressBar::setValue);
    connect(this, &MainWindow::getGy, ui->pgBar_gy, &QProgressBar::setValue);
    connect(this, &MainWindow::getGz, ui->pgBar_gz, &QProgressBar::setValue);
    connect(this, &MainWindow::getMx, ui->pgBar_mx, &QProgressBar::setValue);
    connect(this, &MainWindow::getMy, ui->pgBar_my, &QProgressBar::setValue);
    connect(this, &MainWindow::getMz, ui->pgBar_mz, &QProgressBar::setValue);
    //connect(this, &MainWindow::getYaw, ui->pgBar_yaw, &QProgressBar::setValue);
    //connect(this, &MainWindow::getPitch, ui->pgBar_pitch, &QProgressBar::setValue);
    //connect(this, &MainWindow::getRoll, ui->pgBar_roll, &QProgressBar::setValue);
    connect(this, &MainWindow::getThrustFL, ui->pgBar_FL, &QProgressBar::setValue);
    connect(this, &MainWindow::getThrustFR, ui->pgBar_FR, &QProgressBar::setValue);
    connect(this, &MainWindow::getThrustRL, ui->pgBar_RL, &QProgressBar::setValue);
    connect(this, &MainWindow::getThrustRR, ui->pgBar_RR, &QProgressBar::setValue);


    connect(this, &MainWindow::getAx, this, &MainWindow::setAx);
    connect(this, &MainWindow::getAy, this, &MainWindow::setAy);
    connect(this, &MainWindow::getAz, this, &MainWindow::setAz);
    connect(this, &MainWindow::getGx, this, &MainWindow::setGx);
    connect(this, &MainWindow::getGy, this, &MainWindow::setGy);
    connect(this, &MainWindow::getGz, this, &MainWindow::setGz);
    connect(this, &MainWindow::getMx, this, &MainWindow::setMx);
    connect(this, &MainWindow::getMy, this, &MainWindow::setMy);
    connect(this, &MainWindow::getMz, this, &MainWindow::setMz);
    connect(this, &MainWindow::getYaw, this, &MainWindow::setYaw);
    connect(this, &MainWindow::getPitch, this, &MainWindow::setPitch);
    connect(this, &MainWindow::getRoll, this, &MainWindow::setRoll);

    roll=0, pitch=0;
    gyroPlot();


/*
    this->vid_ep=tcp::endpoint(address::from_string("10.42.0.184"), 2017); //156 for mac, 184 for pi
    this->ep=tcp::endpoint(address::from_string("10.42.0.184"), 2018);
    this->sock=tcp::socket(this->service);
    this->vid_sock=tcp::socket(this->vid_service);
*/
    boost::thread thd1([this](){try{loop();}catch(std::exception& e){std::cout<<e.what()<<std::endl;}});
#if(vid==1)
    boost::thread thd2([this](){recVid();});
#endif
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(!event->isAutoRepeat() && event->key()==Qt::Key_W){  //forward
        keyHold[0]='1';
        //ui->statusLabel->setText("Pressing W");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_A){  //move left
        keyHold[1]='1';
        //ui->statusLabel->setText("Pressing A");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_S){  //backward
        keyHold[2]='1';
        //ui->statusLabel->setText("Pressing S");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_D){  //move right
        keyHold[3]='1';
        //ui->statusLabel->setText("Pressing D");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Up ){  //increase throttle
        keyHold[4]='1';
        //ui->statusLabel->setText("Pressing Space");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Down){   //decrease throttle
        keyHold[5]='1';
        //ui->statusLabel->setText("Pressing Shift");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Escape ){  //shut down all motor
            keyHold[0]='1';
            keyHold[1]='1';
            keyHold[2]='1';
            keyHold[3]='1';
            //ui->statusLabel->setText("Pressed ESC");
        }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_I){   //video play/pause
        ui->vidButton->clicked();
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Left ){  // yaw control, spin(c.c.w.)
            keyHold[1]='1';
            keyHold[7]='1';
            //ui->statusLabel->setText("turn left");
        }
        if(!event->isAutoRepeat() && event->key()==Qt::Key_Right ){  // yaw control, spin(c.w.)
            keyHold[3]='1';
            keyHold[7]='1';
            //ui->statusLabel->setText("turn right");
        }

}

void MainWindow::keyReleaseEvent(QKeyEvent *event)  //reset buttons
{

    if(!event->isAutoRepeat() && event->key()==Qt::Key_W){
        keyHold[0]='0';
        //ui->statusLabel->setText("W Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_A){
        keyHold[1]='0';
        //ui->statusLabel->setText("A Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_S){
        keyHold[2]='0';
        //ui->statusLabel->setText("S Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_D){
        keyHold[3]='0';
        //ui->statusLabel->setText("D Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Up){
        keyHold[4]='0';
        //ui->statusLabel->setText("Space Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Down){
        keyHold[5]='0';
        //ui->statusLabel->setText("Shift Released");
    }
    if(!event->isAutoRepeat() && event->key()==Qt::Key_Escape ){  
            keyHold[0]='0';
            keyHold[1]='0';
            keyHold[2]='0';
            keyHold[3]='0';
            //ui->statusLabel->setText("ESC released");
        }
        if(!event->isAutoRepeat() && event->key()==Qt::Key_Left ){  
            keyHold[1]='0';
            keyHold[7]='0';
            //ui->statusLabel->setText("Key released");
        }
        if(!event->isAutoRepeat() && event->key()==Qt::Key_Right ){  
            keyHold[3]='0';
            keyHold[7]='0';
            //ui->statusLabel->setText("Key released");
        }
}

void MainWindow::loop()
{
    while(idx[0]!=1){}
    readIMUSetup();
    for(;;){
        sendKeyStat();
        readIMUdata();
        readYPR();
        readThrust();
        //readHeight();
        sendKvalues();
        gyroPlot();
    }
}

int MainWindow::readIMUSetup()
{
    for(;;){
        char s[100];
        size_t bytes = read(sock, buffer(s), boost::bind(read_complete, s, _1, _2));
        std::string tmp(s, bytes-1); //not to include '/n'
        std::string condt;
        std::string::iterator begin=tmp.begin();
        std::string::iterator end=tmp.end();
        //filter out the null byte
        for(;begin!=end;begin++){
            if(*begin!='\0'){
                condt.push_back(*begin);
            }           
        }
        emit showStr(QString::fromStdString(condt));
        // find termination specific symbol '\t'
        if(condt[condt.size()-1]=='\t'){
            return 0;
        }
    }
}

void MainWindow::sendKeyStat()
{

    std::string msg=keyHold;
    msg += "\n";
    sock.write_some(buffer(msg));

    char buf[1024];
    int bytes=read(sock, buffer(buf), boost::bind(read_complete, buf, _1, _2));
    std::string copy(buf, bytes-1);
    msg=msg.substr(0, msg.size()-1);
    std::cout<<"server echoed our "<< char(atoi(msg.c_str())) <<": "<<(copy==msg? "OK":"FAIL")<<"\r\n";



}

void MainWindow::readIMUdata()
{
    std::array<int, 9> imuData;
    size_t ilen=read(sock, buffer(imuData), boost::bind(len_check, 9, _1, ilen));
    std::cout<<imuData.at(0)<<std::endl;

    emit getAx(imuData.at(0));
    emit getAy(imuData.at(1));
    emit getAz(imuData.at(2));
    emit getGx(imuData.at(3));
    emit getGy(imuData.at(4));
    emit getGz(imuData.at(5));
    emit getMx(imuData.at(6));
    emit getMy(imuData.at(7));
    emit getMz(imuData.at(8));
}

void MainWindow::readYPR()
{
    std::array<float, 3> yprBuff;
    size_t len=read(sock, buffer(yprBuff), boost::bind(len_check, 3, _1, len));
    yaw=yprBuff[0];
    pitch=yprBuff[1];
    roll=yprBuff[2];
    emit getYaw(yprBuff[0]);
    emit getPitch(yprBuff[1]);
    emit getRoll(yprBuff[2]);
}

void MainWindow::readThrust()
{
    std::array<int, 4> thruBuff;
    size_t thlen=read(sock, buffer(thruBuff), boost::bind(len_check, 4, _1, thlen));
    emit getThrustFL(thruBuff[0]);
    emit getThrustFR(thruBuff[1]);
    emit getThrustRL(thruBuff[2]);
    emit getThrustRR(thruBuff[3]);
}

void MainWindow::sendKvalues()
{
    std::vector<float> kvals={float(ui->kp_H->value()), float(ui->ki_H->value()), float(ui->kd_H->value()),
                              float(ui->kp_Roll->value()), float(ui->ki_Roll->value()), float(ui->kd_Roll->value()),
                              float(ui->kp_Pitch->value()), float(ui->ki_Pitch->value()), float(ui->kd_Pitch->value()),
                              float(ui->kp_Wz->value()), float(ui->ki_Wz->value()), float(ui->kd_Wz->value()) };
    kvals.resize(12);
    write(sock, buffer(kvals), boost::asio::transfer_all());
}
/*
void MainWindow::readHeight()
{
    std::array<float, 1> h_cm;
    size_t hcmlen=read(sock, buffer(h_cm), boost::bind(len_check, 1, _1, hcmlen));
    emit getH_cm(h_cm[0]);
}
*/
#if(vid==1)
void MainWindow::recVid()
{

    boost::system::error_code err;

    for(;;){
        if(keyHold[6]=='1'){

            // vid_sock.connect(vid_ep);
            // read FPS
            if(idx[1]==0){
                std::array<char, 8> info_buf;
                size_t t=read(vid_sock, buffer(info_buf), boost::bind(len_check, 8, _1, _2));
                idx[1]++;
                std::cout<<std::string(info_buf.begin(), info_buf.end())<<"\r\n";
                fps=atoi(std::string(info_buf.begin(), info_buf.end()).c_str());
            }

            std::array<char, 16> headlen; //image size string buffer
            size_t hlen=read(vid_sock, buffer(headlen), boost::asio::transfer_all(), err);
            if(hlen!=16){
                continue;
            }
            std::cout<<"length data: "<<std::string(headlen.begin(), headlen.end())<<"\r\n";
            std::vector<uchar> vid_buff(atoi(std::string(headlen.begin(), headlen.end()).c_str()));
            size_t len=read(vid_sock, buffer(vid_buff), boost::asio::transfer_all(), err);

            frame=cv::imdecode(cv::Mat(vid_buff), CV_LOAD_IMAGE_COLOR );
            cv::flip(frame, frame, 0);
            cv::flip(frame, frame, 1);
            cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
            if(idx[3]==1){
                //cascadeBodyDetection();
                HOGPeopleDetection();
            }
            if(idx[4]==1){
                cascadeFaceDetection();
            }
            cv::cvtColor(frame, frame, CV_BGR2RGB);
            Qimg=QImage((const unsigned char*)(frame.data), frame.cols, frame.rows, QImage::Format_RGB888);
            emit showImg(QPixmap::fromImage(Qimg));
            //emit imgResz();
            //usleep(1000000/fps);
            //vid_sock.close();
        }
    }
}
#endif
void MainWindow::gyroPlot()
{
    int r0=60;
        int center_x=80, center_y=80;
        int gs=1.75;
        gyroFig = cv::Mat::zeros(160, 160, CV_8UC3);
        //gimbal
        cv::circle(gyroFig, cv::Point(center_x, center_y), r0, cv::Scalar(0,200,150), 2);
        //ball
        //cv::line(frame, cv::Point(center_x-(r0*cos( ((pitch/(30*M_PI/180))*(asin(45/r0)))+roll )), center_y-(r0*sin(((pitch/(30*M_PI/180))*(asin(45/r0)))+roll))), cv::Point(center_x+(r0*cos(((pitch/(30*M_PI/180))*(asin(45/r0)))-roll)), center_y-(r0*sin(((pitch/(30*M_PI/180))*(asin(45/r0)))-roll))), cv::Scalar(255,255,255), 2);
        //cv::line(frame, cv::Point(center_x-(r0*cos( pitch+roll )), center_y-(r0*sin(pitch+roll))), cv::Point(center_x+(r0*cos(pitch-roll)), center_y-(r0*sin(pitch-roll))), cv::Scalar(255,255,255), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( (gs*-pitch)+roll )), center_y-(r0*sin((gs*-pitch)+roll))), cv::Point(center_x+(r0*cos((gs*-pitch)-roll)), center_y-(r0*sin((gs*-pitch)-roll))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - M_PI/2 )), center_y-(r0*sin(roll - M_PI/2))), cv::Point(center_x-((r0-15)*cos(roll-M_PI/2)), center_y-((r0-15)*sin( roll-M_PI/2))), cv::Scalar(255,200,0), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 100*M_PI/180 )), center_y-(r0*sin(roll - 100*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 100*M_PI/180)), center_y-((r0-10)*sin( roll - 100*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 110*M_PI/180 )), center_y-(r0*sin(roll - 110*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 110*M_PI/180)), center_y-((r0-10)*sin( roll - 110*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 120*M_PI/180 )), center_y-(r0*sin(roll - 120*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 120*M_PI/180)), center_y-((r0-10)*sin( roll - 120*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 80*M_PI/180 )), center_y-(r0*sin(roll - 80*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 80*M_PI/180)), center_y-((r0-10)*sin( roll - 80*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 70*M_PI/180 )), center_y-(r0*sin(roll - 70*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 70*M_PI/180)), center_y-((r0-10)*sin( roll - 70*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll - 60*M_PI/180 )), center_y-(r0*sin(roll - 60*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll - 60*M_PI/180)), center_y-((r0-10)*sin( roll - 60*M_PI/180 ))), cv::Scalar(0,200,150), 2);

        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + M_PI/2 )), center_y-(r0*sin(roll + M_PI/2))), cv::Point(center_x-((r0-15)*cos(roll+M_PI/2)), center_y-((r0-15)*sin( roll+M_PI/2))), cv::Scalar(255,200,0), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 100*M_PI/180 )), center_y-(r0*sin(roll + 100*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 100*M_PI/180)), center_y-((r0-10)*sin( roll + 100*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 110*M_PI/180 )), center_y-(r0*sin(roll + 110*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 110*M_PI/180)), center_y-((r0-10)*sin( roll + 110*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 120*M_PI/180 )), center_y-(r0*sin(roll + 120*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 120*M_PI/180)), center_y-((r0-10)*sin( roll + 120*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 80*M_PI/180 )), center_y-(r0*sin(roll + 80*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 80*M_PI/180)), center_y-((r0-10)*sin( roll + 80*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::putText(gyroFig, std::string("10"), cv::Point(center_x-((r0-15)*cos(roll + 76*M_PI/180)), center_y-((r0-15)*sin( roll + 76*M_PI/180 ))), 0, 0.2, cv::Scalar(255,255,255),1);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 70*M_PI/180 )), center_y-(r0*sin(roll + 70*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 70*M_PI/180)), center_y-((r0-10)*sin( roll + 70*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::putText(gyroFig, std::string("20"), cv::Point(center_x-((r0-15)*cos(roll + 66*M_PI/180)), center_y-((r0-15)*sin( roll + 66*M_PI/180 ))), 0, 0.2, cv::Scalar(255,255,255),1);
        cv::line(gyroFig, cv::Point(center_x-(r0*cos( roll + 60*M_PI/180 )), center_y-(r0*sin(roll + 60*M_PI/180))), cv::Point(center_x-((r0-10)*cos(roll + 60*M_PI/180)), center_y-((r0-10)*sin( roll + 60*M_PI/180 ))), cv::Scalar(0,200,150), 2);
        cv::putText(gyroFig, std::string("30"), cv::Point(center_x-((r0-15)*cos(roll + 56*M_PI/180)), center_y-((r0-15)*sin( roll + 56*M_PI/180 ))), 0, 0.2, cv::Scalar(255,255,255),1);

        cv::line(gyroFig, cv::Point(center_x-10, center_y- (r0*sin(gs*10*M_PI/180)) ), cv::Point(center_x+10, center_y-(r0*sin(gs*10*M_PI/180)) ), cv::Scalar(255,255,255), 1);
        cv::putText(gyroFig, std::string("10"), cv::Point(center_x+14, center_y+(r0*sin(gs*10*M_PI/180)-2) ), 0, 0.2, cv::Scalar(255,255,255),1);
        cv::line(gyroFig, cv::Point(center_x-12, center_y- (r0*sin(gs*20*M_PI/180)) ), cv::Point(center_x+12, center_y-(r0*sin(gs*20*M_PI/180)) ), cv::Scalar(255,255,255), 1);
        cv::putText(gyroFig, std::string("20"), cv::Point(center_x+16, center_y+(r0*sin(gs*20*M_PI/180)-2) ), 0, 0.2, cv::Scalar(255,255,255),1);
        cv::line(gyroFig, cv::Point(center_x-16, center_y- (r0*sin(gs*30*M_PI/180)) ), cv::Point(center_x+15, center_y-(r0*sin(gs*30*M_PI/180)) ), cv::Scalar(255,255,255), 1);
        cv::putText(gyroFig, std::string("30"), cv::Point(center_x+19, center_y+(r0*sin(gs*30*M_PI/180)-2) ), 0, 0.2, cv::Scalar(255,255,255),1);

        cv::line(gyroFig, cv::Point(center_x-10, center_y+ (r0*sin(gs*10*M_PI/180)) ), cv::Point(center_x+10, center_y+(r0*sin(gs*10*M_PI/180)) ), cv::Scalar(255,255,255), 1);
        cv::line(gyroFig, cv::Point(center_x-12, center_y+ (r0*sin(gs*20*M_PI/180)) ), cv::Point(center_x+12, center_y+(r0*sin(gs*20*M_PI/180)) ), cv::Scalar(255,255,255), 1);
        cv::line(gyroFig, cv::Point(center_x-16, center_y+ (r0*sin(gs*30*M_PI/180)) ), cv::Point(center_x+15, center_y+(r0*sin(gs*30*M_PI/180)) ), cv::Scalar(255,255,255), 1);

        cv::line(gyroFig, cv::Point(center_x-r0/2, center_y), cv::Point(center_x+r0/2, center_y), cv::Scalar(255,200,0), 2);
        cv::line(gyroFig, cv::Point(center_x, center_y-r0/2), cv::Point(center_x, center_y+r0/2), cv::Scalar(255,200,0), 2);

        cv::cvtColor(gyroFig, gyroFig, CV_BGR2RGB);
        QimgGyro = QImage((const unsigned char*)(gyroFig.data), gyroFig.cols, gyroFig.rows, QImage::Format_RGB888);
        emit showGyro(QPixmap::fromImage(QimgGyro));

        //
}

void MainWindow::loadSettings()
{
    int c=0, pos=0;
    double kvalue[12];
    std::fstream log("/Users/Peter/Desktop/GUIClient/log.txt", std::ios_base::in);
    while (getline(log, strtmp, ' ')) {
        if(strtmp[0]!='#'){ strs.push_back(strtmp);}
    }
    for(int j=0; j<strs.size(); j++){
        convertFromStr(strs.at(j), kvalue[j]);
    }
    ui->kp_H->setValue(kvalue[0]);
    ui->ki_H->setValue(kvalue[1]);
    ui->kd_H->setValue(kvalue[2]);
    ui->kp_Roll->setValue(kvalue[3]);
    ui->ki_Roll->setValue(kvalue[4]);
    ui->kd_Roll->setValue(kvalue[5]);
    ui->kp_Pitch->setValue(kvalue[6]);
    ui->ki_Pitch->setValue(kvalue[7]);
    ui->kd_Pitch->setValue(kvalue[8]);
    ui->kp_Wz->setValue(kvalue[9]);
    ui->ki_Wz->setValue(kvalue[10]);
    ui->kd_Wz->setValue(kvalue[11]);
    log.close();
}

void MainWindow::cascadeBodyDetection()
{
    std::vector<cv::Rect> bodies;
    cv::Mat frame_gray_equ;
    std::vector<cv::Point> contour;
    cv::equalizeHist(frame_gray, frame_gray_equ);

    body_cascade.detectMultiScale(frame_gray_equ, bodies, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    for(size_t i=0; i<bodies.size(); i++){
        /*
        cv::Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse(frame, center, cv::Size(faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
         */
        cv::rectangle(frame, bodies[i], cv::Scalar(0, 200, 255), 2);
        std::stringstream ss;
        ss<<i+1;
        cv::putText(frame, ss.str(),  cv::Point(bodies[i].x+bodies[i].width+5, bodies[i].y), 2, 0.5, cv::Scalar(30, 200, 255), 1);
        //printf("plot on body, body size %d.........\n", bodies.size());
    }
}

void MainWindow::cascadeFaceDetection()
{
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray_equ;
    std::vector<cv::Point> contour;
    cv::equalizeHist(frame_gray, frame_gray_equ);

    face_cascade.detectMultiScale(frame_gray_equ, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    for(size_t i=0; i<faces.size(); i++){
        /* diamond shape
        contour.push_back(cv::Point(faces[i].x + faces[i].width/2, faces[i].y - faces[i].height/4)); //top point
        contour.push_back(cv::Point(faces[i].x - faces[i].width/4, faces[i].y + faces[i].height/2)); //left point
        contour.push_back(cv::Point(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height + faces[i].height/4)); //bottom point
        contour.push_back(cv::Point(faces[i].x + faces[i].width + faces[i].width/4, faces[i].y+faces[i].height/2)); //right point
        cv::polylines(frame, contour, true, cv::Scalar(0, 255, 0), 3);
        */
        /*
        cv::Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse(frame, center, cv::Size(faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
         */
        cv::rectangle(frame, faces[i], cv::Scalar(0, 255, 0), 2);
        std::stringstream ss;
        ss<<i+1;
        cv::putText(frame, ss.str(),  cv::Point(faces[i].x+faces[i].width+5, faces[i].y), 2, 0.5, cv::Scalar(30, 200, 0), 1);
        //printf("plot on face, face size %d.........\n", faces.size());

        /* eye detection
        cv::Mat faceROI = frame_gray(faces[i]);
        std::vector<cv::Rect> eyes;

        eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

        for(size_t j=0; j<eyes.size(); j++){

            cv::Point eye_center(faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2);
            int radius = cvRound(0.25*(eyes[j].width + eyes[j].height));
            cv::circle(frame, eye_center, radius, cv::Scalar(255, 0, 0), 4, 8, 0);
            printf("plot eyes, eye number %d\n\n\n", eyes.size());
        }
        */
    }
}

void MainWindow::HOGPeopleDetection()
{
    std::vector<cv::Rect> peoples;
    cv::HOGDescriptor peopleDetector;
    peopleDetector.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    peopleDetector.detectMultiScale(frame_gray, peoples, 0,
                                    cv::Size(4, 4), cv::Size(0, 0), 1.1, 2);
    for(int i=0; i<peoples.size(); i++){
        cv::rectangle(frame, peoples[i], cv::Scalar(0, 200, 255), 2);
        std::stringstream ss;
        ss<<i+1;
        cv::putText(frame, ss.str(),  cv::Point(peoples[i].x+peoples[i].width+5, peoples[i].y), 2, 0.5, cv::Scalar(30, 200, 255), 1);
    }
}

void MainWindow::labResz()
{
    ui->vidLabel->resize(ui->vidLabel->pixmap()->size());
}

void MainWindow::setYaw(float y)
{
    //std::stringstream strs;
    //strs<<y*180/M_PI;
    //std::string str=strs.str();
    ui->pgBar_yaw->setValue(int(y*180/M_PI));
    ui->label_ang_yaw->setText(QString::number(y*180/M_PI, 'f', 4));
}

void MainWindow::setPitch(float p)
{
    //std::stringstream strs;
    //strs<<p*180/M_PI;
    //std::string str=strs.str();
    ui->pgBar_pitch->setValue(p*180/M_PI);
    ui->label_ang_pitch->setText(QString::number(p*180/M_PI, 'f', 4));
}

void MainWindow::setRoll(float r)
{
    //std::stringstream strs;
    //strs<<r*180/M_PI;
    //std::string str=strs.str();
    ui->pgBar_roll->setValue(r*180/M_PI);
    ui->label_ang_roll->setText(QString::number(r*180/M_PI, 'f', 4));
}

void MainWindow::setText1()
{
    if(ui->connectButton->text()=="Connect"){
        ui->connectButton->setText("Quit");
    }else if(ui->connectButton->text()=="Quit"){
        ui->connectButton->setText("Connect");
    }
}

void MainWindow::setText2()
{
    if(ui->vidButton->text()=="Receive Video"){
        ui->vidButton->setText("Stop Video stream");
        keyHold[6]='1';
    }else if(ui->vidButton->text()=="Stop Video stream"){
        ui->vidButton->setText("Receive Video");
        keyHold[6]='0';
    }
}

void MainWindow::sockConnection()
{
    sock.connect(ep);
    vid_sock.connect(vid_ep);
    idx[0]= (idx[0]==1)? 0:1;
}

void MainWindow::setLock()
{
    if(idx[2]==0){
        ui->kp_H->setEnabled(false);
        ui->ki_H->setEnabled(false);
        ui->kd_H->setEnabled(false);
        ui->kp_Roll->setEnabled(false);
        ui->ki_Roll->setEnabled(false);
        ui->kd_Roll->setEnabled(false);
        ui->kp_Pitch->setEnabled(false);
        ui->ki_Pitch->setEnabled(false);
        ui->kd_Pitch->setEnabled(false);
        ui->kp_Wz->setEnabled(false);
        ui->ki_Wz->setEnabled(false);
        ui->kd_Wz->setEnabled(false);
        ui->lockButton->setText("unlock");

    }else if(idx[2]==1){
        ui->kp_H->setEnabled(true);
        ui->ki_H->setEnabled(true);
        ui->kd_H->setEnabled(true);
        ui->kp_Roll->setEnabled(true);
        ui->ki_Roll->setEnabled(true);
        ui->kd_Roll->setEnabled(true);
        ui->kp_Pitch->setEnabled(true);
        ui->ki_Pitch->setEnabled(true);
        ui->kd_Pitch->setEnabled(true);
        ui->kp_Wz->setEnabled(true);
        ui->ki_Wz->setEnabled(true);
        ui->kd_Wz->setEnabled(true);
        ui->lockButton->setText("lock");
    }

    idx[2] = (idx[2] == 1)? 0:1;
}

void MainWindow::saveSettings()
{
    std::fstream log("/Users/Peter/Desktop/GUIClient/log.txt", std::ios::out);
    std::stringstream s1, s2, s3, s4;
    s1<<"#h "<<ui->kp_H->value()<<" "<<ui->ki_H->value()<<" "<<ui->kd_H->value()<<" #\n";
    s2<<"#roll "<<ui->kp_Roll->value()<<" "<<ui->ki_Roll->value()<<" "<<ui->kd_Roll->value()<<" #\n";
    s3<<"#pitch "<<ui->kp_Pitch->value()<<" "<<ui->ki_Pitch->value()<<" "<<ui->kd_Pitch->value()<<" #\n";
    s4<<"#yaw "<<ui->kp_Wz->value()<<" "<<ui->ki_Wz->value()<<" "<<ui->kd_Wz->value()<<" #\n";
    log<<s1.str();
    log<<s2.str();
    log<<s3.str();
    log<<s4.str();
    log.close();
}

void MainWindow::detln()
{
    ui->textBrowser->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
    ui->textBrowser->moveCursor(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
    ui->textBrowser->moveCursor(QTextCursor::End, QTextCursor::KeepAnchor);
    ui->textBrowser->textCursor().removeSelectedText();
    ui->textBrowser->textCursor().deleteChar();
    ui->textBrowser->moveCursor(QTextCursor::PreviousRow);
}

void MainWindow::setAx(int v)
{
    double dv=(v*9.81)/8192;  //+-4g
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_ax->setFormat(QString::fromStdString(str));
    ui->label_ax->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setAy(int v)
{
    double dv=(v*9.81)/8192;
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_ay->setFormat(QString::fromStdString(str));
    ui->label_ay->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setAz(int v)
{
    double dv=(v*9.81)/8192;
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_az->setFormat(QString::fromStdString(str));
    ui->label_az->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setGx(int v)
{
    double dv=v/65.536f; //+-500 deg/s
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_gx->setFormat(QString::fromStdString(str));
    ui->label_gx->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setGy(int v)
{
    double dv=v/65.536f; //+-500 deg/s
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_gy->setFormat(QString::fromStdString(str));
    ui->label_gy->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setGz(int v)
{
    double dv=v/65.536f; //+-500 deg/s
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_gz->setFormat(QString::fromStdString(str));
    ui->label_gz->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setMx(int v)
{
    double dv=v*8192/2400; //+-1200 uT, 13-bit, 2^13=8192
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_mx->setFormat(QString::fromStdString(str));
    ui->label_mx->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setMy(int v)
{
    double dv=v*8192/2400; //+-1200 uT, 13-bit, 2^13=8192
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_my->setFormat(QString::fromStdString(str));
    ui->label_my->setText(QString::number(dv, 'f', 4));
}

void MainWindow::setMz(int v)
{
    double dv=v*8192/2400; //+-1200 uT, 13-bit, 2^13=8192
    //std::stringstream strs;
    //strs<<dv;
    //std::string str=strs.str();
    //ui->pgBar_mz->setFormat(QString::fromStdString(str));
    ui->label_mz->setText(QString::number(dv, 'f', 4));
}



void MainWindow::on_faceButton_clicked()
{
    idx[4] = (idx[4]==1)? 0:1;
    switch (idx[4]) {
    case 0:
        ui->textBrowser->append("face detection : OFF");
        break;
    case 1:
        ui->textBrowser->append("face detection : ON");
        break;
    }
}

void MainWindow::on_bodyButton_clicked()
{
    idx[3] = (idx[3]==1)? 0:1;
    switch (idx[3]) {
    case 0:
        ui->textBrowser->append("body detection : OFF");
        break;
    case 1:
        ui->textBrowser->append("body detection : ON");
        break;
    }
}
