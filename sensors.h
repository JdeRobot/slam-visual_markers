#ifndef SENSORS_H
#define SENSORS_H

//ICE
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
//QT
#include <QtGui>
//INTERFACES
#include <jderobot/camera.h>
//#include "../src/ArDrone.h"
#include <jderobot/pose3d.h>
//#include <quadrotor.h>

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visionlib/colorspaces/colorspacesmm.h>

#define LINX 0.3
#define LINY 0.3
#define LINZ 0.8
#define ANGZ 1.0
#define ANGY 0.0
#define ANGX 0.0

class Sensors
{
    public:
        Sensors(Ice::CommunicatorPtr ic);
        virtual ~Sensors();
        cv::Mat getImage();

        void update();

//		jderobot::Navdata getNavdata();
        jderobot::Pose3DDataPtr getPose3DData();
        void takeOff();
        void land();
        void reset();
        void toggleCam();
        void sendVelocitiesToUAV(float vx,float vy,float vz,float roll,float pitch,float yaw);

    private:
        QMutex mutex;
        QMutex mutexDrone;
        cv::Mat image;
        //jderobot::Navdata navdata;
        Ice::CommunicatorPtr ic;
        jderobot::CameraPrx cprx;
        //jderobot::ArDronePrx adprx;
        //jderobot::QuadrotorPrx quadprx;
        jderobot::Pose3DPrx p3dprx;
        jderobot::Pose3DDataPtr pose3DDataPtr;
        bool tracking;
        bool flying;
        bool rst;

};

#endif // SENSORS_H
