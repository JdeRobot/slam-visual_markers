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

//COMM
#include "jderobot/comm/communicator.hpp"
#include <jderobot/comm/cameraClient.hpp>


#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visionlib/colorspaces/colorspacesmm.h>

#include "jderobot/comm/communicator.hpp"
#include <jderobot/comm/laserClient.hpp>
#include <jderobot/comm/cameraClient.hpp>
#include <jderobot/comm/pose3dClient.hpp>


#define LINX 0.3
#define LINY 0.3
#define LINZ 0.8
#define ANGZ 1.0
#define ANGY 0.0
#define ANGX 0.0

class Sensors
{
    public:

		Sensors(Comm::Communicator* jdrc);
        virtual ~Sensors();
        cv::Mat getImage();
    	JdeRobotTypes::Image getImage1();

        void update();

//		jderobot::Navdata getNavdata();
        jderobot::Pose3DDataPtr getPose3DData();
        void takeOff();
        void land();
        void reset();
        void toggleCam();
        void sendVelocitiesToUAV(float vx,float vy,float vz,float roll,float pitch,float yaw);

    private:

		Comm::Communicator* jdrc;
		Comm::CameraClient* camera;


        QMutex mutex;
        QMutex mutexDrone;
        cv::Mat image;
        //jderobot::Navdata navdata;


        //jderobot::ArDronePrx adprx;
        //jderobot::QuadrotorPrx quadprx;
        jderobot::Pose3DPrx p3dprx;
        jderobot::Pose3DDataPtr pose3DDataPtr;
        bool tracking;
        bool flying;
        bool rst;

};

#endif // SENSORS_H
