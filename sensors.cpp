#include "sensors.h"

Sensors::Sensors(Ice::CommunicatorPtr ic)
{
    this-> ic = ic;
    Ice::PropertiesPtr prop = ic->getProperties();
    Ice::PropertiesPtr prop3 = ic->getProperties();
    //Camera
    Ice::ObjectPrx base = ic->propertyToProxy("UavViewer.Camera.Proxy");
    if (0==base)
        throw "Could not create ventral camera proxy";

    /*cast to CameraPrx*/
    cprx = jderobot::CameraPrx::checkedCast(base);
    if (0==cprx)
        throw "Invalid ventral camera proxy";

    jderobot::ImageDataPtr data = cprx->getImageData(colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);

    //ArDrone
/*	Ice::ObjectPrx baseDrone = ic->stringToProxy("ArDroneControl:default -p 9998");
    if (baseDrone==0)
        throw "Could not create proxy";

    /*cast to ArDrone*/
/*	adprx = jderobot::ArDronePrx::checkedCast(baseDrone);
    if (adprx==0)
        throw "Invalid proxy";
*/
    //ArDrone
//    Ice::ObjectPrx baseDrone = ic->propertyToProxy("UavViewer.ArDroneControl.Proxy");
//    if (baseDrone==0)
//        throw "Could not create control proxy";

    /*cast to ArDrone*/
//    quadprx = jderobot::QuadrotorPrx::checkedCast(baseDrone);
//    if (quadprx==0)
//        throw "Invalid control proxy";


//    //IMU
//    Ice::ObjectPrx baseImu = ic->propertyToProxy("UavViewer.Imu.Proxy");
//    if (baseImu==0)
//        throw "Could not create imu proxy";

//    p3dprx = jderobot::Pose3DPrx::checkedCast(baseImu);
//    if (p3dprx==0)
//        throw "Invalid imu proxy";
//    pose3DDataPtr = p3dprx->getPose3DData();

    this->tracking=false;
    this->flying=false;
    this->rst=false;

}

Sensors::~Sensors(){

}

//void Sensors::sendVelocitiesToUAV(float vx,float vy,float vz,float roll,float pitch,float yaw)
//{
//    mutexDrone.lock();
//        jderobot::Velocities vel;
//        vel.linear.x=vx;
//        vel.linear.y=vy;
//        vel.linear.z=vz;
//        vel.angular.z=yaw;
//        vel.angular.x=roll;
//        vel.angular.y=pitch;

//        //adprx->cmdVel(vel);
//        quadprx->cmdVel(vel);
//    mutexDrone.unlock();
//}

void Sensors::update()
{
    mutex.lock();
        jderobot::ImageDataPtr data = cprx->getImageData(colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);
        image.create(data->description->height, data->description->width, CV_8UC3);
        memcpy((unsigned char *) image.data ,&(data->pixelData[0]), image.cols*image.rows * 3);
    mutex.unlock();
    mutexDrone.lock();
//		navdata=adprx->getNavdata();
        //pose3DDataPtr = p3dprx->getPose3DData();
    mutexDrone.unlock();

}

cv::Mat Sensors::getImage()
{
    mutex.lock();
        cv::Mat result = image.clone();
    mutex.unlock();
    return result;
}


//void Sensors::takeOff(){
//    mutexDrone.lock();
//        if(!rst){
//            //adprx->takeoff();
//            quadprx->takeoff();
//            this->flying=true;
//            std::cout << "takeoff"<<std::endl;
//        }
//    mutexDrone.unlock();
//}

//void Sensors::land(){
//    mutexDrone.lock();
//        if(!rst){
//            jderobot::Velocities cmd;
//            cmd.linear.x=cmd.linear.y=cmd.linear.z=0;
//            cmd.angular.x=cmd.angular.y=cmd.angular.z=0;
//            //adprx->cmdVel(cmd);
//            //adprx->land();
//            quadprx->land();
//            this->flying=false;
//            this->tracking=false;
//            std::cout << "land"<<std::endl;
//        }
//    mutexDrone.unlock();
//}

//void Sensors::reset(){
//    mutexDrone.lock();
//        quadprx->reset();
//        this->flying=false;
//        this->tracking=false;
//        this->rst=!this->rst;
//    mutexDrone.unlock();
//}

//void Sensors::toggleCam(){
//    mutex.lock();
//        quadprx->toggleCam();
//    mutex.unlock();
//}

jderobot::Pose3DDataPtr Sensors::getPose3DData(){
    jderobot::Pose3DDataPtr tmp = new jderobot::Pose3DData();
    mutexDrone.lock();
        tmp->x=pose3DDataPtr->x;
        tmp->y=pose3DDataPtr->y;
        tmp->z=pose3DDataPtr->z;
        tmp->h=pose3DDataPtr->h;
        tmp->q0=pose3DDataPtr->q0;
        tmp->q1=pose3DDataPtr->q1;
        tmp->q2=pose3DDataPtr->q2;
        tmp->q3=pose3DDataPtr->q3;
    mutexDrone.unlock();
    return tmp;
}

/*jderobot::Navdata Sensors::getNavdata(){
    jderobot::Navdata tmp;
    mutexDrone.lock();
        tmp=navdata;
    mutexDrone.unlock();
    return tmp;
}*/
