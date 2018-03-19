#include "sensors.h"
#include <iostream>


Sensors::Sensors(Comm::Communicator* jdrc)
{
	this-> jdrc = jdrc;
	this-> camera = Comm::getCameraClient(jdrc,"CamAutoloc.Camera");


}

Sensors::~Sensors(){

}


void Sensors::update()
{	
	JdeRobotTypes::Image data;
    mutex.lock();
	data =  this->camera->getImage();
    
    image.create(data.height, data.width, CV_8UC3);
        //memcpy((unsigned char *) image.data ,&(data->pixelData[0]), image.cols*image.rows * 3);
    mutex.unlock();


    mutexDrone.lock();
//		navdata=adprx->getNavdata();
        //pose3DDataPtr = p3dprx->getPose3DData();
    mutexDrone.unlock();

}

cv::Mat Sensors::getImage()
{
	JdeRobotTypes::Image img;

   
    if (this->camera){
    	img = this->camera->getImage();
    }

    return img.data;
    
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
