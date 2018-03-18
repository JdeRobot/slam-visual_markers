#include <QApplication>
#include "mainwindow.h"
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include "threadgui.h"
#include "sensors.h"
#include "threadsensors.h"
#include "pose3di.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gtkmm.h>
#include <gtkglmm.h>
#include <gdkglmm.h>
#include <iostream>

#include "jderobot/config/config.h"
#include "jderobot/comm/communicator.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char *argv[])
{

	if (argc<3)
	{
	std::cout<<"Parameter error:\nUSAGE:\tslam_markers path_to_config.yml path_to_calib_file.yml\n The file markers.txt and .glade must be in the folder"<<std::endl;
	exit(-1);
	}

    QApplication a(argc, argv);


		
	
    try
    {	
		//-----------------Comm----------------//
		Config::Properties props = Config::load(argc, argv);	
        Comm::Communicator* jdrc = new Comm::Communicator(props);

		int serverPose=props.asInt("CamAutoloc.Pose3D.Server");
		std::string topic = props.asString("CamAutoloc.Pose3D.Topic");
		std::string calib_filename = argv[2];
		ros::Publisher pub;

		if (serverPose == 1)
		{
			// Publish Pose3d with ROS

			std::string endpoint = props.asString("CamAutoloc.Pose3D.Proxy");		
			std::cout<<"You have selected ICE to publish the estimated pose:\n"<<endpoint<<std::endl;
			Ice::CommunicatorPtr ic;
			ic = Ice::initialize(argc, argv);

		    Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("Pose3D", endpoint);
		    Ice::ObjectPtr object = new Pose3DI();
		    adapter->add(object, ic->stringToIdentity("Pose3D"));
		    adapter->activate();
		


		}
		else
		{
			ros::init(argc, argv, "slam_markers");
		}
		


        Sensors* sensors = new Sensors(jdrc);
        threadGUI* gui = new threadGUI(sensors,serverPose,topic,calib_filename);
        ThreadSensors* threadSensors = new ThreadSensors(sensors);

        threadSensors->start();
        gui->start();
		


	
    }
    catch(const Ice::Exception& ex)
    {
        std::cerr << ex << std::endl;
        exit(-1);
    }
    catch (const char* msg)
    {
        std::cerr << msg << std::endl;
        exit(-1);
    }


    return a.exec();
}
