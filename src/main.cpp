#include <QApplication>
#include "mainwindow.h"
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

	ros::init(argc, argv, "visual_marker");
		
	
    try
    {	
		//-----------------Comm----------------//
		Config::Properties props = Config::load(argc, argv);	
        Comm::Communicator* jdrc = new Comm::Communicator(props);

		int serverPose=props.asInt("VisualMarkers.Pose3D.Server");
		std::string topic_pose = props.asString("VisualMarkers.Pose3D.Topic");
		std::string topic_numMarker = props.asString("VisualMarkers.NumMarker.Topic");
		std::string topic_Timer = props.asString("VisualMarkers.Timer.Topic");
		std::string calib_filename = argv[2];
		ros::Publisher pub;

		
	
		


        Sensors* sensors = new Sensors(jdrc);
        threadGUI* gui = new threadGUI(sensors,serverPose,topic_pose,topic_Timer,topic_numMarker,calib_filename);
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
