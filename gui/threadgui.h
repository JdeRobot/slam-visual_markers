#ifndef THREADGUI_H
#define THREADGUI_H

#include <iostream>
#include <sys/time.h>
#include "mainwindow.h"
#include "sensors.h"
#include <ros/ros.h>


#define cycle_gui 70 //miliseconds

class threadGUI : public QThread
{
public:
    threadGUI(Sensors* sensors,int option, std::string topic_pose,std::string topic_timer,std::string topic_numMarker ,std::string calib_filename);

private:
    MainWindow* gui;
    Sensors* sensors;
	int option;
	std::string topic;

protected:
    void run();
};

#endif // THREADGUI_H
