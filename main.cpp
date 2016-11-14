#include <QtWidgets/QApplication>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
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

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Ice::CommunicatorPtr ic;
    try
    {
        ic = Ice::initialize(argc, argv);

        Sensors* sensors = new Sensors(ic);
        threadGUI* gui = new threadGUI(sensors);
        ThreadSensors* threadSensors = new ThreadSensors(sensors);

        threadSensors->start();
        gui->start();
        Ice::PropertiesPtr prop = ic->getProperties();
        std::string endpoint = prop->getProperty("UavViewer.Pose3D.Proxy");
        Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("Pose3d", endpoint);
        Ice::ObjectPtr object = new Pose3DI();
        adapter->add(object, ic->stringToIdentity("Pose3d"));
        adapter->activate();
        //ic->waitForShutdown();
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
