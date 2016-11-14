#ifndef THREADGUI_H
#define THREADGUI_H

#include <iostream>
#include <sys/time.h>
#include "mainwindow.h"
#include "sensors.h"

#define cycle_gui 166 //miliseconds

class threadGUI : public QThread
{
public:
    threadGUI(Sensors* sensors);

private:
    MainWindow* gui;
    Sensors* sensors;

protected:
    void run();
};

#endif // THREADGUI_H
