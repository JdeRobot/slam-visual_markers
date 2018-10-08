#!/bin/bash


#### Installation deps
## Aruco 2
mkdir aruco_lib;cd aruco_lib;
wget https://sourceforge.net/projects/aruco/files/2.0.19/aruco-2.0.19.zip 
unzip aruco-2.0.19.zip
cd aruco-2.0.19; mkdir build; cd build;
cmake ..
make
make install
cd ../../..
## Jderobot-deps
apt install jderobot-deps-dev -y

# Installation componenent
mkdir /opt/jderobot/include/jderobot/jderobotutil/utils;
cp /opt/jderobot/include/jderobot/jderobotutil/CameraUtils.h /opt/jderobot/include/jderobot/jderobotutil/utils/
rm -rf build; mkdir build; cd build



cmake .. \
-DJderobotInterfaces=/opt/jderobot/lib/libJderobotInterfaces.so \
-Dcomm=/opt/jderobot/lib/libcomm.so \
-Djderobotutil=/opt/jderobot/lib/libjderobotutil.so \
-DparallelIce=/opt/jderobot/lib/libparallelIce.so \
-Dcolorspacesmm=/opt/jderobot/lib/libcolorspacesmm.so \
-DxmlParser=/opt/jderobot/lib/libxmlParser.so \
-Dprogeo=/opt/jderobot/lib/libprogeo.so \
-Dconfig=/opt/jderobot/lib/libconfig.so \
-Dlogger=/opt/jderobot/lib/liblogger.so \
-DlibjderobotHandlers=/opt/jderobot/lib/libjderobotHandlers.so \
-DGlog=/usr/lib/x86_64-linux-gnu/libglog.so \
-DlibIceStorm=/usr/lib/x86_64-linux-gnu/libIceStorm++11.so.36  
make
