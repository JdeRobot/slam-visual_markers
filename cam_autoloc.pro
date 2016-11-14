#-------------------------------------------------
#
# Project created by QtCreator 2014-10-18T12:26:33
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = cam_autoloc
TEMPLATE = app

QMAKE_CFLAGS += -std=c99
#QMAKE_LFLAGS += -std=c99
#CONFIG += -std=c99
CONFIG += no_keywords

SOURCES += main.cpp\
        mainwindow.cpp \
    pose3di.cpp \
    threadgui.cpp \
    sensors.cpp \
    threadsensors.cpp \
    april_tags/matd.c \
    april_tags/zhash.c \
    april_tags/zarray.c \
    april_tags/workerpool.c \
    april_tags/unionfind.c \
    april_tags/tag36h11.c \
    april_tags/tag36h10.c \
    april_tags/segment2.c \
    april_tags/image_u32.c \
    april_tags/image_u8.c \
    april_tags/image_f32.c \
    april_tags/homography.c \
    april_tags/graymodel.c \
    april_tags/g2d.c \
    april_tags/apriltag.c \
    world.cpp \
    drawworld.cpp \
    geometryutils.cpp \
    markerinfo.cpp \
    drawingutils.cpp \
    cameramanager.cpp \
    kalmanfilter.cpp \
    weightedaveragefilter.cpp \
    AprilTagsSrc/Edge.cc \
    AprilTagsSrc/UnionFindSimple.cc \
    AprilTagsSrc/TagFamily.cc \
    AprilTagsSrc/TagDetector.cc \
    AprilTagsSrc/TagDetection.cc \
    AprilTagsSrc/Segment.cc \
    AprilTagsSrc/Quad.cc \
    AprilTagsSrc/MathUtil.cc \
    AprilTagsSrc/Homography33.cc \
    AprilTagsSrc/GrayModel.cc \
    AprilTagsSrc/GLineSegment2D.cc \
    AprilTagsSrc/GLine2D.cc \
    AprilTagsSrc/Gaussian.cc \
    AprilTagsSrc/FloatImage.cc \
    sharer.cpp

HEADERS  += mainwindow.h \
    threadgui.h \
    sensors.h \
    threadsensors.h \
    april_tags/apriltag.h \
    april_tags/zhash.h \
    april_tags/zarray.h \
    april_tags/workerpool.h \
    april_tags/unionfind.h \
    april_tags/timeprofile.h \
    april_tags/tag36h11.h \
    april_tags/tag36h10.h \
    april_tags/matd.h \
    april_tags/line_fit.h \
    april_tags/image_u32.h \
    april_tags/image_u8.h \
    april_tags/image_f32.h \
    april_tags/homography.h \
    april_tags/gridder.h \
    april_tags/graymodel.h \
    april_tags/common.h \
    AprilTags/Edge.h \
    world.h \
    drawworld.h \
    IGraphichAlgorithmer.h \
    geometryutils.h \
    markerinfo.h \
    drawingutils.h \
    cameramanager.h \
    kalmanfilter.h \
    ardronedefines.h \
    weightedaveragefilter.h \
    AprilTags/XYWeight.h \
    AprilTags/UnionFindSimple.h \
    AprilTags/TagFamily.h \
    AprilTags/TagDetector.h \
    AprilTags/TagDetection.h \
    AprilTags/Tag36h11_other.h \
    AprilTags/Tag36h11.h \
    AprilTags/Tag36h9.h \
    AprilTags/Tag25h9.h \
    AprilTags/Tag25h7.h \
    AprilTags/Tag16h5_other.h \
    AprilTags/Tag16h5.h \
    AprilTags/Segment.h \
    AprilTags/Quad.h \
    AprilTags/pch.h \
    AprilTags/MathUtil.h \
    AprilTags/Homography33.h \
    AprilTags/Gridder.h \
    AprilTags/GrayModel.h \
    AprilTags/GLineSegment2D.h \
    AprilTags/GLine2D.h \
    AprilTags/Gaussian.h \
    AprilTags/FloatImage.h \
    pose3di.h \
    sharer.h

FORMS    += mainwindow.ui

LIBS += -L/usr/lib/ -lIce \
                    -lIceUtil \
                    -lGL \
                    -lgtkglextmm-x11-1.2 \
                    -lglibmm-2.4 \
                    -lgtkmm-2.4 \
                    -lgnomecanvasmm-2.6 \
                    -lglademm-2.4 \
                    -lgdkglextmm-x11-1.2 \
                    -latkmm-1.6 \
                    -lsigc-2.0 \
                    -lgdkmm-2.4 \
                    -lGL \
                    -lglut \
                    -lGLU \
                    -lprogeo

LIBS += -L/usr/lib/x86_64-linux-gnu   -lglib-2.0 \
                                      -lopencv_core \
                                      -lopencv_imgproc \
                                      -lopencv_highgui \
                                      -lopencv_calib3d \
                                      -laruco

LIBS += -L/usr/local/lib -laruco

LIBS += -L/usr/local/lib/jderobot/ -lJderobotInterfaces \
                                   -lcolorspaces \
                                   -lcolorspacesmm

INCLUDEPATH += /usr/include \
                /usr/local/include \
                /usr/local/include/jderobot \
                /usr/include/GL \
                /usr/include/gtkglextmm-1.2 \
                /usr/lib/gtkglextmm-1.2/include \
                /usr/lib/x86_64-linux-gnu/gtkglextmm-1.2 \
                /usr/lib/gtkglext-1.0/include \
                /usr/lib/x86_64-linux-gnu/glibmm-2.4/include \
                /usr/include/glibmm-2.4 \
                /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include \
                /usr/include/sigc++-2.0 \
                /usr/lib/x86_64-linux-gnu/sigc++-2.0/include \
                /usr/include/gdkmm-2.4 \
                /usr/include/gtk-2.0 \
                /usr/include/cairo \
                /usr/include/pango-1.0 \
                /usr/lib/x86_64-linux-gnu/gtk-2.0/include \
                /usr/include/gdk-pixbuf-2.0 \
                /usr/lib/x86_64-linux-gnu/gdkmm-2.4/include \
                /usr/lib/x86_64-linux-gnu/pangomm-1.4/include \
                /usr/lib/x86_64-linux-gnu/cairomm-1.0/include \
                /usr/include/cairomm-1.0 \
                /usr/include/freetype2 \
                /usr/include/pangomm-1.4 \
                /usr/include/giomm-2.4 \
                /usr/include/gtkglext-1.0 \
                /usr/include/gtkmm-2.4 \
                /usr/lib/x86_64-linux-gnu/gtkmm-2.4/include \
                /usr/include/atkmm-1.6 \
                /usr/include/atk-1.0 \
                /usr/include/libglademm-2.4 \
                /usr/include/libgnomecanvasmm-2.6 \
                /usr/include/libgnomecanvas-2.0 \
                /usr/include/libart-2.0 \
                AprilTags
