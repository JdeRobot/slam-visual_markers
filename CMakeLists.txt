project( visual_markers )
cmake_minimum_required( VERSION 2.8.7 )



set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(QMAKE_CXXFLAGS "-std=c++11")

#required modules
#find_package(Qt5Widgets REQUIRED)
find_package(PkgConfig)

#Version
pkg_check_modules(GTKMM gtkmm-2.4)  
find_package(Qt5Widgets REQUIRED) 
pkg_check_modules(GTKGLEXTMM gtkglextmm-1.2) 
pkg_check_modules(LIBGLADEMM libglademm-2.4)
pkg_check_modules(LIBGNOMECANVASMM libgnomecanvasmm-2.6)
find_package(OpenCV REQUIRED)
find_package( ZLIB REQUIRED )
find_package(catkin REQUIRED 
COMPONENTS
roscpp
geometry_msgs)

#find_package(OpenGL REQUIRED) 
find_package(GLUT REQUIRED)
#find_package(Ice REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}  ${Qt5Core_EXECUTABLE_COMPILE_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS} ${Qt5Svg_EXECUTABLE_COMPILE_FLAGS} -std=c++11 ")


set(SOURCES_FILES 	sensors/threadsensors.cpp
		        gui/threadgui.cpp
		        gui/mainwindow.cpp
		        src/main.cpp
		        sensors/sensors.cpp
				algorithm/rosPublisher.cpp
				algorithm/numMarkerPublisherROS.cpp
				algorithm/timerPublisherROS.cpp
				gui/threadgui.cpp
				sensors/sensors.cpp
				sensors/threadsensors.cpp
				AprilTags/april_tags/matd.c
				AprilTags/april_tags/zhash.c
				AprilTags/april_tags/zarray.c
				AprilTags/april_tags/workerpool.c
				AprilTags/april_tags/unionfind.c
				AprilTags/april_tags/tag36h11.c
				AprilTags/april_tags/tag36h10.c
				AprilTags/april_tags/segment2.c
				AprilTags/april_tags/image_u32.c
				AprilTags/april_tags/image_u8.c
				AprilTags/april_tags/image_f32.c
				AprilTags/april_tags/homography.c
				AprilTags/april_tags/graymodel.c
				AprilTags/april_tags/g2d.c
				AprilTags/april_tags/apriltag.c
				AprilTags/AprilTagsSrc/Edge.cc
				AprilTags/AprilTagsSrc/UnionFindSimple.cc
				AprilTags/AprilTagsSrc/TagFamily.cc
				AprilTags/AprilTagsSrc/TagDetector.cc
				AprilTags/AprilTagsSrc/TagDetection.cc
				AprilTags/AprilTagsSrc/Segment.cc
				AprilTags/AprilTagsSrc/Quad.cc
				AprilTags/AprilTagsSrc/MathUtil.cc
				AprilTags/AprilTagsSrc/Homography33.cc
				AprilTags/AprilTagsSrc/GrayModel.cc
				AprilTags/AprilTagsSrc/GLineSegment2D.cc
				AprilTags/AprilTagsSrc/GLine2D.cc
				AprilTags/AprilTagsSrc/Gaussian.cc
				AprilTags/AprilTagsSrc/FloatImage.cc
				algorithm/pose3di.cpp
    			algorithm/world.cpp 
    			algorithm/drawworld.cpp 
    			algorithm/geometryutils.cpp 
    			algorithm/markerinfo.cpp 
    			algorithm/drawingutils.cpp 
    			algorithm/cameramanager.cpp 
    			algorithm/kalmanfilter.cpp 
    			algorithm/weightedaveragefilter.cpp   
    			algorithm/sharer.cpp
	)

set(HEADERS 		sensors/threadsensors.h
			sensors/sensors.h
			gui/threadgui.h
			gui/mainwindow.h
			algorithm/rosPublisher.h
			algorithm/numMarkerPublisherROS.h
			algorithm/timerPublisherROS.h
			gui/threadgui.h
			AprilTags/april_tags/apriltag.h 
			AprilTags/april_tags/zhash.h 
			AprilTags/april_tags/zarray.h 
			AprilTags/april_tags/workerpool.h 
			AprilTags/april_tags/unionfind.h 
			AprilTags/april_tags/timeprofile.h 
			AprilTags/april_tags/tag36h11.h 
			AprilTags/april_tags/tag36h10.h 
			AprilTags/april_tags/matd.h 
			AprilTags/april_tags/line_fit.h 
			AprilTags/april_tags/image_u32.h 
			AprilTags/april_tags/image_u8.h 
			AprilTags/april_tags/image_f32.h 
			AprilTags/april_tags/homography.h 
			AprilTags/april_tags/gridder.h 
			AprilTags/april_tags/graymodel.h 
			AprilTags/april_tags/common.h 
			AprilTags/AprilTags/Edge.h 
			AprilTags/AprilTags/XYWeight.h 
			AprilTags/AprilTags/UnionFindSimple.h 
			AprilTags/AprilTags/TagFamily.h 
			AprilTags/AprilTags/TagDetector.h 
			AprilTags/AprilTags/TagDetection.h 
			AprilTags/AprilTags/Tag36h11_other.h 
			AprilTags/AprilTags/Tag36h11.h 
			AprilTags/AprilTags/Tag36h9.h 
			AprilTags/AprilTags/Tag25h9.h 
			AprilTags/AprilTags/Tag25h7.h 
			AprilTags/AprilTags/Tag16h5_other.h 
			AprilTags/AprilTags/Tag16h5.h 
			AprilTags/AprilTags/Segment.h 
			AprilTags/AprilTags/Quad.h 
			AprilTags/AprilTags/pch.h 
			AprilTags/AprilTags/MathUtil.h 
			AprilTags/AprilTags/Homography33.h 
			AprilTags/AprilTags/Gridder.h 
			AprilTags/AprilTags/GrayModel.h 
			AprilTags/AprilTags/GLineSegment2D.h 
			AprilTags/AprilTags/GLine2D.h 
			AprilTags/AprilTags/Gaussian.h 
			AprilTags/AprilTags/FloatImage.h 
			algorithm/world.h 
			algorithm/drawworld.h 
			algorithm/IGraphichAlgorithmer.h
			algorithm/geometryutils.h
			algorithm/markerinfo.h
			algorithm/drawingutils.h
			algorithm/cameramanager.h
			algorithm/kalmanfilter.h
			algorithm/ardronedefines.h
			algorithm/weightedaveragefilter.h
			algorithm/pose3di.h
			algorithm/sharer.h
    )


include_directories(	${Qt5Widgets_INCLUDES}
			${Qt5Widgets_INCLUDE_DIRS}
			${Qt5Core_INCLUDE_DIRS}
			${Qt5Gui_INCLUDE_DIRS}
			${Qt5Svg_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIR}
			${Ice_INCLUDE_DIR}
			${INTERFACES_CPP_DIR}
			${easyiceconfig_INCLUDE_DIRS}
			${GTKMM_INCLUDE_DIRS} 
			${GTKGLEXTMM_INCLUDE_DIRS}
			${easyiceconfig_INCLUDE_DIRS}
			${LIBGLADEMM_INCLUDE_DIRS}
			${LIBGNOMECANVASMM_INCLUDE_DIRS}
			${GLUT_INCLUDE_DIRS}
			${OPENGL_INCLUDE_DIRS} 
			${JDE_INCLUDE_DIRS}
			${catkin_INCLUDE_DIRS}

			#GLUT_DIRECTORY
			/usr/include/GL

			#JDEROBOT_DIRECTORY
		    /opt/jderobot/include 
			/opt/jderobot/include/jderobot	
			
			#PROJECT_DIRECTORY
			${PROJECT_SOURCE_DIR}/AprilTags
			${PROJECT_SOURCE_DIR}/AprilTags/AprilTags
			${PROJECT_SOURCE_DIR}/algorithm	
			${PROJECT_SOURCE_DIR}/sensors
			${PROJECT_SOURCE_DIR}/gui					
					
)
  

link_directories(${JDE_LIBS}
                ${easyiceconfig_LIBRARY_DIRS}
				${GTKMM_LIBRARY_DIRS}
				${GTKGLEXTMM_LIBRARY_DIRS}
				${LIBGLADEMM_LIBRARY_DIRS}
				${LIBGNOMECANVASMM_LIBRARY_DIRS}
)




add_definitions(${Qt5Widgets_DEFINITIONS})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

add_executable(visual_markers
                    ${SOURCES_FILES}
                    ${HEADERS})





###############################
add_library(jderobotutil SHARED IMPORTED)
set_property(TARGET jderobotutil PROPERTY IMPORTED_LOCATION
		"/opt/jderobot/lib/libjderobotutil.so")


### ICE
add_library(Ice2 SHARED IMPORTED)
set_property(TARGET Ice2 PROPERTY IMPORTED_LOCATION
		"/usr/lib/x86_64-linux-gnu/libIce.so")


add_library(Ice SHARED IMPORTED)
set_property(TARGET Ice PROPERTY IMPORTED_LOCATION
		"/usr/lib/x86_64-linux-gnu/libIce++11.so.36")

add_library(IceUtil SHARED IMPORTED)
set_property(TARGET IceUtil PROPERTY IMPORTED_LOCATION
		"/usr/lib/x86_64-linux-gnu/libIceUtil++11.so.36")

add_library(libeasyiceconfig SHARED IMPORTED)
set_property(TARGET libeasyiceconfig PROPERTY IMPORTED_LOCATION
		"/opt/jderobot/lib/libeasyiceconfig.so")

## aruco
add_library(Aruco SHARED IMPORTED)
set_property(TARGET Aruco PROPERTY IMPORTED_LOCATION
			"/usr/local/lib/libaruco.so")


add_library(libz SHARED IMPORTED)
set_property(TARGET libz PROPERTY IMPORTED_LOCATION
			"/usr/lib/x86_64-linux-gnu/libz.so")


target_link_libraries(visual_markers
		${Qt5Widgets_LIBRARIES}
		${Qt5Core_LIBRARIES}
		${Qt5Gui_LIBRARIES}
		${Qt5Svg_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
		${gtkglextmm_LIBRARIES}
		${OpenCV_LIBRARIES}
		${OpenCVGUI_LIBRARIES}
		${Ice_LIBRARY}
		${easyiceconfig_LIBRARIES}
		${ZeroCIce_LIBRARIES}
		${GTKMM_LIBRARIES}
		${GTKGLEXTMM_LIBRARIES}
		${LIBGLADEMM_LIBRARIES}
		${LIBGNOMECANVASMM_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARIES}
		${GLOG_LIBRARIES}
		${catkin_LIBRARIES}
		${config_LIBRARIES}
		${Qt5Widgets_LIBRARIES}
		${Qt5Core_LIBRARIES}
		${Qt5Gui_LIBRARIES}
		${Qt5Svg_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
		${catkin_LIBRARIES}
		${libIceStorm}
		${gtkglextmm_LIBRARIES}
		${OpenCV_LIBRARIES}
		${OpenCVGUI_LIBRARIES}
		${Ice_LIBRARY}
		${easyiceconfig_LIBRARIES}
		${ZeroCIce_LIBRARIES}
		${GTKMM_LIBRARIES}
		${GTKGLEXTMM_LIBRARIES}
		${LIBGLADEMM_LIBRARIES}
		${LIBGNOMECANVASMM_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARIES}
		#${GLOG_LIBRARIES}
		${ZeroCIce_LIBRARIES}
		${comm}


		${JderobotInterfaces}
		jderobotutil
		${parallelIce}
		${colorspacesmm}
		${colorspaces}
		${xmlParser}
		${progeo}
		pthread
		#Ice
		${Glog}
		Ice2
		IceUtil
		Aruco
		ICE
		libz

		${config}
		${logger}
		libeasyiceconfig
		${Boost_LIBRARIES}
		${jderobotutil}

		
                ${libjderobotHandlers}
		#${comm_LIBRARIES}

		)

install(TARGETS visual_markers
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT visual_markers
    )

INSTALL (FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ardrone_slam.glade
	${CMAKE_CURRENT_SOURCE_DIR}/resources/introrob.glade
	${CMAKE_CURRENT_SOURCE_DIR}/resources/markers.txt
	${CMAKE_CURRENT_SOURCE_DIR}/resources/configuration.cfg
	${CMAKE_CURRENT_SOURCE_DIR}/resources/sim_calib.yml
	DESTINATION
	${CMAKE_INSTALL_PREFIX}/share/jderobot/conf
	COMPONENT visual_markers
	 )
    



