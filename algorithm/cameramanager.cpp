#include "cameramanager.h"
#if USE_APRIL_TAGS_C_LIBRARY
#include "april_tags/tag36h11.h"
#include "april_tags/matd.h"
#else
#include "AprilTags/Tag36h11.h"
#endif
#include "drawingutils.h"
#include <fstream>
#include <iostream>
#include "opencv2/opencv.hpp"

//#define MARKER_SIZE 0.17
//#define HALF_MARKER_SIZE 0.085

namespace Ardrone
{
    const double CameraManager::MARKER_SIZE = CameraManager::GetMarkerSize();
    const double CameraManager::HALF_MARKER_SIZE = CameraManager::MARKER_SIZE/2;
    const std::map<int, Ardrone::MarkerInfo*> CameraManager::MARKERS = CameraManager::BuildMarkers();
}

using namespace Ardrone;


CameraManager::CameraManager(/*int rows, int columns, */const std::string& calibFile,double foaX, double foaY, double foaZ)
{


    //Matriz de parámetros intrínsecos
    ifstream file(calibFile);
    
	if (!file)	//no utilizamos el archivo de calibracion
    {
 	//Cámara real
	std::cout<<"Usamos los parametros por defecto"<<std::endl;
    m_RealCamera.position.X = 0.0;
    m_RealCamera.position.Y = 0.0;
    m_RealCamera.position.Z = 0.0;
    m_RealCamera.position.H = 1.0;
    m_RealCamera.foa.X = 0.0;
    m_RealCamera.foa.Y = 0.0;
    m_RealCamera.foa.Z = 0.0;
    m_RealCamera.foa.H = 1.0;
    m_RealCamera.roll = 0.0;
    m_RealCamera.fdistx = 187.336;
    m_RealCamera.fdisty = 187.336;
    m_RealCamera.u0 = 160;
    m_RealCamera.v0 = 120;
    m_RealCamera.skew = 0.0;
    m_RealCamera.rows = 240;
    m_RealCamera.columns = 320;
    update_camera_matrix(&m_RealCamera);

    //Cámara estimada (Parámetros simulación gazebo)
    m_EstimatedCamera.position.X = 0.0;
    m_EstimatedCamera.position.Y = 0.0;
    m_EstimatedCamera.position.Z = 0.0;
    m_EstimatedCamera.position.H = 1.0;
    m_EstimatedCamera.foa.X = 0.0;
    m_EstimatedCamera.foa.Y = 0.0;
    m_EstimatedCamera.foa.Z = 0.0;
    m_EstimatedCamera.foa.H = 1.0;
    m_EstimatedCamera.roll = 0.0;
    m_EstimatedCamera.fdistx = 187.336;
    m_EstimatedCamera.fdisty = 187.336;
    m_EstimatedCamera.u0 = 160;
    m_EstimatedCamera.v0 = 120;
    m_EstimatedCamera.skew = 0.0;
    m_EstimatedCamera.rows = 240;
    m_EstimatedCamera.columns = 320;
    update_camera_matrix(&m_EstimatedCamera);







    double cam[] = { 187.336, 0, 160,
                      0, 187.336, 120,
                      0, 0, 1};
    cv::Mat auxMat(3, 3, CV_64FC1, cam);
    auxMat.copyTo(m_CameraMatrix);

    m_K(0,0) = 187.336;
    m_K(0,1) = 0;
    m_K(0,2) = 0;
    m_K(1,0) = 0;
    m_K(1,1) = 187.336;
    m_K(1,2) = 120;
    m_K(2,0) = 0;
    m_K(2,1) = 0;
    m_K(2,2) = 1;
    


    double dis[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    cv::Mat distCoeffs(5, 1 ,CV_64FC1, dis);
    distCoeffs.copyTo(m_DistortionCoeffs);
    }
    else //usamos archivo calibracion

    {	
	std::cout<<"Usamos el archivo de calibracion: "<<calibFile<<std::endl;
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    cameraMatrix.copyTo(m_CameraMatrix);
 	
    m_K(0,0) = cameraMatrix.at<double>(0,0);
    m_K(0,1) = cameraMatrix.at<double>(0,1);
    m_K(0,2) = cameraMatrix.at<double>(0,2);
    m_K(1,0) = cameraMatrix.at<double>(1,0);
    m_K(1,1) = cameraMatrix.at<double>(1,1);
    m_K(1,2) = cameraMatrix.at<double>(1,2);
    m_K(2,0) = cameraMatrix.at<double>(2,0);
    m_K(2,1) = cameraMatrix.at<double>(2,1);
    m_K(2,2) = cameraMatrix.at<double>(2,2);
 
    distCoeffs.copyTo(distCoeffs);
     

	//Cámara real

    m_RealCamera.position.X = 0.0;
    m_RealCamera.position.Y = 0.0;
    m_RealCamera.position.Z = 0.0;
    m_RealCamera.position.H = 1.0;
    m_RealCamera.foa.X = 0.0;
    m_RealCamera.foa.Y = 0.0;
    m_RealCamera.foa.Z = 0.0;
    m_RealCamera.foa.H = 1.0;
    m_RealCamera.roll = 0.0;
    m_RealCamera.fdistx = m_K(0,0);
    m_RealCamera.fdisty = m_K(1,1);
    m_RealCamera.u0 = m_K(0,2);
    m_RealCamera.v0 = m_K(1,2);
    m_RealCamera.skew = 0.0;
    m_RealCamera.rows =  fs["image_height"];
    m_RealCamera.columns = fs["image_width"];
    update_camera_matrix(&m_RealCamera);

    //Cámara estimada (Parámetros simulación gazebo)
    m_EstimatedCamera.position.X = 0.0;
    m_EstimatedCamera.position.Y = 0.0;
    m_EstimatedCamera.position.Z = 0.0;
    m_EstimatedCamera.position.H = 1.0;
    m_EstimatedCamera.foa.X = 0.0;
    m_EstimatedCamera.foa.Y = 0.0;
    m_EstimatedCamera.foa.Z = 0.0;
    m_EstimatedCamera.foa.H = 1.0;
    m_EstimatedCamera.roll = 0.0;
    m_EstimatedCamera.fdistx = m_K(0,0);
    m_EstimatedCamera.fdisty = m_K(1,1);
    m_EstimatedCamera.u0 = m_K(0,2);
    m_EstimatedCamera.v0 = m_K(1,2);
    m_EstimatedCamera.skew = 0.0;
    m_EstimatedCamera.rows = fs["image_height"];
    m_EstimatedCamera.columns = fs["image_width"];
    update_camera_matrix(&m_EstimatedCamera);



    
    }

    

    //Foco de atención "base"
    m_BaseFoaX = foaX;
    m_BaseFoaY = foaY;
    m_BaseFoaZ = foaZ;
    m_RealPose.SetBaseFoa(foaX, foaY, foaZ);
    m_EstimatedPose.SetBaseFoa(foaX, foaY, foaZ);

    //Detector de AprilTags
#if USE_APRIL_TAGS_C_LIBRARY
    m_TagFamily = tag36h11_create();
    m_TagDetector = april_tag_detector_create(m_TagFamily);
#else
    m_TagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
#endif

    //Esquinas de un marcador referenciadas al centro del marcador
    m_MarkerPoints.create(4,3,CV_32FC1);
    m_MarkerPoints.at<float>(0,0) = -HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(0,1) = -HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(0,2) = 0;
    m_MarkerPoints.at<float>(1,0) = -HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(1,1) = HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(1,2) = 0;
    m_MarkerPoints.at<float>(2,0) = HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(2,1) = HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(2,2) = 0;
    m_MarkerPoints.at<float>(3,0) = HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(3,1) = -HALF_MARKER_SIZE;
    m_MarkerPoints.at<float>(3,2) = 0;

    //Esquinas del marcador detectado en coordenadas de imagen
    for (std::map<int, Ardrone::MarkerInfo*>::const_iterator iter = CameraManager::MARKERS.begin(); iter != CameraManager::MARKERS.end(); ++iter)
    {
        m_DetectedMarkerPoints[iter->first].create(4,2,CV_32FC1);
    }

    m_LastMarkerDetected = -1;
}

CameraManager::~CameraManager()
{
#if USE_APRIL_TAGS_C_LIBRARY
    delete m_TagFamily;
#endif
    delete m_TagDetector;
}

double
CameraManager::GetMarkerSize()
{
    double result;



    std::ifstream file("markers.txt");
    std::string line;
    std::getline(file, line);
    std::stringstream aux(line);
    aux >> result;

    return result;
}

std::map<int, Ardrone::MarkerInfo*>
CameraManager::BuildMarkers()
{
    std::map<int, Ardrone::MarkerInfo*> result;

//    //Virtual world
//    result[0] = new MarkerInfo(0, 2.0f, 1.0f, 0, 0, 0, -CV_PI/2, MARKER_SIZE);
//    result[1] = new MarkerInfo(1, -3.0f, 2.0f, 0, 0, 0, -CV_PI/2, MARKER_SIZE);
//    result[2] = new MarkerInfo(2, 1.5f, -3.0f, 0, 0, 0, -CV_PI/2, MARKER_SIZE);
//    result[3] = new MarkerInfo(3, -7.0f, 7.0f, 0, 0, 0, -CV_PI/2, MARKER_SIZE);
//    //    result[4] = new MarkerInfo(4, -5.0f, 5.0f, 3.0f, 0, -CV_PI/2, 0, MARKER_SIZE);
//    //    result[5] = new MarkerInfo(5, 3.0f, 6.0f, 4.0f, -CV_PI/2, 0, CV_PI/2, MARKER_SIZE);
//    //    result[6] = new MarkerInfo(6, 4.0f, -3.0f, 1.5f, CV_PI/2, 0, -CV_PI/2, MARKER_SIZE);
//    //    result[7] = new MarkerInfo(7, 7.0f, 7.0f, 2.5f, 0, CV_PI/2, CV_PI/2, MARKER_SIZE);
//    //    result[8] = new MarkerInfo(8, -7.0f, -7.0f, 2.0f, CV_PI/2, 0, -CV_PI/2, MARKER_SIZE);
//    result[9] = new MarkerInfo(9, -3.0f, -4.0f, 0, 0, 0, -CV_PI/2, MARKER_SIZE);
//    result[10] = new MarkerInfo(10, 5.0f, 0.0f, 3.0f, CV_PI/2, 0, -CV_PI/2, MARKER_SIZE);
//    //AprilTags_4Tags.world
//    result[0] = new MarkerInfo(0, 1.0f, 1.0f, 0.5f, 0, CV_PI/8, -CV_PI/2, MARKER_SIZE);
//    result[1] = new MarkerInfo(1, -1.0f, 1.0f, 0.5f, 0, CV_PI/8, -CV_PI/2, MARKER_SIZE);
//    result[2] = new MarkerInfo(2, 1.0f, -1.0f, 0.5f, 0, CV_PI/8, -CV_PI/2, MARKER_SIZE);
//    result[3] = new MarkerInfo(3, -1.0f, -1.0f, 0.5f, 0, CV_PI/8, -CV_PI/2, MARKER_SIZE);
//    result[4] = new MarkerInfo(4, 2.0f, 0.0f, 0.5f, 0, -CV_PI/8, -CV_PI/2, MARKER_SIZE);
//    //Real world
//    result[0] = new MarkerInfo(0, 1.5f, 0.0f, 0.6f, CV_PI/2, 0, CV_PI, MARKER_SIZE);


    std::ifstream file("markers.txt");
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line))
    {
        if (line.size() > 10 && line.find("#") == std::string::npos)
        {
            int id;
            double x, y, z, roll, pitch, yaw;
            std::stringstream aux(line);
            aux >> id;
            aux >> x;
            aux >> y;
            aux >> z;
            aux >> roll;
            aux >> pitch;
            aux >> yaw;
            result[id] = new MarkerInfo(id, x, y, z, roll, pitch, yaw, MARKER_SIZE);
        }
    }

    return result;
}

void
CameraManager::DrawRectangle(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color, int width)
{
    cv::line(image, points[0], points[1], color, width/*, CV_AA*/);
    cv::line(image, points[1], points[2], color, width/*, CV_AA*/);
    cv::line(image, points[2], points[3], color, width/*, CV_AA*/);
    cv::line(image, points[3], points[0], color, width/*, CV_AA*/);
}

void
CameraManager::DrawMarker(cv::Mat& image, const cv::Mat& points, int id)
{
    cv::Point2f p0(points.at<float>(0,0), points.at<float>(0,1));
    cv::Point2f p1(points.at<float>(1,0), points.at<float>(1,1));
    cv::Point2f p2(points.at<float>(2,0), points.at<float>(2,1));
    cv::Point2f p3(points.at<float>(3,0), points.at<float>(3,1));
    std::vector<cv::Point2f> pointsVector;
    pointsVector.push_back(p0);
    pointsVector.push_back(p1);
    pointsVector.push_back(p2);
    pointsVector.push_back(p3);

    int lineWidth = 1;
    cv::Scalar color(0, 0, 255);

    DrawRectangle(image, pointsVector, cv::Scalar(0, 0, 255), lineWidth);
    cv::rectangle(image, p0 - cv::Point2f(2,2), p0 + cv::Point2f(2,2), cv::Scalar(0,0,255,255), lineWidth/*, CV_AA*/);
    cv::rectangle(image, p1 - cv::Point2f(2,2), p1 + cv::Point2f(2,2), cv::Scalar(0,255,0,255), lineWidth/*, CV_AA*/);
    cv::rectangle(image, p2 - cv::Point2f(2,2), p2 + cv::Point2f(2,2), cv::Scalar(255,0,0,255), lineWidth/*, CV_AA*/);

    char aux[100];
    sprintf(aux,"%d",id);
    //Centroide
    cv::Point cent(0,0);
    cent.x = (p0.x + p1.x + p2.x + p3.x)/4.0;
    cent.y = (p0.y + p1.y + p2.y + p3.y)/4.0;
    cv::putText(image, aux, cent, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255-color[0],255-color[1],255-color[2],255),2);
}

void
CameraManager::UpdateCameraExtrinsics(TPinHoleCamera& cam, const Eigen::Matrix4d& rt)
{
    Eigen::Matrix4d aux = rt;

    cam.rt11 = aux(0,0);
    cam.rt12 = aux(0,1);
    cam.rt13 = aux(0,2);
    cam.rt14 = aux(0,3);
    cam.rt21 = aux(1,0);
    cam.rt22 = aux(1,1);
    cam.rt23 = aux(1,2);
    cam.rt24 = aux(1,3);
    cam.rt31 = aux(2,0);
    cam.rt32 = aux(2,1);
    cam.rt33 = aux(2,2);
    cam.rt34 = aux(2,3);
    cam.rt41 = aux(3,0);
    cam.rt42 = aux(3,1);
    cam.rt43 = aux(3,2);
    cam.rt44 = aux(3,3);
}

double
CameraManager::GetWeight(double distance)
{
    //TODO Ajustar los pesos

    double result = 0.0;

    if ((distance <= 1) && (distance > 0))
    {
        result = 1.4;
    }
    if ((distance <= 2) && (distance > 1))
    {
        result = 1.1;
    }
    else if((distance <= 3) && (distance > 2))
    {
        result = 0.5;
    }
    else if((distance <= 10) && (distance > 3))
    {
        result = 0.2;
    }

    return result;
}

double
CameraManager::GetWeightPose(double distance)
{
    //TODO Ajustar los pesos

    double result = 0.0;

    if ((distance <= 0.5) && (distance > 0))
    {
        result = 1.3;
    }
    else if((distance <= 1) && (distance > 0.5))
    {
        result = 0.9;
    }
    else if(distance > 1)
    {
        result = 0.2;
    }

    return result;
}

const TPinHoleCamera&
CameraManager::GetRealCamera()
{
    return m_RealCamera;
}

const TPinHoleCamera&
CameraManager::GetEstimatedCamera()
{
    return m_EstimatedCamera;
}

std::vector<TPinHoleCamera, Eigen::aligned_allocator<TPinHoleCamera> >&
CameraManager::GetBalCameras()
{
    return m_BalCameras;
}

std::vector<Pose, Eigen::aligned_allocator<Pose> >&
CameraManager::GetBalPoses()
{
    return m_BalPoses;
}


const Eigen::Matrix3d&
CameraManager::GetIntrinsicsMatrix()
{
    return m_K;
}

Pose
CameraManager::GetRealPose()
{
    return m_RealPose;
}

Pose
CameraManager::GetEstimatedPose()
{
    return m_EstimatedPose;
}

bool
CameraManager::GetRealDistanceToMarker(int marker, double& distance)
{
    bool result = false;

    if (MARKERS.find(marker) != MARKERS.end())
    {
        distance = GeometryUtils::GetDistance(m_RealPose.GetX()-MARKERS.at(marker)->GetPosition().x, m_RealPose.GetY()-MARKERS.at(marker)->GetPosition().y, m_RealPose.GetZ()-MARKERS.at(marker)->GetPosition().z);
        result = true;
    }

    return result;
}

bool
CameraManager::GetRealAnglesToMarker(int marker, double& roll, double& pitch, double& yaw)
{
    bool result = false;

    if (MARKERS.find(marker) != MARKERS.end())
    {
        Eigen::Matrix4d RT_CM = m_RealPose.GetRT()*MARKERS.at(marker)->GetWorldRT();
        Eigen::Matrix3d rt;
        rt(0,0) = RT_CM(0,0);
        rt(0,1) = RT_CM(0,1);
        rt(0,2) = RT_CM(0,2);
        rt(1,0) = RT_CM(1,0);
        rt(1,1) = RT_CM(1,1);
        rt(1,2) = RT_CM(1,2);
        rt(2,0) = RT_CM(2,0);
        rt(2,1) = RT_CM(2,1);
        rt(2,2) = RT_CM(2,2);
        GeometryUtils::RotationMatrixToRPY(rt, roll, pitch, yaw);
        result = true;
    }

    return result;
}

int
CameraManager::GetLastMarkerDetected()
{
    return m_LastMarkerDetected;
}



void
CameraManager::SetRealPose(double x, double y, double z, double h, double roll, double pitch, double yaw)
{
    m_RealPose.Update(x, y, z, h, roll, pitch, yaw);
    m_RealCamera.position.X = m_RealPose.GetX();
    m_RealCamera.position.Y = m_RealPose.GetY();
    m_RealCamera.position.Z = m_RealPose.GetZ();
    m_RealCamera.position.H = h;
    m_RealCamera.foa.X = m_RealPose.GetFoaX();
    m_RealCamera.foa.Y = m_RealPose.GetFoaY();
    m_RealCamera.foa.Z = m_RealPose.GetFoaZ();
    m_RealCamera.foa.H = m_RealPose.GetFoaH();
    UpdateCameraExtrinsics(m_RealCamera, m_RealPose.GetRT().inverse().eval());
}

void
CameraManager::SetEstimatedPose(double x, double y, double z, double h, double roll, double pitch, double yaw)
{
    m_EstimatedPose.Update(x, y, z, h, roll, pitch, yaw);
    m_EstimatedCamera.position.X = m_EstimatedPose.GetX();
    m_EstimatedCamera.position.Y = m_EstimatedPose.GetY();
    m_EstimatedCamera.position.Z = m_EstimatedPose.GetZ();
    m_EstimatedCamera.position.H = h;
    m_EstimatedCamera.foa.X = m_EstimatedPose.GetFoaX();
    m_EstimatedCamera.foa.Y = m_EstimatedPose.GetFoaY();
    m_EstimatedCamera.foa.Z = m_EstimatedPose.GetFoaZ();
    m_EstimatedCamera.foa.H = m_EstimatedPose.GetFoaH();
    UpdateCameraExtrinsics(m_EstimatedCamera, m_EstimatedPose.GetRT().inverse().eval());
}


/*
void
CameraManager::SetBalPoses(double x, double y, double z, double h, double roll, double pitch, double yaw)
{

    Pose* p = new Pose(x, y, z, h, roll, pitch, yaw);
    p->Update(x, y, z, h, roll, pitch, yaw);
    m_BalPoses.push_back(p);
    m_EstimatedCamera.position.X = p.GetX();
    m_EstimatedCamera.position.Y = p.GetY();
    m_EstimatedCamera.position.Z = p.GetZ();
    m_EstimatedCamera.position.H = h;
    m_EstimatedCamera.foa.X = p.GetFoaX();
    m_EstimatedCamera.foa.Y = p.GetFoaY();
    m_EstimatedCamera.foa.Z = p.GetFoaZ();
    m_EstimatedCamera.foa.H = p.GetFoaH();
    UpdateCameraExtrinsics(m_EstimatedCamera, p.GetRT().inverse().eval());
}
*/

void
CameraManager::SetEstimatedPose(const Eigen::Matrix4d& rt)
{
    m_EstimatedPose.Update(rt);
    m_EstimatedCamera.position.X = m_EstimatedPose.GetX();
    m_EstimatedCamera.position.Y = m_EstimatedPose.GetY();
    m_EstimatedCamera.position.Z = m_EstimatedPose.GetZ();
    //m_EstimatedCamera.position.H = h;
    m_EstimatedCamera.foa.X = m_EstimatedPose.GetFoaX();
    m_EstimatedCamera.foa.Y = m_EstimatedPose.GetFoaY();
    m_EstimatedCamera.foa.Z = m_EstimatedPose.GetFoaZ();
    m_EstimatedCamera.foa.H = m_EstimatedPose.GetFoaH();
    UpdateCameraExtrinsics(m_EstimatedCamera, m_EstimatedPose.GetRT().inverse().eval());
}



bool
CameraManager::ProcessImage(cv::Mat& image)
{
    bool result = false;

    std::vector<Pose*> poses;
#if USE_APRIL_TAGS_C_LIBRARY

    image_u8_t* img = image_u8_create_from_rgb3(image.cols, image.rows, (uint8_t*)image.data, image.step);
    zarray_t* detections = april_tag_detector_detect(m_TagDetector, img);
    for (int i = 0; i < zarray_size(detections); i++)
    {
        april_tag_detection_t* det;
        zarray_get(detections, i, &det);

//            printf("detection %3d: id %4d, hamming %d, goodness %f, center (%f, %f) hom:\n(%f, %f, %f;\n%f, %f, %f;\n%f %f, %f)\n",
//               i, det->id, det->hamming, det->goodness, det->c[0], det->c[1],
//               MAT_EL(det->H, 0, 0), MAT_EL(det->H, 0, 1), MAT_EL(det->H, 0, 2),
//               MAT_EL(det->H, 1, 0), MAT_EL(det->H, 1, 1), MAT_EL(det->H, 1, 2),
//               MAT_EL(det->H, 2, 0), MAT_EL(det->H, 2, 1), MAT_EL(det->H, 2, 2));


        if (MARKERS.find(det->id) != MARKERS.end())
        {
            m_LastMarkerDetected = det->id;

            //Cálculo de extrínsecos
            cv::Mat imgPoints(4,2,CV_32FC1);
            imgPoints.at<float>(0,0) = det->p[3][0];
            imgPoints.at<float>(0,1) = det->p[3][1];
            imgPoints.at<float>(1,0) = det->p[0][0];
            imgPoints.at<float>(1,1) = det->p[0][1];
            imgPoints.at<float>(2,0) = det->p[1][0];
            imgPoints.at<float>(2,1) = det->p[1][1];
            imgPoints.at<float>(3,0) = det->p[2][0];
            imgPoints.at<float>(3,1) = det->p[2][1];
            cv::Mat rvec, tvec;
            rvec.create(3,1,CV_32FC1);
            tvec.create(3,1,CV_32FC1);
            cv::Mat raux,taux;
            cv::solvePnP(m_MarkerPoints, imgPoints, m_CameraMatrix, m_DistortionCoeffs, raux, taux);
            raux.convertTo(rvec, CV_32F);
            taux.convertTo(tvec, CV_32F);
            DrawMarker(image, imgPoints, det->id);

            cv::Mat R;
            cv::Rodrigues(rvec, R);
            Eigen::Matrix4d RT_CM; //RT marker con respecto a la cámara
            RT_CM(0,0)=R.at<float>(0,0);
            RT_CM(0,1)=R.at<float>(0,1);
            RT_CM(0,2)=R.at<float>(0,2);
            RT_CM(0,3)=tvec.at<float>(0,0);
            RT_CM(1,0)=R.at<float>(1,0);
            RT_CM(1,1)=R.at<float>(1,1);
            RT_CM(1,2)=R.at<float>(1,2);
            RT_CM(1,3)=tvec.at<float>(1,0);
            RT_CM(2,0)=R.at<float>(2,0);
            RT_CM(2,1)=R.at<float>(2,1);
            RT_CM(2,2)=R.at<float>(2,2);
            RT_CM(2,3)=tvec.at<float>(2,0);
            RT_CM(3,0)=0;
            RT_CM(3,1)=0;
            RT_CM(3,2)=0;
            RT_CM(3,3)=1;

            Eigen::Matrix4d RT_MC = RT_CM.inverse().eval(); //RT cámara con respecto al marker
            Eigen::Matrix4d RT_WC = MARKERS.at(det->id)->GetWorldRT()*RT_MC; //RT cámara con respecto al mundo

            //Posición de la cámara en absolutas;
            float x = RT_WC(0,3);
            float y = RT_WC(1,3);
            float z = RT_WC(2,3);
            float h = RT_WC(3,3);

            //Orientación de la cámara
            double yaw, pitch, roll;
            Eigen::Matrix3d rot;
            rot(0,0)=RT_WC(0,0);
            rot(0,1)=RT_WC(0,1);
            rot(0,2)=RT_WC(0,2);
            rot(1,0)=RT_WC(1,0);
            rot(1,1)=RT_WC(1,1);
            rot(1,2)=RT_WC(1,2);
            rot(2,0)=RT_WC(2,0);
            rot(2,1)=RT_WC(2,1);
            rot(2,2)=RT_WC(2,2);
            Ardrone::GeometryUtils::RotationMatrixToRPY(rot, roll, pitch, yaw);

            Pose* p = new Pose(x, y, z, h, roll, pitch, yaw);
            p->SetWeight(GetWeight(Ardrone::GeometryUtils::GetDistance(tvec.at<float>(0,0), tvec.at<float>(1,0), tvec.at<float>(2,0))));
            poses.push_back(p);
        }

        april_tag_detection_destroy(det);
    }
    zarray_destroy(detections);

#else





    // COSA MÍA
    std::vector<float> PosVect(3,.0);
    int npos = 0;

    m_BalPoses.clear();
    m_BalCameras.clear();
    TPinHoleCamera BalCam = m_EstimatedCamera;





    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, CV_BGR2GRAY);
    std::vector<AprilTags::TagDetection> detections = m_TagDetector->extractTags(imageGray);
    for (std::vector<AprilTags::TagDetection>::iterator iter = detections.begin(); iter != detections.end(); ++iter)
    {
        if (MARKERS.find(iter->id) != MARKERS.end())
        {
            m_LastMarkerDetected = iter->id;

            //Cálculo de extrínsecos
            cv::Mat imgPoints(4,2,CV_32FC1);
            imgPoints.at<float>(0,0) = iter->p[0].first;
            imgPoints.at<float>(0,1) = iter->p[0].second;
            imgPoints.at<float>(1,0) = iter->p[3].first;
            imgPoints.at<float>(1,1) = iter->p[3].second;
            imgPoints.at<float>(2,0) = iter->p[2].first;
            imgPoints.at<float>(2,1) = iter->p[2].second;
            imgPoints.at<float>(3,0) = iter->p[1].first;
            imgPoints.at<float>(3,1) = iter->p[1].second;
            imgPoints.copyTo(m_DetectedMarkerPoints[iter->id]);
            cv::Mat rvec, tvec;
            rvec.create(3,1,CV_32FC1);
            tvec.create(3,1,CV_32FC1);
            cv::Mat raux, taux;
            cv::solvePnP(m_MarkerPoints, imgPoints, m_CameraMatrix, m_DistortionCoeffs, raux, taux);
            raux.convertTo(rvec, CV_32F);
            taux.convertTo(tvec, CV_32F);
            DrawMarker(image, imgPoints, iter->id);

            cv::Mat R;
            cv::Rodrigues(rvec, R);
            Eigen::Matrix4d RT_CM; //RT marker con respecto a la cámara
            RT_CM(0,0)=R.at<float>(0,0);
            RT_CM(0,1)=R.at<float>(0,1);
            RT_CM(0,2)=R.at<float>(0,2);
            RT_CM(0,3)=tvec.at<float>(0,0);
            RT_CM(1,0)=R.at<float>(1,0);
            RT_CM(1,1)=R.at<float>(1,1);
            RT_CM(1,2)=R.at<float>(1,2);
            RT_CM(1,3)=tvec.at<float>(1,0);
            RT_CM(2,0)=R.at<float>(2,0);
            RT_CM(2,1)=R.at<float>(2,1);
            RT_CM(2,2)=R.at<float>(2,2);
            RT_CM(2,3)=tvec.at<float>(2,0);
            RT_CM(3,0)=0;
            RT_CM(3,1)=0;
            RT_CM(3,2)=0;
            RT_CM(3,3)=1;

            Eigen::Matrix4d RT_MC = RT_CM.inverse().eval(); //RT cámara con respecto al marker
            Eigen::Matrix4d RT_WC = MARKERS.at(iter->id)->GetWorldRT()*RT_MC; //RT cámara con respecto al mundo

            //Posición de la cámara en absolutas;
            float x = RT_WC(0,3);
            float y = RT_WC(1,3);
            float z = RT_WC(2,3);
            float h = RT_WC(3,3);

            //Orientación de la cámara
            double yaw, pitch, roll;
            Eigen::Matrix3d rot;
            rot(0,0)=RT_WC(0,0);
            rot(0,1)=RT_WC(0,1);
            rot(0,2)=RT_WC(0,2);
            rot(1,0)=RT_WC(1,0);
            rot(1,1)=RT_WC(1,1);
            rot(1,2)=RT_WC(1,2);
            rot(2,0)=RT_WC(2,0);
            rot(2,1)=RT_WC(2,1);
            rot(2,2)=RT_WC(2,2);

            Ardrone::GeometryUtils::RotationMatrixToRPY(rot, roll, pitch, yaw);

            /******************************************************************************************/
//            printf("- ROLL %f, PITCH %f, YAW %f\nn", roll, pitch, yaw);
//            printf("--- %03f %03f %03f %03f ---\n", RT_WC(0,0), RT_WC(0,1), RT_WC(0,2), RT_WC(0,3));
//            printf("--- %03f %03f %03f %03f ---\n", RT_WC(1,0), RT_WC(1,1), RT_WC(1,2), RT_WC(1,3));
//            printf("--- %03f %03f %03f %03f ---\n", RT_WC(2,0), RT_WC(2,1), RT_WC(2,2), RT_WC(2,3));
//            printf("--- %03f %03f %03f %03f ---\n\n", RT_WC(3,0), RT_WC(3,1), RT_WC(3,2), RT_WC(3,3));
//            Eigen::Matrix4d pepe = RT_WC.inverse().eval();
//            printf("@@@ %03f %03f %03f %03f @@@\n", pepe(0,0), pepe(0,1), pepe(0,2), pepe(0,3));
//            printf("@@@ %03f %03f %03f %03f @@@\n", pepe(1,0), pepe(1,1), pepe(1,2), pepe(1,3));
//            printf("@@@ %03f %03f %03f %03f @@@\n", pepe(2,0), pepe(2,1), pepe(2,2), pepe(2,3));
//            printf("@@@ %03f %03f %03f %03f @@@\n\n", pepe(3,0), pepe(3,1), pepe(3,2), pepe(3,3));
//            HPoint3D aux;
//            aux.X = 1.5;
//            aux.Y = 0;
//            aux.Z = 0.6;
//            aux.H = 1;
//            HPoint2D pp = DrawingUtils::myproject(aux, m_K, pepe);
//            printf("* %f %f\n", pp.x, pp.y);
//            project(aux, &pp, m_EstimatedCamera);
//            printf("# %f %f\n", pp.x, pp.y);
            /******************************************************************************************/

            Pose* p = new Pose(x, y, z, h, roll, pitch, yaw);


            //COSA MÍA
            PosVect[0] += x;
            PosVect[1] += y;
            PosVect[2] += z;
            npos += 1;
            m_BalPoses.push_back(*p);

            p->SetWeight(GetWeight(Ardrone::GeometryUtils::GetDistance(tvec.at<float>(0,0), tvec.at<float>(1,0), tvec.at<float>(2,0))));
            poses.push_back(p);
//setBalPoses(x, y, z, h, roll, pitch, yaw);
            //SetEstimatedPose(RT_WC);
        }
    }

#endif



    //#################################
    // "poses" TIENE LAS POSICIONES DETECTADAS POR CADA BALIZA

    if (!poses.empty())
    {


        //COSAMÍA
        //CALCULAR POSICIÓN MEDIA
        PosVect[0] /= npos;
        PosVect[1] /= npos;
        PosVect[2] /= npos;
        printf("#MEAN VECTOR %.3f, %.3f, %.3f\n",PosVect[0], PosVect[1], PosVect[2]);
        double newwgt = 0.0;
        int posSize = poses.size();


        result = true;

        double posX = 0.0;
        double posY = 0.0;
        double posZ = 0.0;
        double sum1Roll = 0.0;
        double sum2Roll = 0.0;
        double sum1Pitch = 0.0;
        double sum2Pitch = 0.0;
        double sum1Yaw = 0.0;
        double sum2Yaw = 0.0;
        double acum = 0.0;
        double poseDist = 0.0;
        for (std::vector<Pose*>::iterator iter = poses.begin(); iter != poses.end(); ++iter)
        {

            //COSAMÍA
            //*

            if (!(posSize < 3)) {

                poseDist = Ardrone::GeometryUtils::GetDistance((*iter)->GetX() - PosVect[0],
                        (*iter)->GetY() - PosVect[1],
                        (*iter)->GetZ() - PosVect[2]);
                printf("POSE DIST : %f \n", poseDist);
                printf("PESOS (dist, mean): %f , %f \n", (*iter)->GetWeight(), GetWeightPose(poseDist) );
                newwgt = (*iter)->GetWeight()*GetWeightPose(poseDist);
                printf("PESO FINAL: %F \n", newwgt );
                (*iter)->SetWeight(newwgt);
                /*Se calcula distancia de cada pose con respecto a la media y se asigna un nuevo
                 * peso según la cercanía a la posición media de las estimaciones.
                 * Total Weight = DistWeight * MediaWeight
                 * Max weight: 1.96, min weight: ~0;
                 * La cercanía a la estimación media tiene más peso que la distancia a la baliza */

                //TODO: CÁLCULO DE ERROR EN ÁNGULOS
            }

            posX += (*iter)->GetWeight()*(*iter)->GetX();
            posY += (*iter)->GetWeight()*(*iter)->GetY();
            posZ += (*iter)->GetWeight()*(*iter)->GetZ();

            sum1Roll += (*iter)->GetWeight()*sin((*iter)->GetRoll());
            sum2Roll += (*iter)->GetWeight()*cos((*iter)->GetRoll());
            sum1Pitch += (*iter)->GetWeight()*sin((*iter)->GetPitch());
            sum2Pitch += (*iter)->GetWeight()*cos((*iter)->GetPitch());
            sum1Yaw += (*iter)->GetWeight()*sin((*iter)->GetYaw());
            sum2Yaw += (*iter)->GetWeight()*cos((*iter)->GetYaw());

            acum += (*iter)->GetWeight();


            //MIO
            BalCam.position.X =(*iter)->GetX();
            BalCam.position.Y = (*iter)->GetY();
            BalCam.position.Z = (*iter)->GetZ();
            BalCam.position.H = 1;
            BalCam.foa.X = (*iter)->GetFoaX();
            BalCam.foa.Y = (*iter)->GetFoaY();
            BalCam.foa.Z = (*iter)->GetFoaZ();
            BalCam.foa.H = (*iter)->GetFoaH();
            UpdateCameraExtrinsics(BalCam, (*iter)->GetRT().inverse().eval());

            m_BalCameras.push_back(BalCam);
            //

            //printf("- %.3f, %.3f, %.3f, %.3f\n", (*iter)->GetX(), (*iter)->GetY(), (*iter)->GetZ(), (*iter)->GetYaw());

            delete *iter;
        }
        SetEstimatedPose(posX/acum, posY/acum, posZ/acum, 1, atan2(sum1Roll, sum2Roll), atan2(sum1Pitch, sum2Pitch), atan2(sum1Yaw, sum2Yaw));

        //Pose calculada
        printf("# %.3f, %.3f, %.3f -- %.3f, %.3f, %.3f\n", m_EstimatedPose.GetX(), m_EstimatedPose.GetY(), m_EstimatedPose.GetZ(), m_EstimatedPose.GetRoll(), m_EstimatedPose.GetPitch(), m_EstimatedPose.GetYaw());
    }
    printf("#######\n");
    return result;
}

cv::Point2f
CameraManager::GetRealProjectedPoint(CvPoint3D32f point)
{
    HPoint3D worldPoint;
    worldPoint.X = point.x;
    worldPoint.Y = point.y;
    worldPoint.Z = point.z;
    worldPoint.H = 1;
    HPoint2D imgPoint;
    project(worldPoint, &imgPoint, m_RealCamera);
    imgPoint = DrawingUtils::opticas2Graficas(imgPoint.x, imgPoint.y, m_RealCamera.rows, m_RealCamera.columns);

    return cv::Point2f(imgPoint.x, imgPoint.y);
}

cv::Point2f
CameraManager::GetEstimatedProjectedPoint(CvPoint3D32f point)
{
    HPoint3D worldPoint;
    worldPoint.X = point.x;
    worldPoint.Y = point.y;
    worldPoint.Z = point.z;
    worldPoint.H = 1;
    HPoint2D imgPoint;
    project(worldPoint, &imgPoint, m_EstimatedCamera);
    imgPoint = DrawingUtils::opticas2Graficas(imgPoint.x, imgPoint.y, m_EstimatedCamera.rows, m_EstimatedCamera.columns);

    return cv::Point2f(imgPoint.x, imgPoint.y);
}

std::vector<cv::Point2f>
CameraManager::GetCornerDiffs(int id)
{
    std::vector<cv::Point2f> result;

    if (MARKERS.find(id) != MARKERS.end())
    {
        std::vector<CvPoint3D32f> cornerPoints = MARKERS.at(id)->GetCornerPositions();
        int i = 0;
        for (std::vector<CvPoint3D32f>::iterator iter = cornerPoints.begin(); iter != cornerPoints.end(); ++iter, ++i)
        {
            result.push_back(GetRealProjectedPoint(*iter) - cv::Point2f(m_DetectedMarkerPoints[id].at<float>(i,0), m_DetectedMarkerPoints[id].at<float>(i,1)));
        }
    }

    return result;
}


void
CameraManager::DrawRealPoint(cv::Mat& image, CvPoint3D32f point, cv::Scalar color)
{
    cv::Point2f aux = GetRealProjectedPoint(point);
    cv::rectangle(image, aux - cv::Point2f(2,2), aux + cv::Point2f(2,2), color, 1);
}

void
CameraManager::DrawEstimatedPoint(cv::Mat& image, CvPoint3D32f point, cv::Scalar color)
{
    cv::Point2f aux = GetEstimatedProjectedPoint(point);
    //cv::rectangle(image, aux - cv::Point2f(2,2), aux + cv::Point2f(2,2), color, 1);
    cv::circle(image, aux, 2, color, 2);
}

void
CameraManager::DrawProjectedRealMarker(cv::Mat& image, int id)
{
    if (MARKERS.find(id) != MARKERS.end())
    {
        std::vector<CvPoint3D32f> cornerPoints = MARKERS.at(id)->GetCornerPositions();
        std::vector<cv::Point2f> points;
        for (std::vector<CvPoint3D32f>::iterator iter = cornerPoints.begin(); iter != cornerPoints.end(); ++iter)
        {
            cv::Point2f p = GetRealProjectedPoint(*iter);
            points.push_back(p);
        }
        DrawRectangle(image, points, cv::Scalar(255, 0, 255), 1);
    }
}
