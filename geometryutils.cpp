#include "geometryutils.h"
#include <iostream>

using namespace Ardrone;

Pose::Pose()
{
    SetBaseFoa(0, 0, 1, 1);
    Update(0, 0, 0, 1, 0, 0, 0);
}

Pose::Pose(float x, float y, float z, float h, float roll, float pitch, float yaw, float foax, float foay, float foaz)
{
    SetBaseFoa(foax, foay, foaz, h);
    Update(x, y, z, h, roll, pitch, yaw);
}

Pose::Pose(const Eigen::Matrix4d& rt)
{
    Update(rt);
}

void
Pose::SetBaseFoa(float foax, float foay, float foaz, float h)
{
    m_BaseFoa.X = foax;
    m_BaseFoa.Y = foay;
    m_BaseFoa.Z = foaz;
    m_BaseFoa.H = h;
}

void
Pose::Update(float x, float y, float z, float h, float roll, float pitch, float yaw)
{
    m_Position.X = x;
    m_Position.Y = y;
    m_Position.Z = z;
    m_Position.H = h;
    m_Roll = roll;
    m_Pitch = pitch;
    m_Yaw = yaw;
    m_RT = GeometryUtils::BuildRTMat(x, y, z, roll, pitch, yaw);

    Eigen::Vector4d foaEigenCam(m_BaseFoa.X, m_BaseFoa.Y, m_BaseFoa.Z, m_BaseFoa.H);
    Eigen::Vector4d foaEigen = m_RT*foaEigenCam;
    m_Foa.X = foaEigen(0);
    m_Foa.Y = foaEigen(1);
    m_Foa.Z = foaEigen(2);
    m_Foa.H = h;
}

void
Pose::Update(const Eigen::Matrix4d& rt)
{
    m_RT = rt;

    m_Position.X = rt(0,3);
    m_Position.Y = rt(1,3);
    m_Position.Z = rt(2,3);
    m_Position.H = rt(3,3);

    double yaw, pitch, roll;
    Eigen::Matrix3d rot;
    rot(0,0)=rt(0,0);
    rot(0,1)=rt(0,1);
    rot(0,2)=rt(0,2);
    rot(1,0)=rt(1,0);
    rot(1,1)=rt(1,1);
    rot(1,2)=rt(1,2);
    rot(2,0)=rt(2,0);
    rot(2,1)=rt(2,1);
    rot(2,2)=rt(2,2);
    Ardrone::GeometryUtils::RotationMatrixToRPY(rot, roll, pitch, yaw);
    m_Roll = roll;
    m_Pitch = pitch;
    m_Yaw = yaw;

    //printf("*** %03f %03f %03f %03f ***\n", m_RT(0,0), m_RT(0,1), m_RT(0,2), m_RT(0,3));
    //printf("*** %03f %03f %03f %03f ***\n", m_RT(1,0), m_RT(1,1), m_RT(1,2), m_RT(1,3));
    //printf("*** %03f %03f %03f %03f ***\n", m_RT(2,0), m_RT(2,1), m_RT(2,2), m_RT(2,3));
    //printf("*** %03f %03f %03f %03f ***\n", m_RT(3,0), m_RT(3,1), m_RT(3,2), m_RT(3,3));

    //Eigen::Matrix4d aux = GeometryUtils::BuildRTMat(m_Position.X, m_Position.Y, m_Position.Z, roll, pitch, yaw);
    Eigen::Vector4d foaEigenCam(m_BaseFoa.X, m_BaseFoa.Y, m_BaseFoa.Z, m_BaseFoa.H);
    Eigen::Vector4d foaEigen = rt*foaEigenCam;
    m_Foa.X = foaEigen(0);
    m_Foa.Y = foaEigen(1);
    m_Foa.Z = foaEigen(2);
    m_Foa.H = m_BaseFoa.H;
}

/******************** GEOMETRY UTILS *************************************************************/

HPoint3D
GeometryUtils::GetPointOfLine(HPoint3D a, HPoint3D b, double dist)
{
    Line aux(a, b);
    double u1_2 = pow(aux.U1(), 2);
    double u2_2 = pow(aux.U2(), 2);
    double u3_2 = pow(aux.U3(), 2);

    double t = dist / sqrt(u1_2 + u2_2 + u3_2);

    return aux.GetPoint(t);
}

HPoint3D
GeometryUtils::GetPointOfLineAndPlane(const Line& line, const Plane& plane)
{
    double t = (plane.A()*(plane.X0() - line.X0()) + plane.B()*(plane.Y0() - line.Y0()) + plane.C()*(plane.Z0() - line.Z0()))/(plane.A()*line.U1() + plane.B()*line.U2() + plane.C()*line.U3());

    return line.GetPoint(t);
}

Eigen::Matrix4d
GeometryUtils::BuildRTMat(float x, float y, float z, float rotX, float rotY, float rotZ)
{
    Eigen::Matrix4d result;

    cv::Mat auxMat1(cv::Size(3,3),CV_32FC1);
    cv::Mat auxVec1(cv::Size(1,3),CV_32FC1);
    auxVec1.at<float>(0,0) = rotX;
    auxVec1.at<float>(1,0) = 0;
    auxVec1.at<float>(2,0) = 0;
    cv::Rodrigues(auxVec1, auxMat1);

    cv::Mat auxMat2(cv::Size(3,3),CV_32FC1);
    cv::Mat auxVec2(cv::Size(1,3),CV_32FC1);
    auxVec2.at<float>(0,0) = 0;
    auxVec2.at<float>(1,0) = rotY;
    auxVec2.at<float>(2,0) = 0;
    cv::Rodrigues(auxVec2, auxMat2);

    cv::Mat auxMat3(cv::Size(3,3),CV_32FC1);
    cv::Mat auxVec3(cv::Size(1,3),CV_32FC1);
    auxVec3.at<float>(0,0) = 0;
    auxVec3.at<float>(1,0) = 0;
    auxVec3.at<float>(2,0) = rotZ;
    cv::Rodrigues(auxVec3, auxMat3);

    cv::Mat rotMat = auxMat3*auxMat2;
    rotMat = rotMat*auxMat1;

    result(0,0) = rotMat.at<float>(0,0);
    result(0,1) = rotMat.at<float>(0,1);
    result(0,2) = rotMat.at<float>(0,2);
    result(0,3) = x;
    result(1,0) = rotMat.at<float>(1,0);
    result(1,1) = rotMat.at<float>(1,1);
    result(1,2) = rotMat.at<float>(1,2);
    result(1,3) = y;
    result(2,0) = rotMat.at<float>(2,0);
    result(2,1) = rotMat.at<float>(2,1);
    result(2,2) = rotMat.at<float>(2,2);
    result(2,3) = z;
    result(3,0) = 0;
    result(3,1) = 0;
    result(3,2) = 0;
    result(3,3) = 1;

    return result;
}

float
GeometryUtils::quatToRoll(float qw, float qx, float qy, float qz)
{
    double rotateXa0 = 2.0*(qy*qz + qw*qx);
    double rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
    float rotateX = 0.0;
    if (rotateXa0 != 0.0 && rotateXa1 != 0.0)
        rotateX = atan2(rotateXa0, rotateXa1);

    return rotateX;
}

float
GeometryUtils::quatToPitch(float qw, float qx, float qy, float qz)
{
    double rotateYa0 = -2.0*(qx*qz - qw*qy);
    float rotateY = 0.0;
    if( rotateYa0 >= 1.0 )
        rotateY = M_PI/2.0;
    else if( rotateYa0 <= -1.0 )
        rotateY = -M_PI/2.0;
    else rotateY = asin(rotateYa0);

    return rotateY;
}

float
GeometryUtils::quatToYaw(float qw, float qx, float qy, float qz)
{
    double rotateZa0 = 2.0*(qx*qy + qw*qz);
    double rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
    float rotateZ = 0.0;
    if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
        rotateZ = atan2(rotateZa0, rotateZa1);

    return rotateZ;
}

double
GeometryUtils::StandardRad(double t)
{
  if (t >= 0.)
  {
    t = fmod(t+CV_PI, 2.0*CV_PI) - CV_PI;
  }
  else
  {
    t = fmod(t-CV_PI, -2.0*CV_PI) + CV_PI;
  }
  return t;
}

double
GeometryUtils::StandardRad2(double t)
{
    double result = StandardRad(t);
    if (result < 0.)
    {
        result += 2*CV_PI;
    }
    return result;
}

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) {
    return (epsilon > std::abs(a - b));
}


void
GeometryUtils::RotationMatrixToRPY(const Eigen::Matrix3d& rot, double& roll, double& pitch, double& yaw)
{
    yaw = StandardRad(atan2(rot(1,0), rot(0,0)));
    //double c = cos(yaw);
    //double s = sin(yaw);
    //pitch = StandardRad(atan2(-rot(2,0), rot(0,0)*c + rot(1,0)*s));
    //roll  = StandardRad(atan2(rot(0,2)*s - rot(1,2)*c, -rot(0,1)*s + rot(1,1)*c));
    pitch = StandardRad(atan2(-rot(2,0),sqrt(pow(rot(2,1), 2) + pow(rot(2,2), 2))));
    roll = StandardRad(atan2(rot(2,1), rot(2,2)));

    //std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << "----------" << std::endl;

//    //Si estoy considerando que el orden de giros siempre es roll, pitch, yaw
//    roll = -StandardRad(atan2(rot(1,2), rot(2,2)));
//    pitch = StandardRad(asin(rot(0,2)));
//    yaw = -StandardRad(atan2(rot(0,1), rot(0,0)));

}

void
GeometryUtils::RPYToRotationMatrix(double roll, double pitch, double yaw, Eigen::Matrix3f& rot)
{
    Eigen::Matrix3f rotX;
    rotX(0,0) = 1;
    rotX(0,1) = 0;
    rotX(0,2) = 0;
    rotX(1,0) = 0;
    rotX(1,1) = cos(roll);
    rotX(1,2) = -sin(roll);
    rotX(2,0) = 0;
    rotX(2,1) = sin(roll);
    rotX(2,2) = cos(roll);

    Eigen::Matrix3f rotY;
    rotY(0,0) = cos(pitch);
    rotY(0,1) = 0;
    rotY(0,2) = sin(pitch);
    rotY(1,0) = 0;
    rotY(1,1) = 1;
    rotY(1,2) = 0;
    rotY(2,0) = -sin(pitch);
    rotY(2,1) = 0;
    rotY(2,2) = cos(pitch);

    Eigen::Matrix3f rotZ;
    rotZ(0,0) = cos(yaw);
    rotZ(0,1) = -sin(yaw);
    rotZ(0,2) = 0;
    rotZ(1,0) = sin(yaw);
    rotZ(1,1) = cos(yaw);
    rotZ(1,2) = 0;
    rotZ(2,0) = 0;
    rotZ(2,1) = 0;
    rotZ(2,2) = 1;

    rot = rotX*rotY*rotZ;
}

double
GeometryUtils::GetDistance(double x, double y, double z)
{
    return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

double
GeometryUtils::GetRadialError(const Pose& realPose, const Pose& estimatedPose)
{
    return GetDistance(realPose.GetX()-estimatedPose.GetX(), realPose.GetY()-estimatedPose.GetY(), realPose.GetZ()-estimatedPose.GetZ());
}

double
GeometryUtils::GetXError(const Pose& realPose, const Pose& estimatedPose)
{
    return fabs(realPose.GetX()-estimatedPose.GetX());
}

double
GeometryUtils::GetYError(const Pose& realPose, const Pose& estimatedPose)
{
    return fabs(realPose.GetY()-estimatedPose.GetY());
}

double
GeometryUtils::GetZError(const Pose& realPose, const Pose& estimatedPose)
{
    return fabs(realPose.GetZ()-estimatedPose.GetZ());
}

double
GeometryUtils::GetAngularError(const Pose& realPose, const Pose& estimatedPose)
{
    return (pow(GetRollError(realPose, estimatedPose), 2) + pow(GetPitchError(realPose, estimatedPose), 2) + pow(GetYawError(realPose, estimatedPose), 2))/3.0;
}

double
GeometryUtils::GetRollError(const Pose& realPose, const Pose& estimatedPose)
{
    return StandardRad(realPose.GetRoll() - estimatedPose.GetRoll());
}

double
GeometryUtils::GetPitchError(const Pose& realPose, const Pose& estimatedPose)
{
    return StandardRad(realPose.GetPitch() - estimatedPose.GetPitch());
}

double
GeometryUtils::GetYawError(const Pose& realPose, const Pose& estimatedPose)
{
    return StandardRad(realPose.GetYaw() - estimatedPose.GetYaw());
}
