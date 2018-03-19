#include "markerinfo.h"
#include "geometryutils.h"

using namespace Ardrone;

MarkerInfo::MarkerInfo(int id, double x, double y, double z, double rotX, double rotY, double rotZ, double size)
{
    m_Id = id;
    m_Position = cvPoint3D32f(x, y, z);
    m_WorldRT = GeometryUtils::BuildRTMat(x, y, z, rotX, rotY, rotZ);
    m_MarkerRT = m_WorldRT.inverse().eval();
    m_Size = size;

    Eigen::Matrix3d rt;
    rt(0,0) = m_WorldRT(0,0);
    rt(0,1) = m_WorldRT(0,1);
    rt(0,2) = m_WorldRT(0,2);
    rt(1,0) = m_WorldRT(1,0);
    rt(1,1) = m_WorldRT(1,1);
    rt(1,2) = m_WorldRT(1,2);
    rt(2,0) = m_WorldRT(2,0);
    rt(2,1) = m_WorldRT(2,1);
    rt(2,1) = m_WorldRT(2,2);
    GeometryUtils::RotationMatrixToRPY(rt, m_Roll, m_Pitch, m_Yaw);

    Eigen::Vector4d auxX(1, 0, 0, 1);
    Eigen::Vector4d auxY(0, 1, 0, 1);
    Eigen::Vector4d auxZ(0, 0, 1, 1);
    auxX = m_WorldRT*auxX;
    m_AxisX = cvPoint3D32f(auxX(0), auxX(1), auxX(2));
    auxY = m_WorldRT*auxY;
    m_AxisY = cvPoint3D32f(auxY(0), auxY(1), auxY(2));
    auxZ = m_WorldRT*auxZ;
    m_AxisZ = cvPoint3D32f(auxZ(0), auxZ(1), auxZ(2));

    Eigen::Vector4d aux0(-size/2, -size/2, 0, 1);
    Eigen::Vector4d aux1(-size/2, size/2, 0, 1);
    Eigen::Vector4d aux2(size/2, size/2, 0, 1);
    Eigen::Vector4d aux3(size/2, -size/2, 0, 1);
    aux0 = m_WorldRT*aux0;
    m_CornerPositions.push_back(cvPoint3D32f(aux0(0), aux0(1), aux0(2)));
    aux1 = m_WorldRT*aux1;
    m_CornerPositions.push_back(cvPoint3D32f(aux1(0), aux1(1), aux1(2)));
    aux2 = m_WorldRT*aux2;
    m_CornerPositions.push_back(cvPoint3D32f(aux2(0), aux2(1), aux2(2)));
    aux3 = m_WorldRT*aux3;
    m_CornerPositions.push_back(cvPoint3D32f(aux3(0), aux3(1), aux3(2)));
}

MarkerInfo::~MarkerInfo()
{

}

int
MarkerInfo::GetId()
{
    return m_Id;
}

CvPoint3D32f
MarkerInfo::GetPosition()
{
    return m_Position;
}

double
MarkerInfo::GetRoll()
{
    return m_Roll;
}

double
MarkerInfo::GetPitch()
{
    return m_Pitch;
}

double
MarkerInfo::GetYaw()
{
    return m_Yaw;
}

const Eigen::Matrix4d&
MarkerInfo::GetWorldRT()
{
    return m_WorldRT;
}

const Eigen::Matrix4d&
MarkerInfo::GetMarkerRT()
{
    return m_MarkerRT;
}

CvPoint3D32f
MarkerInfo::GetAxisX()
{
    return m_AxisX;
}

CvPoint3D32f
MarkerInfo::GetAxisY()
{
    return m_AxisY;
}

CvPoint3D32f
MarkerInfo::GetAxisZ()
{
    return m_AxisZ;
}

std::vector<CvPoint3D32f>
MarkerInfo::GetCornerPositions()
{
    return m_CornerPositions;
}
