#ifndef MARKERINFO_H
#define MARKERINFO_H

#include <aruco/aruco.h>
#include <eigen3/Eigen/Dense>

namespace Ardrone
{
    class MarkerInfo
    {
    private:
        int m_Id;
        CvPoint3D32f m_Position;
        double m_Roll;
        double m_Pitch;
        double m_Yaw;
        Eigen::Matrix4d m_WorldRT; //Baliza con respecto al mundo
        Eigen::Matrix4d m_MarkerRT; //Mundo con respecto a la baliza
        CvPoint3D32f m_AxisX;
        CvPoint3D32f m_AxisY;
        CvPoint3D32f m_AxisZ;
        int m_Size;
        std::vector<CvPoint3D32f> m_CornerPositions;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MarkerInfo(int id, double x, double y, double z, double rotX, double rotY, double rotZ, double size);
        virtual ~MarkerInfo();

        int GetId();
        CvPoint3D32f GetPosition();
        double GetRoll();
        double GetPitch();
        double GetYaw();
        const Eigen::Matrix4d& GetWorldRT();
        const Eigen::Matrix4d& GetMarkerRT();
        CvPoint3D32f GetAxisX();
        CvPoint3D32f GetAxisY();
        CvPoint3D32f GetAxisZ();
        std::vector<CvPoint3D32f> GetCornerPositions();
    };
}

#endif // MARKERINFO_H
