#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#define USE_APRIL_TAGS_C_LIBRARY 0

#include "geometryutils.h"
#include "markerinfo.h"
#include <progeo/progeo.h>
#include <aruco/aruco.h>

#if USE_APRIL_TAGS_C_LIBRARY
#include "april_tags/apriltag.h"
#else
#include "AprilTags/TagDetector.h"
#endif

namespace Ardrone
{
    class CameraManager
    {
    public:
        static const double MARKER_SIZE;
        static const double HALF_MARKER_SIZE;
        static const std::map<int, Ardrone::MarkerInfo*> MARKERS;

    private:
        TPinHoleCamera m_RealCamera;
        TPinHoleCamera m_EstimatedCamera;
        cv::Mat m_CameraMatrix;
        Eigen::Matrix3d m_K;
        cv::Mat m_DistortionCoeffs;
        aruco::CameraParameters m_CameraParameters;
        double m_BaseFoaX, m_BaseFoaY, m_BaseFoaZ;
        Pose m_RealPose;
        Pose m_EstimatedPose;
        cv::Mat m_MarkerPoints;
        std::map<int, cv::Mat> m_DetectedMarkerPoints;
        int m_LastMarkerDetected;

#if USE_APRIL_TAGS_C_LIBRARY
        april_tag_family_t* m_TagFamily;
        april_tag_detector_t* m_TagDetector;
#else
        AprilTags::TagDetector* m_TagDetector;
#endif

    private:
        static double GetMarkerSize();
        static std::map<int, Ardrone::MarkerInfo*> BuildMarkers();

        void DrawRectangle(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color, int width);
        void DrawMarker(cv::Mat& image, const cv::Mat& points, int id);

        void UpdateCameraExtrinsics(TPinHoleCamera& cam, const Eigen::Matrix4d& rt);

    public:
        static double GetWeight(double distance);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CameraManager(const std::string& calibFile, double foaX, double foaY, double foaZ);
        virtual ~CameraManager();

        const TPinHoleCamera& GetRealCamera();
        const TPinHoleCamera& GetEstimatedCamera();

        const Eigen::Matrix3d& GetIntrinsicsMatrix();
        Pose GetRealPose();
        Pose GetEstimatedPose();
        bool GetRealDistanceToMarker(int marker, double& distance);
        bool GetRealAnglesToMarker(int marker, double& roll, double& pitch, double& yaw);
        int GetLastMarkerDetected();

        void SetRealPose(double x, double y, double z, double h, double roll, double pitch, double yaw);
        void SetEstimatedPose(double x, double y, double z, double h, double roll, double pitch, double yaw);
        void SetEstimatedPose(const Eigen::Matrix4d& rt);

        bool ProcessImage(cv::Mat& image);

        cv::Point2f GetRealProjectedPoint(CvPoint3D32f point);
        cv::Point2f GetEstimatedProjectedPoint(CvPoint3D32f point);
        std::vector<cv::Point2f> GetCornerDiffs(int id);

        void DrawRealPoint(cv::Mat& image, CvPoint3D32f point, cv::Scalar color);
        void DrawEstimatedPoint(cv::Mat& image, CvPoint3D32f point, cv::Scalar color);
        void DrawProjectedRealMarker(cv::Mat& image, int id);
    };
}

#endif // CAMERAMANAGER_H
