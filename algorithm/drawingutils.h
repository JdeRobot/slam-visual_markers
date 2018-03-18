#ifndef DRAWINGUTILS_H
#define DRAWINGUTILS_H

#include <opencv2/core.hpp>
#include <progeo/progeo.h>
#include <eigen3/Eigen/Dense>

namespace Ardrone
{
    class DrawingUtils
    {
    public:
        static HPoint2D graficas2Opticas(double i, double j, double rows, double columns);
        static HPoint2D opticas2Graficas(double i, double j, double rows, double columns);
        static void drawLine(CvPoint3D32f orig, CvPoint3D32f dest, float width, CvPoint3D32f color);
        static void drawSphere(CvPoint3D32f orig, float width, CvPoint3D32f color);
        static void drawAxis(CvPoint3D32f orig, CvPoint3D32f destX, CvPoint3D32f destY, CvPoint3D32f destZ, float width);
        static void drawCamera(const TPinHoleCamera& cam, CvPoint3D32f color, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT);
        static void drawBalCameras(std::vector<TPinHoleCamera, Eigen::aligned_allocator<TPinHoleCamera> >& cams);
        static HPoint2D myproject(HPoint3D point, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT);
        static HPoint3D mybackproject(HPoint2D, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT);
    };
}
#endif // DRAWINGUTILS_H
