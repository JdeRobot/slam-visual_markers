#ifndef WEIGHTEDAVERAGEFILTER_H
#define WEIGHTEDAVERAGEFILTER_H

#include "geometryutils.h"

namespace Ardrone
{
    class WeightedAverageFilter
    {
    private:
        unsigned int m_Size;
        std::vector<Pose, Eigen::aligned_allocator<Pose> > m_Poses;
        std::vector<double> m_Weights;
        std::vector<double> m_Sumatories;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        WeightedAverageFilter(unsigned int size);
        virtual ~WeightedAverageFilter();

        void Reset();
        Pose GetFilteredPose(const Pose& zpose);
    };
}

#endif // WEIGHTEDAVERAGEFILTER_H
