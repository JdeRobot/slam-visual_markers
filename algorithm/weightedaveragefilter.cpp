#include "weightedaveragefilter.h"

using namespace Ardrone;

WeightedAverageFilter::WeightedAverageFilter(unsigned int size)
{
    m_Size = size;

    //TODO Ajustar los pesos
    double sum = 0;
    for (unsigned int i = 0; i < m_Size; ++i)
    {
        double w = (double)(i+1)/(double)m_Size;
        if (w == 1.0){
            w += 1;
            //Da un punto más de peso a la última posición
        }
        m_Weights.push_back(w);
        sum += w;
        m_Sumatories.push_back(sum);
    }
}

WeightedAverageFilter::~WeightedAverageFilter()
{
}

void
WeightedAverageFilter::Reset()
{
    m_Poses.clear();
}

Pose
WeightedAverageFilter::GetFilteredPose(const Pose& zpose)
{
    if (m_Poses.size() == m_Size)
    {
        m_Poses.erase(m_Poses.begin());
    }
    m_Poses.push_back(zpose);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double sum1Roll = 0.0;
    double sum2Roll = 0.0;
    double sum1Pitch = 0.0;
    double sum2Pitch = 0.0;
    double sum1Yaw = 0.0;
    double sum2Yaw = 0.0;
    for (unsigned int i = 0; i < m_Poses.size(); ++i)
    {
        x += m_Weights[i]*m_Poses[i].GetX();
        y += m_Weights[i]*m_Poses[i].GetY();
        z += m_Weights[i]*m_Poses[i].GetZ();

        sum1Roll += m_Weights[i]*sin(m_Poses[i].GetRoll());
        sum2Roll += m_Weights[i]*cos(m_Poses[i].GetRoll());
        sum1Pitch += m_Weights[i]*sin(m_Poses[i].GetPitch());
        sum2Pitch += m_Weights[i]*cos(m_Poses[i].GetPitch());
        sum1Yaw += m_Weights[i]*sin(m_Poses[i].GetYaw());
        sum2Yaw += m_Weights[i]*cos(m_Poses[i].GetYaw());
    }

    double sum = m_Sumatories[m_Poses.size()-1];
    return Pose(x/sum, y/sum, z/sum, 1, atan2(sum1Roll, sum2Roll), atan2(sum1Pitch, sum2Pitch), atan2(sum1Yaw, sum2Yaw), zpose.GetBaseFoaX(), zpose.GetBaseFoaY(), zpose.GetBaseFoaZ());
}
