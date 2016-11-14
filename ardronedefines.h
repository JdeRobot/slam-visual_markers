#ifndef ARDRONEDEFINES_H
#define ARDRONEDEFINES_H

namespace Ardrone
{
    /*** Cálculo de la distancia focal ***/
    //hfov = 1.413717
    //width_ventral = 240;
    //width_frontal = 240;
    //focal_length = width/2*tan(hfov/2);
    //double ventralFocalLength = 140.51; //240x240
    //double frontalFocalLength = 187.336; //320x240
    /*************************************/

    enum ETemporalFusionFilter
    {
        TEMPORAL_FUSION_FILTER_NONE = 0,
        TEMPORAL_FUSION_FILTER_KALMAN = 1,
        TEMPORAL_FUSION_FILTER_WEIGHTED_AVERAGE = 2
    };
}

#endif // ARDRONEDEFINES_H
