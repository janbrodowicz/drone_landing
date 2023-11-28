#include "VisionProcessing/AltitudeDataProcessingBase.h"

using namespace altitude;


AltitudeDataProcessingBase::AltitudeDataProcessingBase(float altitude) : 
    m_altitude(altitude)
{

};

int AltitudeDataProcessingBase::exp_square_size(int circle_size)
{
    int square_size;
    square_size = int(circle_size / 5);

    return square_size;
}