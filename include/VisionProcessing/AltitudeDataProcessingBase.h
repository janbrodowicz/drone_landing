#include "IAltitudeDataProcessing.h"


namespace altitude
{

class AltitudeDataProcessingBase : public IAltitudeDataProcessing
{
    public:

        AltitudeDataProcessingBase(float altitude);

        int exp_square_size(int circle_size) override;

    protected:

        float m_altitude;

};


} // namespace altitude