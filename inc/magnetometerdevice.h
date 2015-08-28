#ifndef MAGNETOMETERDEVICE
#define MAGNETOMETERDEVICE

#include "ThreeAxisSensors.h"

namespace Sensors {

template <typename DataType, typename RawDataType>
class Magnetometer : public ThreeAxisSensors<DataType, RawDataType> {
public:

    virtual void updateData( void ) = 0;

private:


};

}

#endif // MAGNETOMETERDEVICE

