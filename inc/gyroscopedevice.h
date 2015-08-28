#ifndef GYROSCOPEDEVICE
#define GYROSCOPEDEVICE

#include "ThreeAxisSensors.h"

namespace Sensors {

template <typename DataType, typename RawDataType>
class Gyroscope : public ThreeAxisSensors<DataType, RawDataType> {

public:
    virtual void updateData() = 0;

private:


};

}

#endif // GYROSCOPEDEVICE

