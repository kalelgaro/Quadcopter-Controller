#ifndef ACCELEROMETERDEVICE
#define ACCELEROMETERDEVICE

#include "ThreeAxisSensors.h"

namespace Sensors {

template <typename DataType, typename RawDataType>
class Accelerometer : public ThreeAxisSensors<DataType, RawDataType> {

public:
    virtual void updateData() = 0;

private:


};

}


#endif // ACCELEROMETERDEVICE

