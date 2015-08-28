#ifndef AHRSBASESYSTEM_H
#define AHRSBASESYSTEM_H

#include "ThreeAxisSensors.h"

namespace AHRS {

template <typename DataType, typename RawDataType>
class AHRSBaseSystem
{
public:
    AHRSBaseSystem(Sensors::ThreeAxisSensors<DataType, RawDataType> & accel,
                   Sensors::ThreeAxisSensors<DataType, RawDataType> & gyro,
                   Sensors::ThreeAxisSensors<DataType, RawDataType> & mag) :
        m_accel(accel)
      , m_gyro(gyro)
      , m_mag(mag)
    { }
    ~AHRSBaseSystem() { }

private:
    Sensors::ThreeAxisSensors<DataType, RawDataType> &m_accel;
    Sensors::ThreeAxisSensors<DataType, RawDataType> &m_gyro;
    Sensors::ThreeAxisSensors<DataType, RawDataType> &m_mag;
};

}
#endif // AHRSSYSTEM_H
