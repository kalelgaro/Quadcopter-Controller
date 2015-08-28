#include "ThreeAxisSensors.h"

template <typename DataType, typename RawDataType>
Sensors::ThreeAxisSensors<DataType, RawDataType>::ThreeAxisSensors() :
    m_data( {DataType(), DataType(), DataType()} )
  , m_x(m_data[0])
  , m_y(m_data[1])
  , m_z(m_data[2])
  , m_scale(0.0)
  , m_rawData( {RawDataType(), RawDataType(), RawDataType() } )
  , m_rawX(m_rawData[0])
  , m_rawY(m_rawData[1])
  , m_rawZ(m_rawData[2])
{

}

template <typename DataType, typename RawDataType>
Sensors::ThreeAxisSensors<DataType, RawDataType>::ThreeAxisSensors(const DataType &x, const DataType &y, const DataType &z, const DataType &scale) :
    m_data({x, y, z})
  , m_x(m_data[0])
  , m_y(m_data[1])
  , m_z(m_data[2])
  , m_scale(scale)
  , m_rawData( {RawDataType(), RawDataType(), RawDataType() } )
  , m_rawX(m_rawData[0])
  , m_rawY(m_rawData[1])
  , m_rawZ(m_rawData[2])
{

}

template <typename DataType, typename RawDataType>
Sensors::ThreeAxisSensors<DataType, RawDataType>::~ThreeAxisSensors()
{

}

template <typename DataType, typename RawDataType>
const DataType &Sensors::ThreeAxisSensors<DataType, RawDataType>::getAbsoluteValue() const
{
    //return sqrtf(m_x*m_x + m_y*m_y + m_z*m_z);
}

template class Sensors::ThreeAxisSensors<float, unsigned int>;
