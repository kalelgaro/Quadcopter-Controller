#ifndef THREEAXISSENSORS_H
#define THREEAXISSENSORS_H

namespace Sensors {

enum SensorIndex {
    X=0,
    Y,
    Z
};

template <typename DataType, typename RawDataType>
class ThreeAxisSensors
{
public:
    ThreeAxisSensors();
    ThreeAxisSensors(const DataType & x, const DataType & y, const DataType & z, const DataType & scale);
    ~ThreeAxisSensors();

    const DataType & getX() const { return m_x; }
    const DataType & getY() const { return m_y; }
    const DataType & getZ() const { return m_z; }

    void setX( const DataType &newX ) { m_x = newX; }
    void setY( const DataType &newY ) { m_y = newY; }
    void setZ( const DataType &newZ ) { m_z = newZ; }

    const DataType & getScale() const { return m_scale; }

    void setScale(const DataType & newScale) { m_scale = newScale; }

    const DataType & getAbsoluteValue() const;

    const RawDataType & getRawX() const { return m_rawX; }
    const RawDataType & getRawY() const { return m_rawY; }
    const RawDataType & getRawZ() const { return m_rawZ; }

    const DataType & operator[](const Sensors::SensorIndex &index) {
        return m_data[index];
    }

private:
    DataType m_data[3];
    DataType &m_x, &m_y, &m_z;

    DataType m_scale;

    RawDataType m_rawData[3];
    RawDataType &m_rawX, &m_rawY, &m_rawZ;

};

}
#endif // THREEAXISSENSORS_H
