#include "AHRSSystem.h"

AHRS::AHRSSystem::AHRSSystem(Sensors::Accelerometer<float, unsigned int> &accel,
                             Sensors::Gyroscope<float, unsigned int> &gyro,
                             Sensors::Magnetometer<float, unsigned int> &mag) :
    AHRSBaseSystem(accel, gyro, mag)
  , m_accel(accel)
  , m_gyro(gyro)
  , m_mag(mag)
{

}

AHRS::AHRSSystem::~AHRSSystem()
{

}

void AHRS::AHRSSystem::update()
{
    m_accel.updateData();
    m_gyro.updateData();
}
