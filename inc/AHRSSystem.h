#ifndef AHRSSYSTEM_H
#define AHRSSYSTEM_H

#include "stm32f4xx.h"

#include "AHRSBaseSystem.h"

#include "ThreeAxisSensors.h"

#include "accelerometerdevice.h"
#include "gyroscopedevice.h"
#include "magnetometerdevice.h"

#include "MPU6050.h"
#include "HMC5883L.h"

namespace AHRS {

class AHRSSystem : public AHRSBaseSystem<float, unsigned int> {

public:
    AHRSSystem(Sensors::Accelerometer<float, unsigned int>  &accel,
               Sensors::Gyroscope<float, unsigned int>      &gyro,
               Sensors::Magnetometer<float, unsigned int>   &mag);

    ~AHRSSystem();

    /* --- Métodos utilizados pela classe. ---*/
    void update( void );


private:
    /* --- Variáveis membros da classe. ---*/
    Sensors::Accelerometer<float, unsigned int> &m_accel;
    Sensors::Gyroscope<float, unsigned int>     &m_gyro;
    Sensors::Magnetometer<float, unsigned int>  &m_mag;

};

}

#endif // AHRSSYSTEM_H
