#ifndef I2CDEVICE
#define I2CDEVICE

#include <stddef.h>

template <typename SlaveAddressType,
          typename RegAddressType,
          typename DataType>
class I2CDevice {
public:
    I2CDevice() {

    }

    void sendData(SlaveAddressType slaveAddress, RegAddressType regAddress,  DataType const * data, size_t size);
    void writeRegister(SlaveAddressType slaveAddress, RegAddressType regAddress, DataType data);

    DataType const * readData(SlaveAddressType deviceAddress, RegAddressType registerAddress, size_t size);
    DataType readData(SlaveAddressType deviceAddress, RegAddressType registerAddress);

    DataType readRegister(SlaveAddressType deviceAddress, RegAddressType registerAddress);


private:
};

#endif // I2CDEVICE

