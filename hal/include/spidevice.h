#ifndef SPIDEVICE
#define SPIDEVICE

#include <stddef.h>

/*---- Classe de interface para generalizar o uso dos dispoitivos SPI. -----*/
template<typename Targ>
class SPIDevice {
public:
    SPIDevice(size_t bufferSize) :
        m_buffer(new Targ(bufferSize)) {  }

    virtual void sendData(const Targ *toSendDataBuffer, Targ *receivedDataBuffer, size_t maxSize) = 0;
    virtual void sendData(const Targ &toSendData, Targ &receiveData) = 0;
    virtual void receiveData(Targ *receivedDataBuffer, size_t maxSize) = 0;

private:
    Targ *m_buffer;

};

#endif // SPIDEVICE

