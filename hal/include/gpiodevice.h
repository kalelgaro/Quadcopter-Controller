#ifndef GPIODEVICE_H
#define GPIODEVICE_H

template <typename PinDescript>
class GPIODevice {
public:
    GPIODevice () { }

    virtual void setOutputBit(PinDescript gpioPin) const = 0;
    virtual void clearOutputBit(PinDescript gpioPin) const = 0;
    virtual void toggleOutputBit(PinDescript gpioPin) const = 0;

    virtual bool readInputBit(PinDescript gpioPin) const = 0;

    virtual void toggleAll( void ) const = 0;
    virtual void setAll( void ) const = 0;
    virtual void clearAll( void ) const = 0;
};

#endif // GPIODEVICE_H

