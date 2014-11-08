#include "UBLOX.h"

#define UBLOX_RECEIVED_BUFFER_SIZE 150

uint8_t receivedData[UBLOX_RECEIVED_BUFFER_SIZE];
uint8_t toTreatData[UBLOX_RECEIVED_BUFFER_SIZE];
uint8_t receivedDataIndex = 0;

void appendToReceivedDataBuffer(uint8_t newData[], uint16_t initialPosition, uint16_t size)
{
    uint8_t i;
    for(i = 0; i < size; i++, receivedDataIndex ++) {
        receivedData[receivedDataIndex] = newData[i + initialPosition];
    }
}

void parseNMEASentence()
{
    static uint8_t *parseIndex = 0;
    uint8_t i = 0;

    //Se o ponteiro parseIndex for zero (NULO), implica que ainda não está "parsenado" uma string.
    if(parseIndex == 0) {
        i = 0;
        while((receivedData[i] != '$') && i < receivedDataIndex) {
            i++;
        }

        if(receivedData[i] == '$') {
            parseIndex = &receivedData[i]; //Armazena a posição do ponteiro do inicio da sentença.
            uint8_t teste = *parseIndex;
        }
    }
}
