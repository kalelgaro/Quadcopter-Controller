#ifndef UBLOX_H_
#define UBLOX_H_

#include "stm32f4xx.h"
#include "string.h"

    /**
     * @brief appendToReceivedDataBuffer
     * Adiciona novos dados no final da fila de dados para tratamento.
     * Utiliza as variáveis receivedData e receivedDataIndex para, respectivamente,
     * armazenar os dados e controlar qual a posição atual para a inserção.
     * O parametro initialPosition indica qual a posição inicial do vetor newData
     * deve ser utilizado para iniciar a recuperação dos dados.
     * @param newData
     * @param initialPosition
     * @param size
     */
    void appendToReceivedDataBuffer(uint8_t newData[], uint16_t initialPosition,  uint16_t size);

    /**
     * @brief treatReceivedData
     * Realiza o tratamento dos bytes recebidos e inseridos no final da variávesl
     * receivedData. Para tratamento dos dados, faz uma cópia dos dados contidos
     * para no vetor de entrada (receivedData) para um segundo vetor.
     *
     */
    void treatReceivedData();

    /**
     * @brief parseNMEASentence
     * Uma sequência de chamadas nesta função realiza a quebra de uma sequência de caracteres que
     * contenham uma sentença GPS e atualizam a variável GPSData. Utiliza o vetor receivedData como
     * a entrada de dados e o vetor toTreatData como local de alocação temporária enquanto encontra os
     * tokens contidos nas mensagens nmea.
     */
    void parseNMEASentence();

    uint8_t foundStartChar;
    uint8_t foundEndCharSequence;

    uint32_t checkSum;

    struct GPSData{
        float lat;
        float lon;
        float alt;
        float hDop;
        float fix;
    };
#endif /* UBLOX_H_ */
