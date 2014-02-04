#include <stm32f4xx_conf.h>

#include "array_functions.h"
//#include <math.h>

void inverter_vetor(uint8_t *buffer, uint8_t tamanho) //Inverte a posição dos elementos num vetor
{
    uint8_t counter = 0;
    uint8_t temp = 0;

    for(; counter < tamanho; counter++)
    {
    	temp = buffer[tamanho-1];					//Buffer do ultimo elemento.
    	buffer[tamanho-1] = buffer[counter];  		//Copia o primeiro elemento para o ultimo.
    	buffer[counter] = temp;						//Copia o ultimo elemento para o primeiro
    	tamanho--;
    }
}

void copy_to(uint8_t *destino,uint8_t *origem, uint8_t posicao_destino,uint8_t lenght)
{
    uint8_t counter = 0;
	
    lenght++;

    for(; lenght > 0; lenght--)
    {
        destino[posicao_destino] = origem[counter];
        posicao_destino++;
	    counter++;
    }
}


void escrever_string_buffer(uint8_t *buffer,const uint8_t *menssagem)
{
    uint8_t i = 0;
    while(menssagem[i] != '\0')
    {
        buffer[i] = menssagem[i];
        i++;
    }

    buffer[i] = '\0';
}

void limpar_buffer(uint8_t *inicial, uint8_t tamanho)
{
	for(;tamanho > 0;tamanho --)
	{
            inicial[tamanho-1] = 0;
	}
	return;
}

uint8_t numToASCII(int16_t numero,uint8_t *buffer) //Converte um numero para caracteres ASCII na mesma ordem com que este está escrito.
{
    uint16_t counter = 0;
    uint8_t flag_negativo = 0;

    if(numero < 0)
    {
        numero = -numero;
        counter++;
        flag_negativo = 1;
    }
    
    do
    {
        buffer[counter] = (numero%10 + '0');
        numero = numero/10;
        counter++;
    }while(numero != 0);
    
    if(counter < 2)
    {
        buffer[counter] = '0';
        counter++;
    }

    if(flag_negativo == 1)
    {
        buffer[counter] = '-';
        counter++;
    }
    
    inverter_vetor(buffer,counter);
    
    return counter;
}


void printfint(double numerof,uint8_t *destino)
{
	uint8_t index = 0;

	if(numerof < 0)
	{
		destino[0] = '-';
		index++;
		numerof = -numerof;
	}else if(numerof > 0)
	{
		destino[0] = '0';
		index++;
	}

	uint16_t numero = (unsigned long)numerof;

    
    uint8_t buffer[15];
      
    uint8_t tamanho = numToASCII(numero,buffer);
    
    copy_to(destino,buffer,index,tamanho);
    
    numerof -= numero;
    numerof *= 100;
    
    numero = (int)numerof;
    
    index = tamanho;
    
    destino[tamanho] = '.';
    
    index++;
     
    tamanho = numToASCII(numero,buffer);
    
    copy_to(destino,buffer,index,tamanho);
    
    index+=tamanho;                                 //Posição após a cópia.
    destino[index] = CR;
    index++;
    destino[index] = LF;
    
}

void copiar(uint8_t *destino, uint8_t *origem, uint8_t tamanho)
{
    uint8_t index = tamanho;
    for(; index>0; index--)
    {
        destino[index-1] = origem[index-1];
    }
}
