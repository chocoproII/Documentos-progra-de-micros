#ifndef ADC
#define ADC

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida espec�fica para microcontroladores AVR
#include <stdint.h>    // Incluye la biblioteca para tipos de datos enteros de tama�o fijo

void ConfADC(void);
uint16_t valorADC(uint8_t canal);

#endif  // Fin de la directiva de inclusi�n condicional
