#ifndef ADC
#define ADC

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida específica para microcontroladores AVR
#include <stdint.h>    // Incluye la biblioteca para tipos de datos enteros de tamaño fijo

void ConfADC(void);
uint16_t valorADC(uint8_t canal);

#endif  // Fin de la directiva de inclusión condicional
