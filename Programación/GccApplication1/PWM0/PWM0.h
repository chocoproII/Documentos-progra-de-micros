#ifndef PWM0
#define PWM0

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida espec�fica para microcontroladores AVR
#include <stdint.h>    // Incluye la biblioteca para tipos de datos enteros de tama�o fijo

// Definici�n de las constantes para la configuraci�n de la se�al PWM
#define invertido 1       // Configuraci�n de PWM invertido
#define no_invertido 0    // Configuraci�n de PWM no invertido

// Declaraci�n de funciones para controlar el PWM en el canal A del Timer 0
void initPWM0FastA(uint8_t inverted, uint16_t prescaler);
// Inicializa el canal A del Timer 0 para generar una se�al PWM r�pida con opci�n de inversi�n y un prescaler espec�fico

void updateDCA(uint8_t duty);
// Actualiza el ciclo de trabajo (duty cycle) de la se�al PWM en el canal A del Timer 0

// Declaraci�n de funciones para controlar el PWM en el canal B del Timer 0
void initPWM0FastB(uint8_t inverted, uint16_t prescaler);
// Inicializa el canal B del Timer 0 para generar una se�al PWM r�pida con opci�n de inversi�n y un prescaler espec�fico

void updateDCB(uint8_t duty);
// Actualiza el ciclo de trabajo (duty cycle) de la se�al PWM en el canal B del Timer 0

#endif  // Fin de la directiva de inclusi�n condicional
