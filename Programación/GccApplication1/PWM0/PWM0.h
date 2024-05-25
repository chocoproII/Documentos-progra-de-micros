#ifndef PWM0
#define PWM0

#include <avr/io.h>    // Incluye la biblioteca de entrada/salida específica para microcontroladores AVR
#include <stdint.h>    // Incluye la biblioteca para tipos de datos enteros de tamaño fijo

// Definición de las constantes para la configuración de la señal PWM
#define invertido 1       // Configuración de PWM invertido
#define no_invertido 0    // Configuración de PWM no invertido

// Declaración de funciones para controlar el PWM en el canal A del Timer 0
void initPWM0FastA(uint8_t inverted, uint16_t prescaler);
// Inicializa el canal A del Timer 0 para generar una señal PWM rápida con opción de inversión y un prescaler específico

void updateDCA(uint8_t duty);
// Actualiza el ciclo de trabajo (duty cycle) de la señal PWM en el canal A del Timer 0

// Declaración de funciones para controlar el PWM en el canal B del Timer 0
void initPWM0FastB(uint8_t inverted, uint16_t prescaler);
// Inicializa el canal B del Timer 0 para generar una señal PWM rápida con opción de inversión y un prescaler específico

void updateDCB(uint8_t duty);
// Actualiza el ciclo de trabajo (duty cycle) de la señal PWM en el canal B del Timer 0

#endif  // Fin de la directiva de inclusión condicional
