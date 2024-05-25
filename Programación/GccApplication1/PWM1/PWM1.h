#ifndef PWM1
#define PWM1

#include <avr/io.h>   // Incluye la biblioteca de entrada/salida AVR
#include <stdint.h>   // Incluye la biblioteca de tipos de datos est�ndar de C

// Definici�n de constantes para configurar la polaridad de la se�al PWM
#define invertido 1       // Constante para el modo PWM invertido
#define no_invertido 0    // Constante para el modo PWM no invertido

// Declaraci�n de funciones para inicializar y actualizar el PWM del canal A del Timer 1
void initPWM1FastA(uint8_t inverted, uint16_t prescaler); // Inicializa el PWM en el canal A con la polaridad y el prescaler especificados
void updateDCA1(uint8_t duty);                            // Actualiza el ciclo de trabajo (duty cycle) del canal A

// Declaraci�n de funciones para inicializar y actualizar el PWM del canal B del Timer 1
void initPWM1FastB(uint8_t inverted, uint16_t prescaler); // Inicializa el PWM en el canal B con la polaridad y el prescaler especificados
void updateDCB1(uint8_t duty);                            // Actualiza el ciclo de trabajo (duty cycle) del canal B

#endif // Fin de la directiva de inclusi�n condicional
