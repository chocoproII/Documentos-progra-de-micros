#include "PWM0.h"  // Incluye el archivo de encabezado que declara las funciones y constantes

// Función para inicializar el PWM en el canal A del Timer 0
void initPWM0FastA(uint8_t inverted, uint16_t prescaler) {
	DDRD |= (1 << DDD6);  // Configura el pin D6 del puerto D como salida para el canal A del Timer 0

	TCCR0A = 0;  // Borra los bits de control del Timer 0

	if (inverted) {
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);  // Configura la salida del canal A en modo PWM invertido
		} else {
		TCCR0A |= (1 << COM0A1);  // Configura la salida del canal A en modo PWM no invertido
	}

	TCCR0A |= (1 << WGM01) | (1 << WGM00);  // Configura el Timer 0 en modo PWM rápido (modo 7)

	if (prescaler == 1024) {
		TCCR0B |= (1 << CS02) | (1 << CS00);  // Configura el prescaler del Timer 0 a 1024
	}
}

// Función para inicializar el PWM en el canal B del Timer 0
void initPWM0FastB(uint8_t inverted, uint16_t prescaler) {
	DDRD |= (1 << DDD5);  // Configura el pin D5 del puerto D como salida para el canal B del Timer 0

	if (inverted) {
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);  // Configura la salida del canal B en modo PWM invertido
		} else {
		TCCR0A |= (1 << COM0B1);  // Configura la salida del canal B en modo PWM no invertido
	}

	TCCR0A |= (1 << WGM01) | (1 << WGM00);  // Configura el Timer 0 en modo PWM rápido (modo 7)

	TCCR0B |= (1 << CS02) | (1 << CS00);  // Configura el prescaler del Timer 0 a 1024
}

// Función para actualizar el ciclo de trabajo (duty cycle) del canal A del Timer 0
void updateDCA(uint8_t duty) {
	OCR0A = duty;  // Actualiza el valor del registro de comparación para el canal A del Timer 0
}

// Función para actualizar el ciclo de trabajo (duty cycle) del canal B del Timer 0
void updateDCB(uint8_t duty) {
	OCR0B = duty;  // Actualiza el valor del registro de comparación para el canal B del Timer 0
}
