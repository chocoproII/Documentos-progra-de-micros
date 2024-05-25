#include "PWM1.h"  // Incluye el encabezado que contiene las declaraciones de las funciones y constantes necesarias

// Función para inicializar el PWM en el canal A del Timer 1
void initPWM1FastA(uint8_t inverted, uint16_t prescaler) {
	// Configura el pin correspondiente al canal A (pin B1) como salida
	DDRB |= (1 << DDB1);
	
	// Configuración del modo de operación del Timer 1 y del modo de salida PWM del canal A
	TCCR1A = 0;  // Borra los bits de control del Timer 1

	if (inverted) {
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0);  // Configura la salida del canal A en modo PWM invertido
		} else {
		TCCR1A |= (1 << COM1A1);  // Configura la salida del canal A en modo PWM no invertido
	}

	TCCR1A |= (1 << WGM10);  // Configura el Timer 1 en modo PWM rápido (parte 1)
	TCCR1B |= (1 << WGM12);  // Configura el Timer 1 en modo PWM rápido (parte 2)
	
	// Configuración del prescaler del Timer 1
	if (prescaler == 1024) {
		TCCR1B |= (1 << CS12) | (1 << CS10);  // Configura el prescaler del Timer 1 a 1024
	}
}

// Función para inicializar el PWM en el canal B del Timer 1
void initPWM1FastB(uint8_t inverted, uint16_t prescaler) {
	// Configura el pin correspondiente al canal B (pin B2) como salida
	DDRB |= (1 << DDB2);
	
	// Configuración del modo de salida PWM del canal B
	if (inverted) {
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0);  // Configura la salida del canal B en modo PWM invertido
		} else {
		TCCR1A |= (1 << COM1B1);  // Configura la salida del canal B en modo PWM no invertido
	}

	TCCR1A |= (1 << WGM10);  // Configura el Timer 1 en modo PWM rápido (parte 1)
	TCCR1B |= (1 << WGM12);  // Configura el Timer 1 en modo PWM rápido (parte 2)
	
	// Configuración del prescaler del Timer 1
	if (prescaler == 1024) {
		TCCR1B |= (1 << CS12) | (1 << CS10);  // Configura el prescaler del Timer 1 a 1024
	}
}

// Función para actualizar el ciclo de trabajo (duty cycle) del canal A del Timer 1
void updateDCA1(uint8_t duty) {
	OCR1A = duty;  // Establece el valor del registro de comparación del canal A
}

// Función para actualizar el ciclo de trabajo (duty cycle) del canal B del Timer 1
void updateDCB1(uint8_t duty) {
	OCR1B = duty;  // Establece el valor del registro de comparación del canal B
}
