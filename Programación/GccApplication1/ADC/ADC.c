#include "ADC.h"

void ConfADC(void) {
	// Inicializa el registro ADMUX a 0
	ADMUX = 0;

	// Selecciona la referencia de voltaje AVcc con un capacitor en AREF
	ADMUX |= (1 << REFS0);   // Establece REFS0 en 1 para AVcc con un capacitor en AREF
	ADMUX &= ~(1 << REFS1);  // Asegura que REFS1 esté en 0

	// Ajuste a la izquierda del resultado del ADC para facilitar la lectura
	ADMUX |= (1 << ADLAR);   // Establece ADLAR en 1 para ajustar el resultado a la izquierda

	// Inicializa el registro ADCSRA a 0
	ADCSRA = 0;

	// Habilita el ADC
	ADCSRA |= (1 << ADEN);   // Establece ADEN en 1 para habilitar el ADC

	// Establece la frecuencia de prescaler del ADC a 128
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Configura el prescaler en 128

	// Deshabilita las entradas digitales en los pines ADC5 y ADC4
	DIDR0 |= (1 << ADC5D);  // Deshabilita la entrada digital en el pin ADC5
	DIDR0 |= (1 << ADC4D);  // Deshabilita la entrada digital en el pin ADC4
}



uint16_t valorADC(uint8_t canal) {
	// Selecciona el canal del ADC limpiando los cuatro bits menos significativos de ADMUX
	ADMUX &= 0xF0;  // Limpia los bits MUX3:0 para seleccionar el canal
	ADMUX |= canal; // Establece los bits MUX3:0 al valor del canal deseado
	
	// Inicia una conversión de ADC
	ADCSRA |= (1 << ADSC); // Establece el bit ADSC para comenzar la conversión
	
	// Espera a que la conversión de ADC termine (el bit ADSC se pondrá a 0)
	while (ADCSRA & (1 << ADSC)); // Espera mientras ADSC sea 1 (la conversión está en progreso)
	
	// Retorna los 8 bits más significativos del resultado del ADC
	return ADCH; // Devuelve el valor de ADCH (los 8 bits más altos del resultado)
}

