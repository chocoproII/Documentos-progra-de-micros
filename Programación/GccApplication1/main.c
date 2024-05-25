//*****************************************************************************
//Universidad del Valle de Guatemala
//Programación de Microcontroladores
//Archivo:Ultimo_proyecto_microcontroladores 
//Hardware:ATMEGA328P
//Autor:Gabriel Alejandro Sanchez
//Carnet:22022
//*****************************************************************************
#define F_CPU 16000000UL // Define la frecuencia de la CPU del microcontrolador (16 MHz)

#include <avr/io.h> // Incluye la biblioteca de entrada/salida AVR
#include <avr/interrupt.h> // Incluye la biblioteca de interrupciones AVR
#include <util/delay.h> // Incluye la biblioteca para funciones de retraso
#include <avr/eeprom.h> // Incluye la biblioteca para manipulación de la EEPROM
#include "PWM0/PWM0.h" // Incluye la biblioteca de PWM para el ojo izquierdo (PWM0/Timer0)
#include "PWM1/PWM1.h" // Incluye la biblioteca de PWM para el ojo derecho (PWM1/Timer1)
#include "PWM0/PWM0.h" // Incluye la biblioteca de PWM para el ojo izquierdo (PWM0/Timer0)
#include "ADC/ADC.h" // Importar ADC de librería externa

// Ciclos de trabajo PWM
uint16_t dutyCycle1;
uint16_t dutyCycle2;
uint16_t dutyCycle3;
uint16_t dutyCycle4;

// Direcciones en la EEPROM para almacenar los modos
uint16_t* UNO_address = 0x00;    // Dirección de EEPROM para el modo 1
uint16_t* DOS_address = 0x01;    // Dirección de EEPROM para el modo 2
uint16_t* TRES_address = 0x54;   // Dirección de EEPROM para el modo 3
uint16_t* CUATRO_address = 0x0E; // Dirección de EEPROM para el modo 4

volatile char bufferRX; //Buffer para comunicación UART
volatile uint8_t buffer_Analog_Digital;
volatile uint8_t buffer_RX22;


void initUART9600(void){
    // Configuración de RX y TX
    DDRD &= ~(1<<DDD0); // RX como entrada
    DDRD |= (1<<DDD1); // TX como salida
    
    // Configuración del modo fast
    UCSR0A = 0;
    UCSR0A |= (1<<U2X0); // Modo de doble velocidad
    
    // Configuración del registro B
    UCSR0B = 0;
	// Habilitar RX, TX e interrupción de RX
    UCSR0B |= (1<<RXCIE0)|(1 << RXEN0)|(1 << TXEN0); 
    
    /*Configuración del registro C: frame - 8 bits de datos, no paridad,
	 1 bit de stop*/
    UCSR0C = 0;
    UCSR0C |= (1 << UCSZ01)|(1 << UCSZ00);
    
    // Baudrate = 9600
    UBRR0 = 207; // Fórmula para 9600 baudios con F_CPU = 16MHz
}

void transUART (unsigned char valorT) {
	// Esperar a que el buffer de transmisión esté vacío
	while (!(UCSR0A & (1 << UDRE0))); 
	UDR0 = valorT; // Transmitir carácter
}

unsigned char recivUART(void) {
	return bufferRX;
}


void setup(void) {
	// Configuración de los botones como entradas con resistencias pull-up
	DDRD &= ~(1 << DDD2); // Configura el pin PD2 como entrada
	PORTD |= (1 << DDD2); // Activa la resistencia interna de pull-up en PD2

	DDRD &= ~(1 << DDD3); // Configura el pin PD3 como entrada
	PORTD |= (1 << DDD3); // Activa la resistencia interna de pull-up en PD3

	DDRD &= ~(1 << DDD4); // Configura el pin PD4 como entrada
	PORTD |= (1 << DDD4); // Activa la resistencia interna de pull-up en PD4

	DDRD &= ~(1 << DDD7); // Configura el pin PD7 como entrada
	PORTD |= (1 << DDD7); // Activa la resistencia interna de pull-up en PD7

	DDRB &= ~(1 << DDB0); // Configura el pin PB0 como entrada
	PORTB |= (1 << DDB0); // Activa la resistencia interna de pull-up en PB0
	
	DDRB &= ~(1 << DDB3); // Configura el pin PB3 como entrada
	PORTB |= (1 << DDB3); // Activa la resistencia interna de pull-up en PB3

	// Configuración de los LEDs como salidas
	DDRB |= (1 << DDB4);  // Configura el pin PB4 como salida
	DDRC |= (1 << DDC0);  // Configura el pin PC0 como salida
	DDRC |= (1 << DDC1);  // Configura el pin PC1 como salida
	DDRC |= (1 << DDC2);  // Configura el pin PC2 como salida
	DDRC |= (1 << DDC3);  // Configura el pin PC3 como salida
}



int main(void)
{
	// Inicialización de los PWMs en los canales A y B del Timer 0 y 1 con un preescalador de 1024
	initPWM0FastA(0,1024);
	initPWM0FastB(0,1024);
	initPWM1FastA(0,1024);
	initPWM1FastB(0,1024);
	
	/*initUART9600();
	sei();*/

	
	// Configuración inicial del sistema
	setup();
	// Inicialización del convertidor analógico-digital (ADC)
	ConfADC();
	
	// Variables para almacenar los valores de los ciclos de trabajo obtenidos del ADC
	uint16_t dutyCycle1 = valorADC(6);
	uint16_t dutyCycle2 = valorADC(7);
	uint16_t dutyCycle3 = valorADC(5);
	uint16_t dutyCycle4 = valorADC(4);
	
	// Apagar los leds
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC &= ~(1 << DDC3);
	
//	writeTextUART("Elija un modo de eeprom\n");
	
	// Loop principal del programa
	while (1)
	{
		
	/*	if (UDR0 == 49) {
			readUNO();
			} else if (UDR0 == 50) {
			readDOS();
			} else if (UDR0 == 51) {
			readTRES();
			} else if (UDR0 == 52) {
			readCUATRO();
		}*/
		
		if (!(PINB & (1 << DDB0))){
			HandMode();
		}
		
		if (!(PIND & (1 << DDD7))){
			readUNO();
			
			if (!(PINB & (1 << DDB3))){
				writeUNO();
			}
		}
		
		if (!(PIND & (1 << DDD4))){
			readDOS();
			
			if (!(PINB & (1 << DDB3))){
				writeDOS();
			}
		}
		
		if (!(PIND & (1 << DDD3))){
			readTRES();
			
			if (!(PINB & (1 << DDB3))){
				writeTRES();
			}
		}
		
		if (!(PIND & (1 << DDD2))){
			readCUATRO();
			
			if (!(PINB & (1 << DDB3))){
				writeCUATRO();
			}
		}
		
		
	}
	
	return 0;
}


void HandMode (){
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC |= (1 << DDC3);
	SERVO_MOTORES();
}


void SERVO_MOTORES (){
	while (1) {
		// Verificar si se ha detectado una señal en alguno de los pines específicos (botones)
		if (!(PIND & (1 << DDD3)) || !(PIND & (1 << DDD4)) || !(PIND & (1 << DDD7)) || !(PINB & (1 << DDB0))) {
			// Si se ha detectado una señal en cualquiera de los botones, salir del bucle
			break;
		}
		
		// Actualizar el ciclo de trabajo del servo del ojo izquierdo basado en el valor del ADC en el pin 6
		dutyCycle1 = valorADC(6);       // Leer el valor del ADC del canal 6 (ojo izquierdo)
		_delay_ms(10);                  // Esperar 10 ms para estabilizar la lectura
		updateDCA(dutyCycle1 / 6);      // Actualizar el ciclo de trabajo del PWM del canal A del Timer 0 (ojo izquierdo)
		
		// Actualizar el ciclo de trabajo del servo del ojo derecho basado en el valor del ADC en el pin 7
		dutyCycle2 = valorADC(7);       // Leer el valor del ADC del canal 7 (ojo derecho)
		_delay_ms(10);            
		      // Esperar 10 ms para estabilizar la lectura
		updateDCB(dutyCycle2 / 6);      // Actualizar el ciclo de trabajo del PWM del canal B del Timer 0 (ojo derecho)
		
		// Actualizar el ciclo de trabajo del segundo servo del ojo izquierdo basado en el valor del ADC en el pin 5
		dutyCycle3 = valorADC(5);       // Leer el valor del ADC del canal 5 (segundo servo ojo izquierdo)
		_delay_ms(10);                  // Esperar 10 ms para estabilizar la lectura
		updateDCA1(dutyCycle3 / 6);     // Actualizar el ciclo de trabajo del PWM del canal A del Timer 1 (segundo servo ojo izquierdo)
		
		// Actualizar el ciclo de trabajo del segundo servo del ojo derecho basado en el valor del ADC en el pin 4
		dutyCycle4 = valorADC(4);       // Leer el valor del ADC del canal 4 (segundo servo ojo derecho)
		_delay_ms(10);                  // Esperar 10 ms para estabilizar la lectura
		updateDCB1(dutyCycle4 / 6);     // Actualizar el ciclo de trabajo del PWM del canal B del Timer 1 (segundo servo ojo derecho)
		
		// Retraso antes de la próxima iteración del bucle
		_delay_ms(25);                  // Esperar 25 ms antes de repetir el bucle
	}

}


void writeUNO (void){
	// Configura los pines correspondientes para UNO
	PORTB &= ~(1 << DDB4);
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC |= (1 << DDC3);
	
	// Realiza movimientos con servos
	SERVO_MOTORES();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t EEPROM1[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)EEPROM1, (void*)UNO_address, sizeof(EEPROM1));
}

void readUNO (){

	PORTB &= ~(1 << DDB4);
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC &= ~(1 << DDC3);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t readEUNO[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)readEUNO, (const void*)UNO_address, sizeof(readEUNO));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = readEUNO[0];
	dutyCycle2 = readEUNO[1];
	dutyCycle3 = readEUNO[2];
	dutyCycle4 = readEUNO[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

void writeDOS (void){
	// Configura los pines correspondientes para DOS
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC |= (1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC |= (1 << DDC3);
	
	// Realiza movimientos con servos
	SERVO_MOTORES();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t EEPROM2[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)EEPROM2, (void*)DOS_address, sizeof(EEPROM2));
}

void readDOS (){

	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC |= (1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC &= ~(1 << DDC3);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t readEDOS[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)readEDOS, (const void*)DOS_address, sizeof(readEDOS));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = readEDOS[0];
	dutyCycle2 = readEDOS[1];
	dutyCycle3 = readEDOS[2];
	dutyCycle4 = readEDOS[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

void writeTRES (void){
	// Configura los pines correspondientes para TRES
	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC |= (1 << DDC2);
	PORTC |= (1 << DDC3);
	
	// Realiza movimientos con servos
	SERVO_MOTORES();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t EEPROM3[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)EEPROM3, (void*)TRES_address, sizeof(EEPROM3));
}

void readTRES (){

	PORTB &= ~(1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC |= (1 << DDC2);
	PORTC &= ~(1 << DDC3);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t readETRES[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)readETRES, (const void*)TRES_address, sizeof(readETRES));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = readETRES[0];
	dutyCycle2 = readETRES[1];
	dutyCycle3 = readETRES[2];
	dutyCycle4 = readETRES[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

void writeCUATRO (void){
	// Configura los pines correspondientes para UNO
	PORTB |= (1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC |= (1 << DDC3);
	
	// Realiza movimientos con servos
	SERVO_MOTORES();
	
	// Lee los valores de los canales ADC y los almacena en un arreglo
	dutyCycle1 = valorADC(6);
	dutyCycle2 = valorADC(7);
	dutyCycle3 = valorADC(5);
	dutyCycle4 = valorADC(4);
	
	uint16_t EEPROM4[4] = {dutyCycle1, dutyCycle2, dutyCycle3, dutyCycle4};
	
	// Escribe el arreglo en la memoria EEPROM
	eeprom_write_block((const void*)EEPROM4, (void*)CUATRO_address, sizeof(EEPROM4));
}

void readCUATRO (){

	PORTB |= (1 << DDB4);
	PORTC &= ~(1 << DDC0);
	PORTC &= ~(1 << DDC1);
	PORTC &= ~(1 << DDC2);
	PORTC &= ~(1 << DDC3);
	
	// Arreglo para almacenar los valores leídos de la memoria EEPROM
	uint16_t readECUATRO[4];
	
	// Lee los datos de la memoria EEPROM y los almacena en el arreglo
	eeprom_read_block((void*)readECUATRO, (const void*)CUATRO_address, sizeof(readECUATRO));
	
	// Asigna los valores leídos a las variables dutyCycle
	dutyCycle1 = readECUATRO[0];
	dutyCycle2 = readECUATRO[1];
	dutyCycle3 = readECUATRO[2];
	dutyCycle4 = readECUATRO[3];
	
	// Actualiza los ciclos de trabajo de los servos con los valores leídos
	updateDCA(dutyCycle1/6);
	_delay_ms(10);
	updateDCB(dutyCycle2/6);
	_delay_ms(10);
	updateDCA1(dutyCycle3/6);
	_delay_ms(10);
	updateDCB1(dutyCycle4/6);
	_delay_ms(10);
}

void writeTextUART(char* texto){
	uint8_t i;
	for(i=0; texto[i]!='\n'; i++){ // Iterar hasta encontrar el carácter de nueva línea
		while(!(UCSR0A & (1<<UDRE0))); // Esperar a que el buffer de transmisión esté vacío
		UDR0 = texto[i]; // Transmitir carácter por carácter
	}
}

ISR(USART_RX_vect){
	
	bufferRX = UDR0;
	buffer_RX22 = UDR0;
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = bufferRX;
	UDR0 = buffer_RX22;
}

