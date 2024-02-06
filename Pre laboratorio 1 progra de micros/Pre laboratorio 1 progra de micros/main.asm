;******************************************************************************
; Universidad del valle de Guatemala
; IE2023: Programacion de microcontroladores
; Autor: Gabriel Sanchez
; Proyecto: Lab 1
; Archivo: Pre laboratorio 1 progra de micros.asm
; Hardware: ATMEGA328P
; Created: 29/01/2024 21:22:34
;
;******************************************************************************
; ENCABEZADO
;******************************************************************************

.include "M328PDEF.inc"
.cseg
.org 0x00

;******************************************************************************
;Stack
;******************************************************************************
LDI R16, LOW(RAMEND)
OUT SPL, R16 
LDI R17, HIGH(RAMEND)
OUT SPH, R17
;******************************************************************************
;Configuracion
;******************************************************************************
Setup:
	LDI R24, (1 << CLKPCE)
	STS CLKPR, R24

	LDI R24, 0b0000_0100
	STS CLKPR, R24 ;Frecuencia de 1Mhz

	LDI R19, 0b1111_1111
	OUT DDRD, R19 ;Establece los pines de salida, en d son todos
	;Se deja como comentario por si hace falta
	/*LDI R30, 0b1000_0000
	OUT PORTD, R30 ;Apaga los pines*/

	LDI R16, 0b0111_1111
	OUT PORTC, R16

	LDI R19, 0b0000_0000
	OUT DDRC, R19 
	/*Establece los pines de salida. 
	en c todos los pines son de entrada asi que todo esta en 0*/
	;Se deja como comentario por si hace falta
	;CBI PORTC, 0x7 ;Apaga los pines
	
	SBI PORTB, PB5
	LDI R19, 0b0001_1111
	OUT DDRB, R19
	
	 CBI PORTB, PB4
	 CBI PORTB, PB3
	 CBI PORTB, PB2
	 CBI PORTB, PB1
	 CBI PORTB, PB0
	/*Establece los pines de salida, en B los pines de 0 a 4 
	son de salida y el resto son de entrada*/
	;Le cambie de CBI a SBI para probar
	;Se deja como comentario por si hace falta
	;CBI PORTB, PB5 ;Apaga los pines
	CLR R20

	LDI R20, 0b0000_0000 
	/*Se establece R20 como base para el 
	primer contador. Esta en 0.*/
	LDI R21, 0b0000_0000 
	/*Se establece R21 como base para el 
	segundo contador. Esta en 0.*/
	LDI R22, 0b0000_0000 
	/*Se establece R22 como el segmento que sumara. Esta en 0.*/
	LDI R25, 0b0000_0000
	LDI R30, 0b0000_0000
	SBI PORTB, PB0

loop:
;Se carga cada seccion de entradas en un registro independiente
	
	
	IN R16, PINC

	SBRS R16, PC3
	SBI PINB, PB0
	//JMP Delaybounce
	
	;Boton de aumento del contador 1
	SBIC PINB, PB5
	CALL aumento_1
	SBIC PINB, PB5
	JMP Delaybounce

;Boton de decremento del contador 1
	SBIC PINC, PC0
	CALL decremento_1
	SBIC PINC, PC0
    JMP Delaybounce

;Boton de aumento del contador 2
	SBIC PINC, PC1
	CALL aumento_2
	SBIC PINC, PC1
	JMP Delaybounce

;Boton de decremento del contador 2
	SBIC PINC, PC2
	CALL decremento_2	
	SBIC PINC, PC2
	RJMP Delaybounce

;Boton de suma de los contadores
	SBIC PINC, PC3
	CALL sumador
	SBIC PINC, PC3
	RJMP Delaybounce

	CALL LEDS_1
	CALL LEDS_2
	CALL ovrflw

    rjmp loop

;******************************************************************************
;Subrutinas
;******************************************************************************

;Enciende un led cuando esta por llenarse
Ovrflw:
	SBI PORTD, PD1
	RET

;Apaga el led cuando no esta lleno
ledoff:
	CBI PORTD, PD1
	RET

aumento_1: ;Trabaja con R20
	INC R20

	RET

decremento_1: ;Trabaja con R20
	DEC R20

	RET

aumento_2: ;Trabaja con R21
	INC R21

	RET

decremento_2: ;Trabaja con R21
	DEC R21

	RET

sumador: ;Trabaja con R22
;Sumador de ambos registros
	ADD R22, R21
	ADD R22, R20
	BRCS ovrflw
	BRCC ledoff

;Seccion que enciende los pines del sumador
	SBRC R20, 0
	SBI PORTB, 4
	SBRC R20, 1
	SBI PORTB, 3
	SBRC R20, 2
	SBI PORTB, 2
	SBRC R20, 3
	SBI PORTB, 1
	RET

LEDS_1:
;Seccion que enciende los pines del contador 1
	SBRC R20, 0
	SBI PORTB, 4
	SBRC R20, 1
	SBI PORTB, 3
	SBRC R20, 2
	SBI PORTB, 2
	SBRC R20, 3
	SBI PORTB, 1
	RET

LEDS_2:
;Seccion que enciende los pines del contador 2
	SBRC R21, 0
	SBI PORTB, 0
	SBRC R21, 1
	SBI PORTD, 7
	SBRC R21, 2
	SBI PORTD, 6
	SBRC R21, 3
	SBI PORTD, 5
	RET

Delaybounce:
;Genera un delay de tiempo determinado
	LDI R23, 100
	delay:
		DEC R23
		BRNE delay

;Lee cada boton en orden para regresar al delay
	SBIS PINB, PB5
	RJMP Delaybounce

	SBIS PINC, PC0
	RJMP Delaybounce

	SBIS PINC, PC1
	RJMP Delaybounce

	SBIS PINC, PC2
	RJMP Delaybounce

	SBIS PINC, PC3
	RJMP Delaybounce

	JMP loop