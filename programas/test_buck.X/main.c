/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 15, 2023, 7:55 PM
 */


#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>


// USART  variables
#define UART_BUFFER 64
uint8_t writePointer = 0;
uint8_t readPointer = 0;
char uartBuffer[UART_BUFFER];



void send_data(const char* data, uint8_t num);





int main(void) {
    //  ------ CONFIGURACIÓN UART -----------------
    
    UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad
    
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
   
    UBRR0 = 51; //19200 baud rate, at 8MHz clock
    
    SREG |= 1<<7;
    
    send_data("Hola Inicio\n",11);
    while (1) {
        
        send_data("Hola\n",5);
        _delay_ms(10);    
        
    }
}

void send_data(const char* data, uint8_t num){
    
    
    
    for (int i=0; i<num ; i++ ) {
        
        /*while( !( UCSR0A&(1<<UDRE0))  );
        
        UDR0 = data[i];*/
        uartBuffer[writePointer] = data[i];
        writePointer=(writePointer+1)%UART_BUFFER;
    }
    
   UCSR0B |= 1<<UDRIE0; // HAbilitar interrupcion
    return;
}

// ---------------- ISR ---------------------------

void __interrupt(USART_UDRE_vect_num) udreInt(void){
    
    if( writePointer == readPointer ){
        UCSR0B&=~(1<<UDRIE0);//desabilitar interrupcion   
    }
    else{
        UDR0 = uartBuffer[readPointer];
        readPointer = (readPointer+1)%64;
    }
    
    
    
}