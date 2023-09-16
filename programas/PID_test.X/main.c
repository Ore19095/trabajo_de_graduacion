/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 2, 2023, 3:38 PM
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// Constants for PID controller
#define KP 1.0
#define KI 0.1
#define KD 0.01

// Variables for PID controller
double setpoint = 0.0;
double current = 0.0;
double previous_error = 0.0;
double integral = 0.0;

// ADC channel for current measurement
#define ADC_CHANNEL 0

// PWM Configuration
void PWM_Init() {
    // Set Timer 1 in Fast PWM mode, non-inverted output
    TCCR1A = (1 << WGM11) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // Prescaler = 1
    ICR1 = 255; // TOP value for 8-bit PWM
    DDRB |= (1 << PB1); // Set OC1A (Pin 9) as an output
}

// ADC Configuration
void ADC_Init() {
    ADMUX = (1 << REFS0) | (ADC_CHANNEL & 0x07); // AVCC as reference, select ADC channel
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler = 64
}

// Read ADC value
uint16_t ADC_Read() {
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// Timer 2 Configuration
void Timer2_Init() {
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler = 1024
    OCR2A = 156; // Set compare value for a 10ms interrupt
    TIMSK2 = (1 << OCIE2A); // Enable Timer 2 compare match interrupt
}

// PID Controller function
double PID_Controller() {
    double error = setpoint - current;
    integral += error;
    double derivative = error - previous_error;
    previous_error = error;
    double output = KP * error + KI * integral + KD * derivative;
    
    // Limit the output to the PWM range
    if (output > 255.0) {
        output = 255.0;
    } else if (output < 0.0) {
        output = 0.0;
    }
    
    return output;
}

int main() {
    // Initialize peripherals
    PWM_Init();
    ADC_Init();
    Timer2_Init();
    
    sei(); // Enable global interrupts
    
    while (1) {
        // Read current from ADC
        current = (double)ADC_Read();
        
        // Calculate PID output
        double pid_output = PID_Controller();
        
        // Set PWM duty cycle based on PID output
        OCR1A = (uint8_t)pid_output;
    }
    
    return 0;
}