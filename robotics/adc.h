#ifndef ADC_H
#define ADC_H

/*
 *
 * ATmega128 ADC library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

class adc
{
private:
public:
    adc();
    ~adc();
    
    void ADCinit(void);
    uint16_t ADCRead(uint8_t channel);
};

adc::adc(){
    ADCinit();
}

adc::~adc(){
}

void adc::ADCinit(void){
    // AVCC with external capacitor at AREF pin
    ADMUX = (1 << REFS0);
    // ADC enable, prescaler = 128
    ADCSRA = (1 << ADEN) | (7 << ADPS0);
}

uint16_t adc::ADCRead(uint8_t channel){
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

#endif
