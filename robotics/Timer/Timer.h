#ifndef Timer_H
#define Timer_H
/*
 *
 * ATmega128 Main Timer library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

#define TIMER0_PRESCALER 1024
#define TIMER0_COMPARE_VALUE ((F_CPU / TIMER0_PRESCALER) * 0.008) - 1

class Timer
{
private:
public:
    Timer();
    ~Timer();

    void cntTimerInit();
    int cnt0 = 0;
    int cnt0_s = 0;
};

Timer::Timer(){
    cntTimerInit();
}

Timer::~Timer(){
}

void Timer::cntTimerInit(){
    // 8ms Timer
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00); 
    OCR0A = TIMER0_COMPARE_VALUE;
    TIMSK |= (1 << OCIE0A);
}


#endif