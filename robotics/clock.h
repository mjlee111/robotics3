#ifndef CLOCK_H
#define CLOCK_H
/*
 *
 * ATmega128 Main Timer library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

class clock
{
private:
public:
    clock();
    ~clock();

    void cntTimerInit();
    int cnt0 = 0;
    int cnt0_s = 0;
};

clock::clock(){
    cntTimerInit();
}

clock::~clock(){
}

void clock::cntTimerInit(){
    // 8ms Timer
    TCNT0 = 131;
    TCCR0 = 0x07;
    TIMSK = 0x01;
}


#endif