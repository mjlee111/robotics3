#ifndef TIMER_H
#define TIMER_H

#include <avr/io.h>
#include <util/delay.h>

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
	    TCCR0 |= (1 << WGM01);
	    TCCR0 &= ~(1 << WGM00);
	    TCCR0 |= (1 << CS02) | (1 << CS00);
	    OCR0 = 62;
	    TIMSK |= (1 << OCIE0);
}

#endif // TIMER_H
