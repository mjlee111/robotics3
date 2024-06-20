/*
 * TermProject
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

#include "stepmotor.h"
#include "servomotor.h"
#include "lcd.h"
#include "filters.h"

// Timer cnt
void cntTimerInit();
int cnt0 = 0;
int cnt0_s = 0;

// Step Motor L4988 Motor Driver
stepmotor stepmotor_;
//------------------------------------

// Servo Motor
servomotor servomotor_;
//------------------------------------

// LCD
lcd lcd_;
//------------------------------------

// Filters
double lpf_alpha = 0.1;
LPF lpf(lpf_alpha);

double hpf_alpha = 0.1;
HPF hpf(hpf_alpha);

double process_noise = 1e-5;
double measurement_noise = 1e-1;
double estimated_error = 1.0;
double initial_value = 0.0;
KalmanFilter kf(process_noise, measurement_noise, estimated_error, initial_value);
//------------------------------------

int main(void) {  
  sei();
  return 0;
}

// Timer cnt
void cntTimerInit() {
  // 8ms Timer
  TCNT0 = 131;
  TCCR0 = 0x07;
  TIMSK = 0x01;
}

ISR(TIMER0_OVF_vect) {
  cnt0++;
  if (cnt0 == 125) {
    cnt0_s++;
    cnt0 = 0;
  }
  if (cnt0_s == 2147483646) {
    cnt0_s = 0;
  }
  TCNT0 = 131;
}
//------------------------------------

