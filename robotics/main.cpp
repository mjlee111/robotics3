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
#include "Timer.h"
#include "adc.h"

#define STEP_MOTOR_DIR PB1
#define STEP_MOTOR_STEP PB0 
#define STEP_MOTOR_ENABLE PB2

#define SERVO_PWM_PIN PD5

StepMotor stepmotor_(STEP_MOTOR_DIR, STEP_MOTOR_STEP, STEP_MOTOR_ENABLE); 
ServoMotor servomotor_(SERVO_PWM_PIN);
lcd lcd_; // PCn
adc adc_; // PAn
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

// Timer Count
Timer Timer_;
ISR(TIMER0_OVF_vect) {
  Timer_.cnt0++;
  for(int i=0 ; i < 8 ; i++) adc_.ADCRead(i);
  TCNT0 = 131;
}
//------------------------------------

int main(void) {  
  sei();
  return 0;
}

//------------------------------------

