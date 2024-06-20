#ifndef STEPMOTOR_H
#define STEPMOTOR_H
/*
 *
 * ATmega128 Step Motor library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

#define DIR_PIN PD4
#define STEP_PIN PD5
#define ENABLE_PIN PD6

class stepmotor {
private:
public:
  stepmotor();
  ~stepmotor();

  bool motor_status = false;
  
  void delay_ms(uint16_t ms);
  
  void stepMotorInit(uint8_t dir_pin, uint8_t step_pin, uint8_t enable_pin);
  void stepMotorStatus(bool init);
  void stepMotorRotate(int step, int dir, int speed);
  bool setpMotorGetStat(void);
};

stepmotor::stepmotor(){
  stepMotorInit(DIR_PIN, STEP_PIN, ENABLE_PIN);
}

stepmotor::~stepmotor(){

}

void stepmotor::delay_ms(uint16_t ms){
    for (uint16_t i = 0; i < ms; i++) {
		  _delay_us(1); // Delay 1 millisecond
	  }
}

void stepmotor::stepMotorInit(uint8_t dir_pin, uint8_t step_pin, uint8_t enable_pin) {
    // 스텝모터 핀 출력으로 설정
    DDRD |= (1 << dir_pin) | (1 << step_pin) | (1 << enable_pin);
    stepMotorStatus(false);
}

void stepmotor::stepMotorStatus(bool init) {
    // 스텝모터 활성화/비활성화
    motor_status = init;
    if (init) {
      PORTD &= ~(1 << ENABLE_PIN);
      return;
    } else if (!init) {
      PORTD |= (1 << ENABLE_PIN);
      return;
    }
}

void stepmotor::stepMotorRotate(int step, int dir, int speed) {
    // 방향 설정
    if (dir == 0) {
      PORTD |= (1 << DIR_PIN); // 시계방향
    } else if (dir == 1) {
      PORTD &= ~(1 << DIR_PIN); // 반시계방향
    }
    stepMotorStatus(true);

    for (int i = 0; i < step ; i++) {
      PORTD |= (1 << STEP_PIN);
      _delay_ms(5);
      PORTD &= ~(1 << STEP_PIN);
      _delay_ms(5);
    }

    stepMotorStatus(false);
    return;
}

bool stepmotor::setpMotorGetStat(void){
  return motor_status;
}


#endif