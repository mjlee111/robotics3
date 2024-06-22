#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

/*
 *
 * ATmega128 Servo Motor library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

class ServoMotor {
private:
  int degree;

public:
  ServoMotor(uint8_t pwm);
  ~ServoMotor();

  void init(uint8_t pwm);
  void setAngle(uint8_t angle);
  int getAngle();
};

ServoMotor::ServoMotor(uint8_t pwm) { init(pwm); }

ServoMotor::~ServoMotor() {
  // 아무 정리 작업이 필요 없을 때
}

void ServoMotor::init(uint8_t pwm) {
  // PWM 핀 설정
  DDRD |= (1 << pwm);

  TCCR1A = (1 << COMA1) | (0 << COMA0) | (1 << WGM11) | (0 << WGM10);
  TCCR1B =
      (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10);
  ICR1 = 4999;
  OCR1A = 1000;
  degree = 0;
}

void ServoMotor::setAngle(uint8_t angle) {
  double width;
  double duty;

  width = (angle / 90.0) + 0.5;
  duty = (width / 20.0) * 100.0;

  OCR1A = int(duty / 100 * ICR1);
  return OCR1A;
  degree = angle;
}

int ServoMotor::getAngle() { return degree; }

#endif
