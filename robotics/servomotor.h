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

ServoMotor::ServoMotor(uint8_t pwm) {
    init(pwm);
}

ServoMotor::~ServoMotor() {
    // 아무 정리 작업이 필요 없을 때
}

void ServoMotor::init(uint8_t pwm) {
    // PWM 핀 설정
    DDRD |= (1 << pwm);

    // 타이머 설정 : Fast PWM, 비반전 모드, 프리스케일러 8
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // ICR1 설정 : 20ms 주기 (50Hz)
    ICR1 = 19999;

    // 초기 서보 위치 0도로 설정 (1ms 펄스 폭)
    OCR1A = 1000;
    degree = 0;
}

void ServoMotor::setAngle(uint8_t angle) {
    // 각도에 따라 PWM 듀티 사이클 설정 (0도: 1ms, 180도: 2ms)
    OCR1A = 1000 + ((angle * 1000) / 180);
    degree = angle;
}

int ServoMotor::getAngle() {
    return degree;
}

#endif
