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

#define SERVO_PWM_PIN PB1

class servomotor
{
private:
public:
    servomotor();
    ~servomotor();

    int degree = 0;

    void servoMotorInit(uint8_t pwm_pin);
    void servoMotorSetAngle(uint8_t angle);
    int servoMotorGetAngle(void);
};

servomotor::servomotor(){
    servoMotorInit(SERVO_PWM_PIN);
}

servomotor::~servomotor(){
}

void servomotor::servoMotorInit(uint8_t pwm_pin) {
    // pwm 핀 출력 설정
    DDRB |= (1 << pwm_pin);

    // 타이머 설정 : Fast PWM, 비반전 모드, 프리스케일러 8
    TCCR1A |= (1 << WGM11) | (1 << COM1A1);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // ICR1 설정 : 20ms 주기 (50Hz)
    ICR1 = 19999;

    // 초기 서모 위치 0도로 설정 (1ms 펄스 폭)
    OCR1A = 1000;
    degree = 0;
}

void servomotor::servoMotorSetAngle(uint8_t angle){
    // 각도에 따라 PWM 듀티 사이클 설정 (0도: 1ms, 180도: 2ms)
    OCR1A = 1000 + ((angle * 1000) / 180);
    degree = angle;
}

int servomotor::servoMotorGetAngle(void){
    return degree;
}

#endif
