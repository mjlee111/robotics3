#ifndef STEPMOTOR_H
#define STEPMOTOR_H

#include <avr/io.h>
#include <util/delay.h>

class StepMotor {
private:
    uint8_t dir_pin;
    uint8_t step_pin;
    uint8_t enable_pin;
    bool motor_status;

public:
    StepMotor(uint8_t dir, uint8_t step, uint8_t enable);
    ~StepMotor();

    void delay_ms(uint16_t ms);
    void init();
    void setStatus(bool enabled);
    void rotate(int steps, int dir);
    bool getStatus();
};

StepMotor::StepMotor(uint8_t dir, uint8_t step, uint8_t enable) {
    dir_pin = dir;
    step_pin = step;
    enable_pin = enable;
    init();
}

StepMotor::~StepMotor() {
}

void StepMotor::delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        _delay_ms(1); 
    }
}

void StepMotor::init() {
    // 스텝모터 핀 설정
    DDRB |= (1 << dir_pin) | (1 << step_pin) | (1 << enable_pin);
    setStatus(false); 
}

void StepMotor::setStatus(bool enabled) {
    motor_status = enabled;
    if (enabled) {
        PORTB &= ~(1 << enable_pin); 
    } else {
        PORTB |= (1 << enable_pin);
    }
}

void StepMotor::rotate(int steps, int dir) {
    if (dir == 0) {
        PORTB |= (1 << dir_pin); // 시계 방향
    } else {
        PORTB &= ~(1 << dir_pin); // 반시계 방향
    }

    setStatus(true);
    // 스텝 전송
    for (int i = 0; i < steps; i++) {
        PORTB |= (1 << step_pin);
        _delay_ms(1);
        PORTB &= ~(1 << step_pin);
        _delay_ms(1);
    }

    setStatus(false); 
}

bool StepMotor::getStatus() {
    return motor_status;
}

#endif
