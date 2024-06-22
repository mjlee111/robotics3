#ifndef SK6812_H
#define SK6812_H

/*
 *
 * ATmega128 SK6812 RGBW LED library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>

#define SK6812_PIN PA6

class SK6812
{
private:
    void sendBit(uint8_t bit);
    void sendByte(uint8_t byte);
public:
    SK6812();
    ~SK6812();
    
    void init(void);
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
};

SK6812::SK6812(){
    init();
}

SK6812::~SK6812(){
}

void SK6812::init(void){
    // SK6812 핀을 출력으로 설정
    DDRA |= (1 << SK6812_PIN);
}

void SK6812::sendBit(uint8_t bit){
    if(bit) {
        PORTA |= (1 << SK6812_PIN);
        _delay_us(0.8);
        PORTA &= ~(1 << SK6812_PIN);
        _delay_us(0.45);
    } else {
        PORTA |= (1 << SK6812_PIN);
        _delay_us(0.4);
        PORTA &= ~(1 << SK6812_PIN);
        _delay_us(0.85);
    }
}

void SK6812::sendByte(uint8_t byte){
    for(uint8_t i = 0; i < 8; i++) {
        sendBit(byte & (1 << (7 - i)));
    }
}

void SK6812::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w){
    sendByte(g);
    sendByte(r);
    sendByte(b);
    sendByte(w);
    _delay_us(50);  // Reset time
}

#endif
