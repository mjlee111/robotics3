#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "Filter.h"
#include "sensor.cpp"

#define AVCC 5.0

#define SOUND_VELOCITY 340UL //소리 속도 (m/sec)
volatile unsigned int distance;     //거리 변수


#define SERVO_PIN PB3
#define DIR_PIN PD2
#define STEP_PIN PD3
#define ENABLE_PIN PD4

/* ---------- Parameter ---------- */
volatile unsigned int a_adc[8];
volatile double adc_max[3] = {0.0, 0.0, 0.0};         // adc 최댓값 저장
volatile double adc_min[3] = {1000.0, 5000.0, 1000.0};   // adc 최솟값 저장
   
volatile double a_ir[3] = {0};

// LM35
#define sbi(p, m) p |= (1<<m);      // LM35 입력
#define cbi(p, m) p &= ~(1<<m);      // Coller 출력 출력
volatile double a_lm35;            // [.C] lm 센서값을 수치화
volatile double a_cm;            // [cm] psd 센서값을 수치화
volatile double a_hcm;            // [cm] 초음파 센서값을 수치화
volatile double a_mVol[8];         // adc값을 voltage로 변환


/* ---------- Timer/Counter ---------- */
void init_TC1(void);
void init_TC2(void);

/* ---------- USART ---------- */
void init_Uart(void);
void Uart_Trans(unsigned char data);
unsigned char Uart_Receive(void);
void Uart_Trans_INT(double data);

/* ---------- ADC ---------- */
void init_ADC(void);
unsigned int get_adc(int id);

void get_Vol_lm35(unsigned int adc_2);
void get_Vol_psd(unsigned int adc_3);
void get_Vol_hc(unsigned int adc_7);

void classify_IR(void);

int Fan_Control(void);

void Buzzer_control(void);

/* ---------- MOTOR ---------- */
void init_StepMotor();
void rotateMotor(int steps, int direction, int speed);

void init_Servo();
void set_Servo_Angle(uint8_t angle);
void init_TC0(void);

ISR(TIMER0_OVF_vect){
   TCNT0 = 17;
   
   int i = 0;
   for(int k = i; k < 8; k++) a_adc[k] = get_adc(k);
   
   // 초음파 -> 부저
   get_Vol_hc(a_adc[7]);
   Buzzer_control();
}

ISR(TIMER2_OVF_vect)
{
   TCNT2 = 100;
   
   int i = 0;
   for(int k = i; k < 8; k++) a_adc[k] = get_adc(k);
   
   // LM35 -> fan
   get_Vol_lm35(a_adc[2]);
   OCR1A = Fan_Control();
   
}


int main(void)
{
   
   init_TC0();
   init_TC1();
   init_TC2();
   init_Uart();
   init_ADC();
   init_StepMotor();
   init_Servo();
   
   
   // LED
   DDRA = 0xFF; // PA 핀을 입력으로 사용(LED)
   
   sbi(DDRB, 4); // OCR0 Buzzer 출력
   sbi(DDRB, 5); // OCR1A cooler 출력
   sbi(DDRB, 6); // OCR1B DC motor 출력
   sbi(DDRB, 7); // OCR1C LED 출력
   sbi(DDRF, 6); // 초음파센서 출력
   
   cbi(DDRD, 0); // button1 입력
   cbi(DDRD, 1); // button2 입력
   cbi(DDRD, 2); // button3 입력
   cbi(DDRD, 3); // button4 입력
   cbi(DDRF, 0); // 가변저항 입력
   cbi(DDRF, 1); // CDS 입력
   cbi(DDRF, 2); // LM35 입력
   cbi(DDRF, 3); // PSD 입력
   cbi(DDRF, 4); // Flex 입력
   cbi(DDRF, 5); // 압력 입력
   cbi(DDRF, 7); // 초음파센서 입력
   
   sei();
    while (1);
}

/* ---------- USART ---------- */
void init_Uart(void){
   DDRD = 0b00001000; // Enable Tx pin;
   UCSR1A = 0x00; // use as flag
   UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Enable receive and trans
   UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // TX/RX in 8bit
   
   // baud rate = 115200bps
   UBRR1L = 0;
   UBRR1H = 8;
}

void Uart_Trans(unsigned char data){
   while(!(UCSR1A & (1 << UDRE1)));
   UDR1 = data;
}

unsigned char Uart_Receive(void){
   while(!(UCSR1A & (1 << RXC1)));
   return UDR1;
}

void Uart_Trans_INT(double data){
   unsigned char int_data;
   
   int_data = (int)data / 10000   + 48;
   Uart_Trans(int_data);
   
   int_data = (int)data / 1000      + 48;
   Uart_Trans(int_data);
   
   int_data = (int)data / 100      + 48;
   Uart_Trans(int_data);
   
   int_data = (int)data / 10      + 48;
   Uart_Trans(int_data);
   
   int_data = (int)data         + 48;
   Uart_Trans(int_data);
}

/* ---------- Timer/Counter ---------- */
void init_TC0(void){
   TCCR0 = (0<<CS02)|(1<<CS01)|(1<<CS00);
   TIMSK = (1<<TOIE0);
   TCNT0 = 17;
}

void init_TC1(void){
   TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<COM1C1)|(0<<COM1C0)|(1<<WGM11)|(0<<WGM10);
   TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS12)|(0<<CS11)|(0<<CS10);
   ICR1 = 1249;
}

void init_TC2(void){
   TCCR2 = (0 << WGM21)|(0 << WGM20)|(0 << COM21)|(0 << COM20)|(1 << CS22)|(0 << CS21)|(1 << CS20);
   TIMSK = (1 << TOIE2);
   TCNT2 = 100;   // 2ms
}

/* ---------- ADC ---------- */
void init_ADC(void){
   ADMUX = 0b00000000;
   ADCSRA = 0b10000111;
}

unsigned int get_adc(int id){
   ADMUX = 0x40 | id;   // id에 따라 사용할 ADC 채널 결정
   cbi(ADCSRA, ADIF);   // ADC 변환이 끝난 후 1로 setting된 ADIF를 0으로 clear
   sbi(ADCSRA, ADSC);   // ADC 변환 시작
   while(!(ADCSR & (1 << ADIF)));   // ADC 변환이 끝날때까지 기다림
   return ADC;
}

void get_Vol_lm35(unsigned int adc_2){
   // ADC 수치 변환
   a_mVol[2] = (double)adc_2 * (AVCC / 1024.0);
   a_lm35 = 100 * a_mVol[2];
   a_lm35 = iir_lpf_lm35(a_lm35);
   //USART0_NUM(a_Lux); USART0_TX_vect(',');
}

void get_Vol_psd(unsigned int adc_3){
   a_mVol[3] =(double)adc_3 * (AVCC/1024.0);
   a_cm = 27.717*pow(a_mVol[3], -1.433);
   if(a_cm > 50) a_cm = 50;
   // a_cm = iir_lpf_psd(a_cm);
   //USART0_NUM(a_cm); USART0_TX_vect(',');
}

void get_Vol_hc(unsigned int adc_7){
   a_mVol[7] = (double)adc_7 * (AVCC/1024.0);
   a_hcm = a_mVol[7];
   distance = (unsigned int)(SOUND_VELOCITY * (TCNT0*4/2)/1000);
   //USART0_NUM(distance); USART0_TX_vect(',');
}

int Fan_Control(void){
   // 온도가 100도 이상이면 PAN 속도를 최대로 올리고
   // 그 값 이해일 때는 100도에 대한 비율로 PAN의 속도를 조절한다.
   unsigned int pan_power = 0;
   
   if(a_lm35 < 100 && a_lm35 > 50){
      pan_power = ICR1*(a_lm35/100);
   }
   else if(a_lm35 > 100) pan_power = ICR1;
   
   return pan_power;
}

void Buzzer_control(void){
   if(distance<300){     //30cm 이내 장애물
      for(int i=0;i<5;i++){    //연속하여 "삐~" 지속
         PORTB=0x10;
         _delay_ms(1);
         PORTB=0x00;
         _delay_ms(1);
      }
   }
   else if(distance<600){     //60cm 이내 장애물
      for(int i=0;i<50;i++){     //0.1초 동안 "삐~"
         PORTB=0x10;
         _delay_ms(1);
         PORTB=0x00;
         _delay_ms(1);
      }
      _delay_ms(100);     //0.1초 동안 묵음
   }
   else if(distance<1000){      //1m 이내 장애물
      for(int i=0;i<250;i++){     //0.5초동안 "삐~"
         PORTB=0x10;
         _delay_ms(1);
         PORTB=0x00;
         _delay_ms(1);
      }
      _delay_ms(300);      //0.3초 동안 묵음
   }
   else;     //1m 이내 장애물 없을 시 버저 울리지 않음

}



/* ---------- MOTOR ---------- */
void init_StepMotor() {
   DDRD |= (1 << DIR_PIN) | (1 << STEP_PIN) | (1 << ENABLE_PIN);

   PORTD |= (1 << ENABLE_PIN);
}

void rotateMotor(int steps, int direction, int speed) {
   if (direction) {
      PORTD |= (1 << DIR_PIN); // 시계 방향
      } else {
      PORTD &= ~(1 << DIR_PIN); // 반시계 방향
   }

   PORTD &= ~(1 << ENABLE_PIN);

   for (int i = 0; i < steps; i++) {
      PORTD |= (1 << STEP_PIN);
      _delay_us(speed);
      
      PORTD &= ~(1 << STEP_PIN);
      _delay_us(speed);
   }

   PORTD |= (1 << ENABLE_PIN);
}

void init_Servo() {
   DDRB |= (1 << SERVO_PIN);

   TCCR1A |= (1 << WGM11) | (1 << COM1A1);
   TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);
   ICR1 = 19999;

   OCR1A = 1000;

   sei();
}

void set_Servo_Angle(uint8_t angle) {
   OCR1A = 1000 + ((angle * 1000) / 180);
}

