#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>


#define UBRR 207 // 16MHz -> Baud rate 9600, U2X = 1
#define MAX_CNT 1000

class Gyro {
private:
  volatile int g_cnt;
  volatile unsigned char g_buffer[12];
  volatile int g_gyro_raw[3];
  volatile int g_acc_raw[3];
  volatile double g_gyro[3];
  volatile double g_acc[3];

public:
  Gyro() {
    g_cnt = 0;
    memset(g_buffer, 0, sizeof(g_buffer));
    memset(g_gyro_raw, 0, sizeof(g_gyro_raw));
    memset(g_acc_raw, 0, sizeof(g_acc_raw));
    memset(g_gyro, 0, sizeof(g_gyro));
    memset(g_acc, 0, sizeof(g_acc));
  }

  void USART0_TX(unsigned char data);
  unsigned char USART0_RX(void);
  void USART0_NUM(int nNum);
  void USART0_TX_String(const char *str);
  void USART0_TX_Float(double number, int precision);

  void set_TWI(void);
  unsigned char twi_read(unsigned char addr);
  int twi_write(unsigned char addr, char data);

  void set_MPU6050();
  void get_MPU6050();

  void set_Timer();
  void set_USART0();

  static void timerISR();
};

// USART functions
void Gyro::USART0_TX(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)))
    ; // UDRE = 1 -> Buffer empty -> ready to write
  UDR0 = data;
}

unsigned char Gyro::USART0_RX(void) {
  while (!(UCSR0A & (1 << RXC0)))
    ;
  return UDR0;
}

void Gyro::USART0_NUM(int nNum) {
  if (nNum == 0) {
    USART0_TX('0');
    return;
  }

  char buffer[10];
  int index = 0;

  if (nNum < 0) {
    USART0_TX('-');
    nNum = -nNum;
  }

  while (nNum > 0) {
    buffer[index++] = (nNum % 10) + '0';
    nNum /= 10;
  }

  for (int i = index - 1; i >= 0; i--) {
    USART0_TX(buffer[i]);
  }
}

void Gyro::USART0_TX_String(const char *str) {
  while (*str) {
    USART0_TX(*str++);
  }
}

void Gyro::USART0_TX_Float(double number, int precision) {
  if (number < 0) {
    USART0_TX('-');
    number = -number;
  }

  int integerPart = (int)number;
  USART0_NUM(integerPart);

  USART0_TX('.');

  double fraction = number - integerPart;
  for (int i = 0; i < precision; i++) {
    fraction *= 10;
    int digit = (int)fraction;
    USART0_TX(digit + '0');
    fraction -= digit;
  }
}

// MPU6050 and I2C functions
void Gyro::set_TWI(void) {
  TWCR = (1 << TWEN);
  TWBR = 12; // 400 kHz
}

unsigned char Gyro::twi_read(unsigned char addr) {
  TWCR = 0x00;
  int cnt;
  unsigned char a_data = 0;

  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Start
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x08) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = 0xD0;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x18) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = addr;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x28) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x10) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = 0xD1;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x40) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x58) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  a_data = TWDR;
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  _delay_us(50);

  return a_data;
}

int Gyro::twi_write(unsigned char addr, char data) {
  int cnt;
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x08) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = 0xD0;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x18) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = addr;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x28) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  cnt = 0;
  while (!(TWCR & (1 << TWINT))) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }
  while ((TWSR & 0xF8) != 0x28) {
    cnt++;
    if (cnt > MAX_CNT)
      return 0;
  }

  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  _delay_us(50);

  return 1;
}

void Gyro::set_MPU6050() {
  _delay_ms(150);
  twi_write(0x6B, 0x00);
  twi_write(0x1A, 0x01);
  twi_write(0x19, 0x07);
  twi_write(0x1B, 0x00);
  twi_write(0x1C, 0x00);
}

void Gyro::get_MPU6050() {
  g_buffer[0] = twi_read(0x3B);
  g_buffer[1] = twi_read(0x3C);
  g_buffer[2] = twi_read(0x3D);
  g_buffer[3] = twi_read(0x3E);
  g_buffer[4] = twi_read(0x3F);
  g_buffer[5] = twi_read(0x40);
  g_buffer[6] = twi_read(0x43);
  g_buffer[7] = twi_read(0x44);
  g_buffer[8] = twi_read(0x45);
  g_buffer[9] = twi_read(0x46);
  g_buffer[10] = twi_read(0x47);
  g_buffer[11] = twi_read(0x48);

  for (int i = 0; i < 3; i++) {
    g_acc_raw[i] = ((g_buffer[2 * i] << 8) | g_buffer[2 * i + 1]);
    g_acc[i] = g_acc_raw[i] / 16384.0;
    g_gyro_raw[i] = ((g_buffer[2 * i + 6] << 8) | g_buffer[2 * i + 7]);
    g_gyro[i] = g_gyro_raw[i] / 131.0;
  }
}

// Timer functions
void Gyro::set_Timer() {
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
  ICR1 = 9999;
  TIMSK |= (1 << TOIE1);
  sei();
}

void Gyro::timerISR() { g_cnt++; }

// USART functions
void Gyro::set_USART0() {
  UBRR0H = (unsigned char)(UBRR >> 8);
  UBRR0L = (unsigned char)UBRR;
  UCSR0A |= (1 << U2X0);                  // Double speed mode 사용
  UCSR0B = (1 << TXEN0);                  // Enable Transmit
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data format
}

// ISR
ISR(TIMER1_OVF_vect) { Gyro::timerISR(); }

int main() {
  Gyro gyro;
  gyro.set_USART0();
  gyro.set_TWI();
  gyro.set_MPU6050();
  gyro.set_Timer();

  while (1) {
    gyro.get_MPU6050();
    gyro.USART0_TX_Float(gyro.g_acc[0], 2);
    gyro.USART0_TX_String("\r\n");
    _delay_ms(1000);
  }
}
