#ifndef LCD_H
#define LCD_H
/*
 *
 * ATmega128 LCD library
 *
 * Created: 2024-06-19
 * Author : LeeMyeongJin
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#ifndef __Text_Lcd_H
#define __Text_Lcd_H

#define LCD_PORT			PORTC
#define LCD_PORT_SETUP()	DDRC = 0xFF

#define FUNCSET		0x28	//Function set		0010 1000
#define ENTMODE		0x06	//Entry Mode Set	0000 0110
#define ALLCLR		0x01	//All Clear			0000 0001
#define DISPON		0x0c	//Display On		0000 1100
#define LINE1		0x80	//1st line Move		1000 0000
#define LINE2		0xc0	//2nd line Move		1100 0000

#define U8 unsigned char
#define U16 unsigned int

class lcd
{
private:
    /* data */
public:
    lcd(/* args */);
    ~lcd();

    void lcdInit(void);
    void lcdClear(void);
    void lcdString(U8 line, U8 col, char  *str);
    void lcdNumber(U8 line, U8 col, int num);
    void lcdCommand(U8 byte);
    void lcdData(U8 byte);
    void lcdDisplayPosition( U8 line, U8 col );
    void Busy(void);
    void delay_us(U16 time_us);
};

lcd::lcd(/* args */)
{
	lcdInit();
}

lcd::~lcd()
{
}

void lcd::lcdInit(void){
    _delay_ms(30);
	LCD_PORT_SETUP();
	LCD_PORT &= 0xFB;	//E = 0
	_delay_ms(15);
	
	lcdCommand(0x20);
	_delay_ms(5);
	
	lcdCommand(0x20);
	delay_us(200);
	
	lcdCommand(0x20);
	lcdCommand(FUNCSET);
	lcdCommand(DISPON);
	lcdCommand(ALLCLR);
	lcdCommand(ENTMODE);
	
	lcdString(0,0,"Init OK");
}

void lcd::lcdClear(void){
    lcdCommand(ALLCLR);
}

void lcd::lcdString(U8 line, U8 col, char  *str){
    char  *pStr = 0;
	
	lcdDisplayPosition( line, col );
	pStr = str;
	while(*pStr)
	{
		lcdData(*pStr++);
	}
}

void lcd::lcdNumber(U8 line, U8 col, int num){
	char byte[100] = {0, };
	
	sprintf(byte,"%d",num);
	
	lcdString(line, col, byte);
}

void lcd::lcdCommand(U8 byte){
	busy();
	//인스트럭션 상위 바이트
	LCD_PORT = (byte & 0xf0); //data
	LCD_PORT &= 0xfe;  //RS = 0
	LCD_PORT &= 0xfd;  //RW = 0
	delay_us(1);
	LCD_PORT |= 0x04;  //E = 1
	delay_us(1);
	LCD_PORT &= 0xfb;  //E = 0
	//instruction low byte
	LCD_PORT = ((byte<<4) & 0xf0); //data
	LCD_PORT &= 0xfe;  //RS = 0
	LCD_PORT &= 0xfd;  //RW = 0
	delay_us(1);
	LCD_PORT |= 0x04;  //E = 1
	delay_us(1);
	LCD_PORT &= 0xfb;  //E = 0
}

void lcd::lcdData(U8 byte){
    busy();
	//data high byte
	LCD_PORT = (byte & 0xF0); //data
	LCD_PORT |= 0x01;  //RS = 1
	LCD_PORT &= 0xfd;  //RW = 0
	delay_us(1);
	LCD_PORT |= 0x04;  //E = 1
	delay_us(1);
	LCD_PORT &= 0xfb;  //E = 0
	//data low byte
	LCD_PORT = ((byte<<4) & 0xF0); //data
	LCD_PORT |= 0x01;  //RS = 1
	LCD_PORT &= 0xfd;  //RW = 0
	delay_us(1);
	LCD_PORT |= 0x04;  //E = 1
	delay_us(1);
	LCD_PORT &= 0xfb;  //E = 0
}

void lcd::lcdDisplayPosition( U8 line, U8 col )
{
	if(line == 0)		lcdCommand( LINE1 + col );
	else				lcdCommand( LINE2 + col );
}

void lcd::busy(void)
{
	_delay_ms(2);
}

void lcd::delay_us(U16 time_us)
{
	register U16 i;
	
	for(i = 0; i < time_us; i++)				/* 4 cycle +				*/
	{
		asm volatile(" PUSH  R0 ");				/* 2 cycle +				*/
		asm volatile(" POP   R0 ");				/* 2 cycle +				*/
		asm volatile(" PUSH  R0 ");				/* 2 cycle +				*/
		asm volatile(" POP   R0 ");				/* 2 cycle +				*/
		asm volatile(" PUSH  R0 ");				/* 2 cycle +				*/
		asm volatile(" POP   R0 ");				/* 2 cycle    =  16 cycle		*/
	}
}

#endif