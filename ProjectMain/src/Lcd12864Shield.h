#ifndef __LCD12864SHIELD__H
#define __LCD12864SHIELD__H

#include "Arduino.h"


class Lcd12864Shield
{
public:
	Lcd12864Shield(unsigned char CS,unsigned char RS,unsigned char RST,unsigned char CLK,unsigned char MOSI)
	{ cs_ = CS; rs_ = RS; rst_ = RST; clk_ = CLK; mosi_ = MOSI;	
	  pinMode(cs_,OUTPUT); pinMode(rs_,OUTPUT); pinMode(rst_,OUTPUT); pinMode(clk_,OUTPUT); pinMode(mosi_,OUTPUT); }
	void initialLcd(void);
	void transferCommand(int data);
	void transferData(int data);
	void lcdAddress(unsigned char page,unsigned char column);
	void clearScreen(void);
	void drawXLine(char line);
	void drawYLine(char line);
	void drawYLine(unsigned char x,unsigned char dot1,unsigned char dot2);
	void drawWord(unsigned char x,unsigned char y,int data,char *spr);
	void drawLetter(unsigned char  x,unsigned char y,int data,char *spr);
	void drawLetter(unsigned char  x,unsigned char y,char number,int data,char *spr);	
	
public:
	unsigned char cs_,rs_,rst_,clk_,mosi_;	
};


#endif