#include "lcd1602.h"
//#include "i2c.h"
#include "lib_delay.h"

//-------------------------------------------------------------------------------------------------------------//
char str_LCD [16];
uint8_t portlcd; //ячейка для хранения данных порта микросхемы расширения

//-------------------------------------------------------------------------------------------------------------//
 void WriteByte_I2C_LCD(uint8_t bt)
{
	uint8_t buffer_LCD[1]={0};
	buffer_LCD [0] = bt;
//	HAL_I2C_Master_Transmit(&hi2c2,(uint16_t) 0x4E, buffer_LCD,1,1000);
} 

//-------------------------------------------------------------------------------------------------------------//
void sendhalfbyte(uint8_t c)
{
	c<<=4;
	e_set();//включаем линию E
	delay_us(50);
	WriteByte_I2C_LCD (portlcd|c);
	e_reset();//выключаем линию E
	delay_us (50);
}

//-------------------------------------------------------------------------------------------------------------//
void sendbyte(uint8_t c, uint8_t mode)
{
	uint8_t hc=0;
	if(mode==0)
		rs_reset();
	else
		rs_set();
	hc = c>>4;
	sendhalfbyte(hc);
	sendhalfbyte(c);
}

//------------------------------------------установка позиции курсора------------------------------------------//
void LCD_SetPos(uint8_t x_column, uint8_t y_line)
{
	uint32_t current_pos; //буффер текущей позиции курсора
	if (x_column > 15) //не больше 16 символов в строке
		x_column = 0;
  switch( y_line)
  {
    case 0: //1 строка
      current_pos = x_column|0x80;
      break;
		
    case 1: //2 строка
      current_pos = (0x40+x_column)|0x80;
      break;
		
  /*  case 2: //3 строка
      CUR_POS = (0x14+x_column)|0x80;
      break;
		
    case 3:  //4 строка
      CUR_POS = (0x54+x_column)|0x80;
      break;*/
		
		default:
			current_pos = 0x80;
			break;
  }
  sendbyte(current_pos,0);
}

//-------------------------------------------------------------------------------------------------------------//
void LCD_String(char* st)
{
  uint8_t i=0;

  while (*(st+i) != 0)
  {
    sendbyte(*(st+i), 1);
    i++;
  }
}

//-------------------------------------------------------------------------------------------------------------//
void LCD_ini(void)
{
	// интерфейс - 4-битный, строк - 2, шрифт - 5х7 точек
	WriteByte_I2C_LCD (0);
	setwrite();			//запись
	delay_us (10000); //10 мс
	delay_us (10000); //10 мс
	sendhalfbyte(0x03);
	delay_us(450);
	sendhalfbyte(0x03);
	delay_us(450);
	sendhalfbyte(0x03);
	delay_us(450);
	sendhalfbyte(0x02);
	sendbyte(0x28,0); 		//режим 4 бит, 2 линии
	sendbyte(0x08,0);			//выключение дисплея
	delay_us(1000); 			//1 мс
	sendbyte(0x01,0);			//уборка мусора
	delay_us(2000);			//2 мс
	sendbyte(0x06,0);		// ориентация строки слева направо
	delay_us (1000); 		//1 мс
	sendbyte(0x0C,0);		//включение дисплея (D=1),  без отображения курсора
	sendbyte(0x02,0);		//курсор на место
	sendbyte(0X80,0);		//SET POS LINE 0
	delay_us(2000); //2 мс
	setled();				//подсветка
}

//-----------------------------------------------очистка дисплея-----------------------------------------------//
void clear_LCD1602 (void)
{
	char buffer[16] ;
	uint8_t count = 0;

	for (count = 0; count < 16; count++)
	{
		buffer[count] = ' '; //заполнение буффера символами пробела
	}

	for (count = 0; count < 2; count++)
	{
		LCD_SetPos(0, count);
		LCD_String (buffer);
	}
}
//-------------------------------------------------------------------------------------------------------------//
