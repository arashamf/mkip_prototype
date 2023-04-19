#ifndef MAIN_LCD1602_H_
#define MAIN_LCD1602_H_
//--------------------------------------------------------------------------//
#include "main.h"

//--------------------------------------------------------------------------//
#define e_set() WriteByte_I2C_LCD(portlcd|=0x04) 		 //установка линии Е в 1
#define e_reset() WriteByte_I2C_LCD(portlcd&=~0x04) 	//установка линии Е в 0
#define rs_set() WriteByte_I2C_LCD(portlcd|=0x01)  	//установка линии RS в 1
#define rs_reset() WriteByte_I2C_LCD(portlcd&=~0x01) 	//установка линии RS в 0
#define setled() WriteByte_I2C_LCD(portlcd|=0x08) 	//установка линии BL в 1
#define setwrite() WriteByte_I2C_LCD(portlcd&=~0x02) //установка линии RW в 0
#define setread() WriteByte_I2C_LCD(portlcd|=0x02) 	//установка линии RW в 1
//--------------------------------------------------------------------------//

//--------------------------------------------------------------------------//
void WriteByte_I2C_LCD(uint8_t );
void sendhalfbyte(uint8_t );
void sendbyte(uint8_t , uint8_t );
void LCD_SetPos(uint8_t , uint8_t );
void LCD_String(char* st);
void LCD_ini(void);
void clear_LCD1602 (void);
//--------------------------------------------------------------------------//

extern char str_LCD [];
#endif /* MAIN_LCD1602_H_ */
