/*
 * i2c-lcd.c
 *
 */

/** Put this in the src folder **/


/** Put this in the src folder **/
#include<string.h>
#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
static void lcd_write_4bit (char val, char rs);



void lcd_write_4bit (char val, char rs)
{
  char data_u, data_l;
	_Bool en = 1;
	uint8_t data_t;

	data_u = ((val<<4)&0xf0);
	data_l = (0x08 | (en?0x04:0x00) | rs);

	data_t = data_u|data_l;  //en=1, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD, &data_t, 1, 100);
	HAL_Delay(2);

	en = 0;
	data_l = (0x08 | (en?0x04:0x00) | rs);

	data_t = data_u|data_l;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD, &data_t, 1, 100);
	HAL_Delay(2);
}


void lcd_send_cmd (char cmd)
{
  char data_u, data_l;

	data_u = (cmd >> 4);
	lcd_write_4bit(data_u, 0);

	data_l = (cmd&0xf);
	lcd_write_4bit(data_l, 0);
}


void lcd_send_data (char data)
{
  char data_u, data_l;

	data_u = (data >> 4);
	lcd_write_4bit(data_u, 1);

	data_l = (data & 0xf);
	lcd_write_4bit(data_l, 1);
}

void lcd_clear (void)
{
	lcd_send_cmd (LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	HAL_Delay(10);  // this command takes a long time!
}

/*
void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}
*/
void lcd_init (void)
{
	HAL_Delay(1000);
	lcd_write_4bit (0x03, 0);
	HAL_Delay(10);
	lcd_write_4bit (0x03, 0);
	HAL_Delay(10);
	lcd_write_4bit (0x03, 0);
	HAL_Delay(10);

	lcd_write_4bit (0x02, 0);
	HAL_Delay(10);

	lcd_write_4bit (0x02, 0);
	HAL_Delay(10);
	lcd_write_4bit (0x08, 0);

	HAL_Delay(10);
	lcd_write_4bit (0x00, 0);
	HAL_Delay(10);
	lcd_write_4bit (0x0C, 0);

	HAL_Delay(10);
	lcd_write_4bit (0x00, 0);
	HAL_Delay(10);
	lcd_write_4bit (0x06, 0);
	HAL_Delay(200);
}


void lcd_set_cursor (uint8_t row, uint8_t col)
{
	uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

	if (row > 3 || col > 19)
		return;

	lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_goto_line (uint8_t line)
{
	if (line < 1 || line > 4)
		return;

	lcd_set_cursor(line-1, 0);
}

void lcd_send_string (char *str)
{
	uint8_t len = strlen(str);
	if (len > 20) return;

	for (uint8_t i=0; i<len; i++)
	{
		lcd_send_data (str[i]);
		HAL_Delay(1);
	}
}

