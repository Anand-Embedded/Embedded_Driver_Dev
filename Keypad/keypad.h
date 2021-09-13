/*
 * keypad.h
 *
 *  Created on: Sep 3, 2021
 *      Author: ashantharam
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "main.h"

#define  KEYPAD_ROW_PIN_1		GPIO_PIN_0
#define  KEYPAD_ROW_PIN_2		GPIO_PIN_1
#define  KEYPAD_ROW_PIN_3		GPIO_PIN_2
#define  KEYPAD_ROW_PIN_4		GPIO_PIN_3

#define  KEYPAD_COLUMN_PIN_1	GPIO_PIN_8
#define  KEYPAD_COLUMN_PIN_2	GPIO_PIN_9
#define  KEYPAD_COLUMN_PIN_3	GPIO_PIN_10
#define  KEYPAD_COLUMN_PIN_4	GPIO_PIN_11

#define KEYPAD_COLUMN_SHIFT(x) 	((1) << ((x)+(8)))

#define KEYPAD_ROW_INDEX_0	0
#define KEYPAD_ROW_INDEX_1	1
#define KEYPAD_ROW_INDEX_2	2
#define KEYPAD_ROW_INDEX_3	3

#define KEYPAD_COULMN_INDEX_0	0
#define KEYPAD_COULMN_INDEX_1	1
#define KEYPAD_COULMN_INDEX_2	2
#define KEYPAD_COULMN_INDEX_3	3

#define KEYPAD_PORT			GPIOD

#define DEBOUNCE_DELAY_MS	200UL


void Keypad_Pins_init(void);
void Keypad_Key_detection(uint16_t const Row_Pin_number, uint8_t const Row_number);
void Keypad_Run(void);


static inline void keypad_rows_High(void)
{
	//Make all the rows to High state
	HAL_GPIO_WritePin(KEYPAD_PORT, (KEYPAD_ROW_PIN_1 | KEYPAD_ROW_PIN_2 | KEYPAD_ROW_PIN_3 | KEYPAD_ROW_PIN_4), GPIO_PIN_SET);
}


static inline void keypad_row_low(uint16_t row_number)
{
	HAL_GPIO_WritePin(KEYPAD_PORT, row_number, GPIO_PIN_RESET);
}

static inline GPIO_PinState keypad_read_column(uint16_t column_number)
{
	return HAL_GPIO_ReadPin(KEYPAD_PORT, column_number);
}


#endif /* INC_KEYPAD_H_ */
