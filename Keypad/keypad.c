/*
 * keypad.c
 *
 *  Created on: Sep 3, 2021
 *      Author: ashantharam
 */
#include "keypad.h"


void Keypad_Pins_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin : Row Pins  */
	GPIO_InitStruct.Pin = KEYPAD_ROW_PIN_1 | KEYPAD_ROW_PIN_2 | KEYPAD_ROW_PIN_3 | KEYPAD_ROW_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KEYPAD_PORT, &GPIO_InitStruct);

	/*Configure GPIO pin : Column Pins  */
	GPIO_InitStruct.Pin = KEYPAD_COLUMN_PIN_1 | KEYPAD_COLUMN_PIN_2 | KEYPAD_COLUMN_PIN_3 | KEYPAD_COLUMN_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP; // To avoid floating state at input
	HAL_GPIO_Init(KEYPAD_PORT, &GPIO_InitStruct);
}

void Keypad_Run(void)
{
	Keypad_Key_detection(KEYPAD_ROW_PIN_1,KEYPAD_ROW_INDEX_0);
	Keypad_Key_detection(KEYPAD_ROW_PIN_2,KEYPAD_ROW_INDEX_1);
	Keypad_Key_detection(KEYPAD_ROW_PIN_3,KEYPAD_ROW_INDEX_2);
	Keypad_Key_detection(KEYPAD_ROW_PIN_4,KEYPAD_ROW_INDEX_3);
}


void Keypad_Key_detection(uint16_t const Row_Pin_number, uint8_t const Row_number)
{
	uint8_t row_index = 0 ;
	uint8_t column_index = 0;
	char keypad[4][4] =
	{	{'1' , '2', '3', 'A'},
		{'4' , '5', '6', 'B'},
		{'7' , '8', '9', 'C'},
		{'*' , '0', '#', 'D'}
	};

	// Make the ROW low
	keypad_row_low(Row_Pin_number);

	//get the row index number
	row_index = Row_number;

#if 1
	for (uint32_t i = 0; i < 4; i++, column_index++)
	{
		if(!keypad_read_column(KEYPAD_COLUMN_SHIFT(i)))
		{
			//delay is introduced to solve the de-bouncing effect
			HAL_Delay(DEBOUNCE_DELAY_MS);
			printf("%c\n",keypad[row_index][column_index]);
		}
	}

#else
	if(!(keypad_read_column(KEYPAD_COLUMN_PIN_1)))
	{
		//delay is introduced to solve the de-bouncing effect
		HAL_Delay(200);
		printf("%c\n",keypad[row_index][KEYPAD_COULMN_INDEX_0]);
	}

	if(!(keypad_read_column(KEYPAD_COLUMN_PIN_2)))
	{
		//delay is introduced to solve the de-bouncing effect
		HAL_Delay(200);
		printf("%c\n",keypad[row_index][KEYPAD_COULMN_INDEX_1]);
	}

	if(!(keypad_read_column(KEYPAD_COLUMN_PIN_3)))
	{
		//delay is introduced to solve the de-bouncing effect
		HAL_Delay(200);
		printf("%c\n",keypad[row_index][KEYPAD_COULMN_INDEX_2]);
	}

	if(!(keypad_read_column(KEYPAD_COLUMN_PIN_4)))
	{
		//delay is introduced to solve the de-bouncing effect
		HAL_Delay(200);
		printf("%c\n",keypad[row_index][KEYPAD_COULMN_INDEX_3]);
	}
#endif
	// Make all the keypad rows to high
	keypad_rows_High();

}


