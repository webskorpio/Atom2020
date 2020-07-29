/*
 * QuardDisplay.c
 * Набор функций и макроопределений для управления 4-х
 * разрядным индикатором Multi-functional Shield.
 * Аппаратная платформа: STM32F030
 * 4-х разрядный индикатор Multi-functional Shield.
 * Тактовая частота МК STM32F030 16МГц.
 */  
 
 //Для работы в теле цикла инициализируем таймер 3 - HAL_TIM_Base_Start_IT(&htim3);

#include "main.h"
#include "stm32f0xx_hal.h"
#include "QuardDisplay.h"
#include "stm32f0xx.h"
//#include "stm32f0xx_it.h"

//extern TIM_HandleTypeDef htim6;

//Массив содержит коды чисел 0...9 для отображения на 4-х разрядном индикаторе.
uint8_t numerals[] = {QD_0, QD_1, QD_2, QD_3, QD_4, QD_5, QD_6, QD_7, QD_8, QD_9};
	
uint8_t count = 0;
char dig1, dig2, dig3, dig4;



/* 
 * Функция выполняет индикацию 4-х цифр или символов.
 * Передаваемое значение: коды для 4-х цифр или символов.
 * Возвращаемое значение: нет.
 * Коды цифр и символов для инидикации представлены в файле QuardDisplay.h.
 */
void DisplayCodeData(char d1,char d2,char d3,char d4)
{	
	dig1 = d1;
	dig2 = d2;
	dig3 = d3;
	dig4 = d4;
}

/* 
 * Функция выполняет индикацию 1 цифры или символа.
 * Передаваемое значение: код цифры или символа для индикации.
 * Возвращаемое значение: нет.
 * Коды цифр и символов для инидикации представлены в файле QuardDisplay.h.
*/
void sendByte(char txbyte)
{
	uint8_t mask;
	for (mask=0x80; mask; mask = mask >> 1)
	{
		if (txbyte & mask)
		{
			HAL_GPIO_WritePin(D_595_GPIO_Port, D_595_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(D_595_GPIO_Port, D_595_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(CLK_595_GPIO_Port, CLK_595_Pin, GPIO_PIN_SET);
		//HAL_Delay(1);
		HAL_GPIO_WritePin(CLK_595_GPIO_Port, CLK_595_Pin, GPIO_PIN_RESET);
	}
}

/* 
 * Функция выполняет индикацию 4-х цифр.
 * Передаваемое значение: 4 цифры, диапазон значений каждой цифры 0...9.
 * Возвращаемое значение: нет.
 * Функция выполняет перекодировку цифр в коды для отображения на 
 * семисегментном инидикаторе и вывод на 4-х разрядный дисплей.
*/
void DisplayByteDigit(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];
	char digit3 = numerals[d3];
	char digit4 = numerals[d4];
	DisplayCodeData(digit1, digit2, digit3, digit4);
}

/* 
 * Функция выполняет индикацию 4-х значного числа.
 * Передаваемое значение: число.
 * Возвращаемое значение: нет.
 * Функция выполняет перекодировку числа в коды для отображения на 
 * семисегментном инидикаторе и вывод на 4-х разрядный дисплей.
*/
void DisplayNumber(uint16_t val)
{
	uint8_t d1 = val/1000;
	uint8_t d2 = (val - d1 * 1000) / 100;
	uint8_t d3 = (val - d1 * 1000 - d2 * 100)/10;
	uint8_t d4 = val - d1 * 1000 - d2 * 100 - d3 * 10;
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];
	char digit3 = numerals[d3];
	char digit4 = numerals[d4];
	DisplayCodeData(digit1, digit2, digit3, digit4);
}

/* 
 * Функция выполняет индикацию 4-х значного числа.
 * Передаваемое значение: число.
 * Возвращаемое значение: нет.
 * Функция выполняет перекодировку числа в коды для отображения на 
 * семисегментном инидикаторе и вывод на 4-х разрядный дисплей, разделённых символом "."..
*/
void DisplaySNumber(uint16_t val)
{
	uint8_t d1 = val/1000;
	uint8_t d2 = (val - d1 * 1000) / 100;
	uint8_t d3 = (val - d1 * 1000 - d2 * 100)/10;
	uint8_t d4 = val - d1 * 1000 - d2 * 100 - d3 * 10;
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];//&QD_DOT;
	char digit3 = numerals[d3];
	char digit4 = numerals[d4];
	DisplayCodeData(digit1, digit2, digit3, digit4);
}



/* 
 * Функция выполняет индикацию времени на 4-х разрядном индикаторе.
 * Передаваемое значение: часы и минуты.
 * Возвращаемое значение: нет.
 * Функция выполняет перекодировку цифр в коды для отображения на 
 * семисегментном инидикаторе и вывод на 4-х разрядный дисплей
 * текущих значений времени, разделённых символом ".".
*/
void DisplayTime(uint8_t hour, uint8_t minute)
{
	uint8_t d1 = hour/10;
	uint8_t d2 = hour - d1*10;
	uint8_t d3 = minute/10;
	uint8_t d4 = minute - d1*10;
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];//&QD_DOT;
	char digit3 = numerals[d3];
	char digit4 = numerals[d4];
	DisplayCodeData(digit1, digit2, digit3, digit4);
}

/* 
 * Функция выполняет индикацию температуры на 4-х разрядном индикаторе.
 * Функция обеспечивает работу динамической индикации
 * Передаваемое значение: значение температуры от 0 до 99.
 * Возвращаемое значение: нет.
 * Функция выполняет перекодировку значения температуры в коды для отображения на
 * семисегментном инидикаторе и вывод на 4-х разрядный дисплей
 * текущих значений температуры с добавлением *С.
*/
void DisplayTemp(uint8_t temp)
{
	uint8_t d1 = temp/10;
	uint8_t d2 = temp - d1*10;
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];
	char digit3 = 0x9C;
	char digit4 = 0xC6;
	DisplayCodeData(digit1, digit2, digit3, digit4);
}

/* 
 * Функция выполняет индикацию градусов на 4-х разрядном индикаторе.
 * Функция обеспечивает работу динамической индикации
 * Передаваемое значение: 16-битное значение градусов.
 * Возвращаемое значение: нет.
*/
void DisplayDegree(uint16_t degree)
{
	uint8_t d1 = degree / 100;
	uint8_t d2 = (degree - d1 * 100)/10;
	uint8_t d3 = degree - d1 * 100 - d2 * 10;
	char digit1 = numerals[d1];
	char digit2 = numerals[d2];
	char digit3 = numerals[d3];
	char digit4 = 0x9C;
	DisplayCodeData(digit1, digit2, digit3, digit4);
}

/* 
 * Функция обрабатывает прерывание по совпадению от таймера 0.
 * Функция обеспечивает работу динамической индикации
 * Передаваемое значение: нет.
 * Возвращаемое значение: нет.
*/

//void TIM6_IRQHandler(void)
void tx_display(void)
{
// HAL_TIM_IRQHandler(&htim6);
//	
	if(count <= 4) count++;
	else count = 1;
	
	switch(count)
	{

		case 1:
		{
			sendByte(dig1);
			sendByte(0x01);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_RESET);
		}
		break;
		
		case 2:
		{
			sendByte(dig2);
			sendByte(0x02);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_RESET);
		}
		break;
		
		case 3:
		{
			
			sendByte(dig3);
			sendByte(0x04);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_RESET);
		}
		break;
		
		case 4:
		{
			sendByte(dig4);
			sendByte(0x08);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_SET);
			//HAL_Delay(1);
			HAL_GPIO_WritePin(L_595_GPIO_Port, L_595_Pin, GPIO_PIN_RESET);
		}
		break;
	}
	
	
}
