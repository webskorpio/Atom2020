#include "seg.h"
//------------------------------------------------
uint16_t digits[18] = {
	0xFC, // 0
	0x60, // 1
	0xDA, // 2
	0xF2, // 3
	0x66, // 4
	0xB6, // 5
	0xBE, // 6
	0xE0, // 7
	0xFE, // 8
	0xF6, // 9
	0x1E, // t
	0x9E, // E
	0xB6, // S
	0x01  // .
};

uint16_t seg[]= {
	0x80,	//1
	0x40,	//2
	0x20,	//3
	0x10	//4
};
uint8_t d=0;


void Shift(uint16_t tim, uint8_t d){

	uint8_t r[4] = {tim/1000%10,tim/100%10,tim/10%10,0};
		for(uint8_t i=0; i<8; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			if(digits[r[d]] & (1 << i )){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); 
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
		}		
		for(uint8_t i=0; i<8; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			if(seg[d] & (1 << i )){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); 
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
		}			
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	d++;
	if(d>=4){d=0;}
	
	}

	void ShiftTest(){
	uint8_t r[4]={10,11,12,10};
		for(uint8_t i=0; i<8; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			if(digits[r[d]] & (1 << i )){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); 
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
		}		
		for(uint8_t i=0; i<8; i++){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			if(seg[d] & (1 << i )){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); 
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  
		}			
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	d++;
	if(d>=4){d=0;}
	}	
	