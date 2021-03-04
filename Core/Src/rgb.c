/*
 * rgb.c
 *
 *  Created on: 12 lis 2020
 *      Author: bortn
 */
#include <stdlib.h>
#include "rgb.h"
#include "tim.h"
_Bool  Switch1()
{
	if(HAL_GPIO_ReadPin(GPIOC, Sterowanie_1_Pin)==0) return false;
	else return true;
}
_Bool  Switch2()
{
	if(HAL_GPIO_ReadPin(GPIOC, Sterowanie_2_Pin)==0) return false;
	else return true;
}
_Bool  Switch3()
{
	if(HAL_GPIO_ReadPin(GPIOC, Sterowanie_1_Pin)==0) return false;
	else return true;

}

void SetColor (uint8_t R, uint8_t G, uint8_t B,uint8_t i)
{

switch(i)
		{
	case 1:
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, G); //GREEN
	break;
	case 2:
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, G); //GREEN
	break;
	case 3:
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, G); //GREEN
	break;
		}
}
void SetColorAll (uint8_t R, uint8_t G, uint8_t B)
{



	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, G); //GREEN


	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, G); //GREEN


	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, R); //RED
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, B); //BLUE
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, G); //GREEN


}

void Fade(uint8_t time)
{

	while(1)
	{
		int r= 255,g=0,b=0;
		for(int i=0; i<255; i++)
		{
			r--;
			g++;
			b=255-r;
			SetColor(r, g, b,1);
			HAL_Delay(time);
		}

		for(int i=0; i<255; i++)
		{
		g--;
		SetColor(r, g, b,1);
		HAL_Delay(time);
		}
		for(int i=0; i<255; i++)
				{
					g++;
					b--;
					SetColor(r, g, b,1);
					HAL_Delay(time);
				}
		for(int i=0; i<255; i++)
				{
					r++;
					g--;
					SetColor(r, g, b,1);
					HAL_Delay(time);
				}




	}
}

void Solar(uint8_t time)
{
	while(1)
	{
	int r= 255,g=0,b=0;
	SetColor(r, g, b,1);
	HAL_Delay(time);
			for(int i=0; i<255; i++)
			{
				r--;
				SetColor(r, g, b,1);
				HAL_Delay(1);
			}

			for(int i=0; i<255; i++)
			{
			r++;
			g++;
			b++;
			SetColor(r, g, b,1);
			HAL_Delay(1);
			}

	HAL_Delay(time);
		for(int i=0; i<255; i++)
			{
			g--;
			b--;
			r--;
			SetColor(r, g, b,1);
			HAL_Delay(1);
			}

		for(int i=0; i<255; i++)
			{
			r++;
			SetColor(r, g, b,1);
			HAL_Delay(1);
			}
	}
}
void FadeAll(uint8_t time)
{

	while(1)
	{
		int r= 255,g=0,b=0;
		for(int i=0; i<255; i++)
		{
			r--;
			g++;
			b=255-r;
			SetColorAll(r, g, b);
			HAL_Delay(time);
		}

		for(int i=0; i<255; i++)
		{
		g--;
		SetColorAll(r, g, b);
		HAL_Delay(time);
		}
		for(int i=0; i<255; i++)
				{
					g++;
					b--;
					SetColorAll(r, g, b);
					HAL_Delay(time);
				}
		for(int i=0; i<255; i++)
				{
					r++;
					g--;
					SetColorAll(r, g, b);
					HAL_Delay(time);
				}




	}
}
