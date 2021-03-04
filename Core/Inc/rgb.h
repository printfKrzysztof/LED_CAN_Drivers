/*
 * rgb.h
 *
 *  Created on: 12 lis 2020
 *      Author: bortn
 */

#ifndef INC_RGB_H_
#define INC_RGB_H_
#include <stdint.h>
#include <stdbool.h>
//Ustawia kolor
void SetColor (uint8_t R, uint8_t G, uint8_t B, uint8_t i);
void SetColorAll (uint8_t R, uint8_t G, uint8_t B);
//Efekt teczy
void Fade(uint8_t time);
void FadeAll(uint8_t time);
//nasze barwy
void Solar(uint8_t time);
//bool ChechBreak;
_Bool Switch1();
_Bool Switch2();
_Bool Switch3();

#endif /* INC_RGB_H_ */
