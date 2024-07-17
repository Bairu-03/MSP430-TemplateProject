/*
 * Encoder.h
 *
 *  Created on: 2024Äê7ÔÂ15ÈÕ
 *      Author: Bairu
 */

#ifndef HARDWARE_ENCODER_ENCODER_H_
#define HARDWARE_ENCODER_ENCODER_H_

#define P20 0
#define P22 1

extern uint16_t P20_Count;
extern uint16_t P22_Count;

void Encoder_Init(void);
float getRotatingSpeed(uint8_t PortX);

#endif /* HARDWARE_ENCODER_ENCODER_H_ */
