/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#ifndef CMD_MOTOR_DTO_H
#define CMD_MOTOR_DTO_H

#include <stdint.h>
#include "../../flexsea-user/inc/flexsea_user_structs.h"

typedef struct motor_dto_s {

    uint8_t controller;
    int16_t ctrl_i;
    int16_t ctrl_o;
    uint8_t gaitCycleFlag;

} motor_dto;


/* IMPORTANT NOTE FOR USING THIS DTO
	never use the following pattern of:
	a) getting a pointer to a member of the struct
	b) dereferencing that pointer directly
	due to byte/word misalignment bad things might happen

	ie, DONT
	int16_t* member = &(motor_dto_reply.temperature);
	fprintf('%d', (*member));

	INSTEAD access variables using the . OR -> operators
	ie, DO
	fprintf('%d', motor_dto_reply.temperature);
*/
typedef struct motor_dto_reply_s {
	
	int16_t strain;
	int16_t analog0;
	int16_t analog1;
	int32_t encoder;
	int16_t current;
	
	int16_t v_vb;
	int16_t v_vg;
	
	int8_t temperature;
	int8_t status1;
	int8_t status2;
	/*
    uint8_t gaitCycleFlag;
	int16_t motorTemp;
	int32_t ctrlSetpoint;
	int32_t ctrlActual;
    */
	
	//packed attribute enforces that no padding bytes are used.
	//
} motor_dto_reply;

#endif
