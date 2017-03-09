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
    int16_t ctrl_i __attribute__((packed));
    int16_t ctrl_o __attribute__((packed));
    uint8_t controller;
    uint8_t gaitCycleFlag;
} __attribute__((packed)) motor_dto;

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
	
	int32_t encoder __attribute__((packed));

	int16_t strain __attribute__((packed));
	int16_t analog0 __attribute__((packed));

	int16_t analog1 __attribute__((packed));
	int16_t current __attribute__((packed));
	
	int16_t v_vb __attribute__((packed));
	int16_t v_vg __attribute__((packed));
	
	int8_t temperature;
	int8_t status1;
    int8_t status2;
	
	//packed attribute enforces that no padding bytes are used.
	//
} __attribute__((packed)) motor_dto_reply;

#endif
