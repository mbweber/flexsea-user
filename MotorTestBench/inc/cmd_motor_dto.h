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
    
typedef struct motor_dto_s {

    uint8_t controller;
    int16_t ctrl_i;
    int16_t ctrl_o;
    uint8_t startGaitCycle;
    
} motor_dto;

#endif
