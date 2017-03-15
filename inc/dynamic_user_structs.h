/****************************************************************************
    [Project] FlexSEA: Flexible & Scalable Electronics Architecture
    [Sub-project] 'flexsea-system' System commands & functions
    Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
    [Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
    [Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
    Biomechatronics research group <http://biomech.media.mit.edu/>
    [Contributors]
*****************************************************************************
    [This file] flexsea_global_structs: contains all the data structures
    used across the project
*****************************************************************************
    [Change log] (Convention: YYYY-MM-DD | author | comment)
    * 2017-03-14 | David Weisdorf | Initial release
    *
****************************************************************************/

#ifndef INC_FLEXSEA_DYNAMIC_USER_STRUCTS_H
#define INC_FLEXSEA_DYNAMIC_USER_STRUCTS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdint.h>
#include <flexsea_board.h>

//The following are for the user to define in dynamic_user_structs_common.c
struct DynamicUserData_s;

typedef struct DynamicUserData_s DynamicUserData_t;
// you may also wish to typedef this type to a more convenient name
// typedef struct DynamicUserData_s YourNameHere_t

void tx_cmd_user_dyn_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
                        uint8_t sendMetaData);

void tx_cmd_user_dyn_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
                        uint8_t numOffsets, uint8_t* offsets);

void rx_cmd_user_dyn_r(uint8_t *buf, uint8_t *info);
void rx_cmd_user_dyn_w(uint8_t *buf, uint8_t *info);

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info);

void init_flexsea_payload_ptr_dynamic();

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
extern DynamicUserData_t dynamicUserData;
#endif
	
#ifdef BOARD_TYPE_FLEXSEA_PLAN

//void initFlexseaDynamicUserStructs();
extern uint8_t newMetaDataAvailable;
extern uint8_t newDataAvailable;

extern uint8_t* dynamicUser_data;
extern uint8_t dynamicUser_numFields;
extern uint8_t* dynamicUser_fieldTypes;
extern uint8_t* dynamicUser_fieldLengths;
extern uint8_t* dynamicUser_labelLengths;
extern char** dynamicUser_labels;

#endif

// Flags
#define SEND_DATA       0x00
#define SEND_METADATA   0x01


#ifdef __cplusplus
    }
#endif

#endif //INC_FLEXSEA_DYNAMIC_USER_STRUCTS_H
