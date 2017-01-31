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
	* 2016-12-08 | jfduval | Initial release
	*
****************************************************************************/

#ifndef INC_FLEXSEA_USER_STRUCT_H
#define INC_FLEXSEA_USER_STRUCT_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>

//****************************************************************************
// Prototype(s):
//****************************************************************************

//****************************************************************************
// Definition(s):
//****************************************************************************

#define GAIT_FLAG 0x01
#define CURRENT_FLAG 0x02
#define CURRENT_UNDER_TEST_FLAG 0x04
	
//****************************************************************************
// Structure(s):
//****************************************************************************
	
struct motortb_s
{
	int16_t mnRunning;
	int16_t mnTestState;
    int16_t mn1[4];
	int32_t ex1[6];
	int32_t ex2[6];
};

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern struct motortb_s motortb;

typedef struct execControllerState
{
    int32_t setpoint;
    int32_t actual;
} execControllerState_t;

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

extern uint8_t motortb_flagsIn;
extern uint8_t motortb_flagsOut;

#endif //BOARD_TYPE_FLEXSEA_EXECUTE

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

extern execControllerState_t exec1ControllerState;
extern execControllerState_t exec2ControllerState;

enum EXEC_TEST_STATE { NONE, GAIT, CURRENT};
extern enum EXEC_TEST_STATE exec1TestState;
extern enum EXEC_TEST_STATE exec2TestState;

extern uint8_t exec1CtrlStateReady;
extern uint8_t exec2CtrlStateReady;

#endif //flexsea manage



#endif	//INC_FLEXSEA_GLOBAL_STRUCT_H
