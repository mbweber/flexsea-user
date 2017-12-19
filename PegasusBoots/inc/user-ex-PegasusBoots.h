/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/PegasusBoots' MIT Biomechatronics Ankle Exoskeleton
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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-PegasusBoots: User code running on Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	*
****************************************************************************/

#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

#ifndef INC_EXO_H
#define INC_EXO_H

//****************************************************************************
// Include(s)
//****************************************************************************

	
//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern struct exodata_s exo;
    
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_pegasus(void);
void pegasus_fsm(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//#define scale 			1000 	//torque inverse scaling factor
#define WKL                 0
#define WKR                 1
#define BP1                 2
#define BP2                 3

//****************************************************************************
// Structure(s)
//****************************************************************************
	
#endif	//INC_EXO_H

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
