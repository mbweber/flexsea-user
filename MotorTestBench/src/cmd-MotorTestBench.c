/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/MIT_2DoF_Ankle' MIT Biomechatronics 2-dof Ankle
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
	[This file] cmd-MotorTestBench: Custom commands for this project
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-12-06 | jfduval | Initial release
	*
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include "../inc/flexsea_system.h"
#include "../../inc/flexsea_cmd_user.h"

//Execute boards only:
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
#include "main.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Variable(s)
//****************************************************************************
extern uint8_t motortb_startCycleFlag;

//****************************************************************************
// Function(s)
//****************************************************************************

void tx_cmd_motortb_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave)
{
	//Variable(s) & command:
	uint16_t index = 0;
	int16_t *motortbPtr;
	(*cmd) = CMD_MOTORTB;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = slave;

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Structure pointer. Points to exec1 by default.
		struct execute_s *exec_s_ptr = &exec1;

		//Assign data structure based on slave:
		if(slave == 0 || slave == 1)
		{
			//Offsets 0 & 1 are for Execute:

			if(slave == 0)
			{
				motortbPtr = motortb.ex1;
				exec_s_ptr = &exec1;
			}
			else
			{
				motortbPtr = motortb.ex2;
				exec_s_ptr = &exec2;
			}

			SPLIT_16((uint16_t)motortbPtr[0], shBuf, &index);
			SPLIT_16((uint16_t)motortbPtr[1], shBuf, &index);
			SPLIT_16((uint16_t)motortbPtr[2], shBuf, &index);
			SPLIT_16((uint16_t)motortbPtr[3], shBuf, &index);
			SPLIT_16((uint16_t)motortbPtr[4], shBuf, &index);
			SPLIT_16((uint16_t)motortbPtr[5], shBuf, &index);

			SPLIT_16(exec_s_ptr->strain, shBuf, &index);
			SPLIT_16(exec_s_ptr->analog[0], shBuf, &index);
			SPLIT_16(exec_s_ptr->analog[1], shBuf, &index);

			SPLIT_32((uint32_t)exec_s_ptr->enc_display, shBuf, &index);
			SPLIT_16((uint16_t)exec_s_ptr->current, shBuf, &index);

			shBuf[index++] = exec_s_ptr->volt_batt;
			shBuf[index++] = exec_s_ptr->volt_int;
			shBuf[index++] = exec_s_ptr->temp;
			shBuf[index++] = exec_s_ptr->status1;
			shBuf[index++] = exec_s_ptr->status2;
		}
		else if(slave == 2)
		{
			//Offset 2: Battery board and Manage status

			//Battery:
			shBuf[index++] = batt1.rawBytes[0];
			shBuf[index++] = batt1.rawBytes[1];
			shBuf[index++] = batt1.rawBytes[2];
			shBuf[index++] = batt1.rawBytes[3];
			shBuf[index++] = batt1.rawBytes[4];
			shBuf[index++] = batt1.rawBytes[5];
			shBuf[index++] = batt1.rawBytes[6];
			shBuf[index++] = batt1.rawBytes[7];

			//Manage:
			SPLIT_16((uint16_t)motortb.mn1[0], shBuf, &index);
			SPLIT_16((uint16_t)motortb.mn1[1], shBuf, &index);
			SPLIT_16((uint16_t)motortb.mn1[2], shBuf, &index);
			SPLIT_16((uint16_t)motortb.mn1[3], shBuf, &index);
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		SPLIT_16((uint16_t)motortb.ex1[0], shBuf, &index);
		SPLIT_16((uint16_t)motortb.ex1[1], shBuf, &index);
		SPLIT_16((uint16_t)motortb.ex1[2], shBuf, &index);
		SPLIT_16((uint16_t)motortb.ex1[3], shBuf, &index);
		SPLIT_16((uint16_t)motortb.ex1[4], shBuf, &index);
		SPLIT_16((uint16_t)motortb.ex1[5], shBuf, &index);

		SPLIT_16(strain_read(), shBuf, &index);
		SPLIT_16(read_analog(0), shBuf, &index);
		SPLIT_16(read_analog(1), shBuf, &index);

		SPLIT_32((uint32_t)refresh_enc_display(), shBuf, &index);
		SPLIT_16((uint16_t)ctrl.current.actual_val, shBuf, &index);

		shBuf[index++] = safety_cop.v_vb;
		shBuf[index++] = safety_cop.v_vg;
		shBuf[index++] = safety_cop.temperature;
		shBuf[index++] = safety_cop.status1;
		shBuf[index++] = safety_cop.status2;

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	//Payload length:
	(*len) = index;
}

void tx_cmd_motortb_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t offset, \
							uint16_t startCycle)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_MOTORTB;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = offset;
	if(offset == 0 || offset == 1)
	{
		shBuf[index++] = startCycle;
	}

	//Payload length:
	(*len) = index;
}

void rx_cmd_motortb_rw(uint8_t *buf, uint8_t *info)
{
	uint8_t offset = 0;
	offset = buf[P_DATA1];

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		int16_t tmp_wanted_current = 0, tmp_open_spd = 0, tmp_start_gait_cycle = 0;
		uint16_t index = P_DATA1;
		offset = buf[index++];

		if(offset == 0 || offset == 1)
		{
			//Update controller:
			// controller info should be here, but we can ignore it since we are running sort of autonomously
			// control_strategy(buf[index++]);
			// instead we just increment index
			index++;
			tmp_start_gait_cycle = (int16_t) REBUILD_UINT16(buf, &index);
			motortb_startCycleFlag = (tmp_start_gait_cycle > 0);
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//using user variables for now

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Reply:
	tx_cmd_motortb_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_motortb_rr(uint8_t *buf, uint8_t *info)
{
	(void)info;	//Unused for now

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

		uint8_t offset = 0, slave = 0;
		uint16_t index = 0;
		struct execute_s *exec_s_ptr = &exec1;
		int16_t *motortbPtr;

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE))

		slave = buf[P_XID];
		//Assign data structure based on slave:
		if(slave == FLEXSEA_EXECUTE_1)
		{
			offset = 0;
			motortbPtr = motortb.ex1;
			exec_s_ptr = &exec1;
		}
		else
		{
			offset = 1;
			motortbPtr = motortb.ex2;
			exec_s_ptr = &exec2;
		}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_PLAN))

			offset = buf[P_DATA1];
			//Assign data structure based on slave:
			if(offset == 0)
			{
				motortbPtr = motortb.ex1;
				exec_s_ptr = &exec1;
			}
			else if(offset == 1)
			{
				motortbPtr = motortb.ex2;
				exec_s_ptr = &exec2;
			}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			index = P_DATA1+1;
			if(offset == 0 || offset == 1)
			{
				motortbPtr[0] = (int16_t) REBUILD_UINT16(buf, &index);
				motortbPtr[1] = (int16_t) REBUILD_UINT16(buf, &index);
				motortbPtr[2] = (int16_t) REBUILD_UINT16(buf, &index);
				motortbPtr[3] = (int16_t) REBUILD_UINT16(buf, &index);
				motortbPtr[4] = (int16_t) REBUILD_UINT16(buf, &index);
				motortbPtr[5] = (int16_t) REBUILD_UINT16(buf, &index);

				exec_s_ptr->strain = (int16_t) REBUILD_UINT16(buf, &index);
				exec_s_ptr->analog[0] = (int16_t) REBUILD_UINT16(buf, &index);
				exec_s_ptr->analog[1] = (int16_t) REBUILD_UINT16(buf, &index);
				exec_s_ptr->enc_display = (int32_t) REBUILD_UINT32(buf, &index);
				exec_s_ptr->current = (int16_t) REBUILD_UINT16(buf, &index);

				exec_s_ptr->volt_batt = buf[index++];
				exec_s_ptr->volt_int = buf[index++];
				exec_s_ptr->temp = buf[index++];
				exec_s_ptr->status1 = buf[index++];
				exec_s_ptr->status2 = buf[index++];
			}
			else if(offset == 2)
			{
				batt1.rawBytes[0] = buf[index++];
				batt1.rawBytes[1] = buf[index++];
				batt1.rawBytes[2] = buf[index++];
				batt1.rawBytes[3] = buf[index++];
				batt1.rawBytes[4] = buf[index++];
				batt1.rawBytes[5] = buf[index++];
				batt1.rawBytes[6] = buf[index++];
				batt1.rawBytes[7] = buf[index++];

				//Manage:
				motortb.mn1[0] = (int16_t) REBUILD_UINT16(buf, &index);
				motortb.mn1[1] = (int16_t) REBUILD_UINT16(buf, &index);
				motortb.mn1[2] = (int16_t) REBUILD_UINT16(buf, &index);
				motortb.mn1[3] = (int16_t) REBUILD_UINT16(buf, &index);
			}

		#endif	//BOARD_TYPE_FLEXSEA_PLAN

	#else

		(void)buf;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

#ifdef __cplusplus
}
#endif
