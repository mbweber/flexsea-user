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
	[This file] cmd-MIT_2DoF_Ankle: Custom commands for this project
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-27 | jfduval | Initial release
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

//Will change this, but for now the payloads will be stored in: (ToDo eliminate soon)
uint8_t tmp_payload_xmit[PAYLOAD_BUF_LEN];

//****************************************************************************
// Function(s)
//****************************************************************************

void tx_cmd_ankle2dof_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_A2DOF;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = slave;

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Structure pointer. Points to exec1 by default.
		struct execute_s *exec_s_ptr = &exec1;

		//Assign data structure based on slave:
		if(slave == 0)
		{
			exec_s_ptr = &exec1;
		}
		else
		{
			exec_s_ptr = &exec2;
		}

		SPLIT_16((uint16_t)exec_s_ptr->gyro.x, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->gyro.y, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->gyro.z, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.x, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.y, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.z, shBuf, &index);

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

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		SPLIT_16((uint16_t)imu.gyro.x, shBuf, &index);
		SPLIT_16((uint16_t)imu.gyro.y, shBuf, &index);
		SPLIT_16((uint16_t)imu.gyro.z, shBuf, &index);
		SPLIT_16((uint16_t)imu.accel.x, shBuf, &index);
		SPLIT_16((uint16_t)imu.accel.y, shBuf, &index);
		SPLIT_16((uint16_t)imu.accel.z, shBuf, &index);

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

void tx_cmd_ankle2dof_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave, uint8_t controller, \
							int16_t ctrl_i, int16_t ctrl_o)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_A2DOF;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = slave;
	shBuf[index++] = controller;
	SPLIT_16((uint16_t)ctrl_i, shBuf, &index);
	SPLIT_16((uint16_t)ctrl_o, shBuf, &index);

	//Payload length:
	(*len) = index;
}

void rx_cmd_ankle2dof_rw(uint8_t *buf, uint8_t *info)
{
	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		int16_t tmp_wanted_current = 0, tmp_open_spd = 0;
		uint8_t slave = 0;
		uint16_t index = P_DATA1;
		slave = buf[index++];

		//Update controller:
		control_strategy(buf[index++]);

		//Only change the setpoint if we are in current control mode:
		if(ctrl.active_ctrl == CTRL_CURRENT)
		{
			index = P_DATA1+2;
			tmp_wanted_current = (int16_t) REBUILD_UINT16(buf, &index);
			ctrl.current.setpoint_val = tmp_wanted_current;
		}
		else if(ctrl.active_ctrl == CTRL_OPEN)
		{
			index = P_DATA1+4;
			tmp_open_spd = (int16_t) REBUILD_UINT16(buf, &index);;
			motor_open_speed_1(tmp_open_spd);
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		uint8_t slave = 0;
		slave = buf[P_DATA1];

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Reply:
	tx_cmd_ankle2dof_w(TX_N_DEFAULT, buf[P_DATA1]);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_ankle2dof_rr(uint8_t *buf, uint8_t *info)
{
	(void)info;	//Unused for now

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

		uint8_t slave = 0;
		uint16_t index = 0;
		struct execute_s *exec_s_ptr = &exec1;

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE))

			slave = buf[P_XID];
			//Assign data structure based on slave:
			if(slave == FLEXSEA_EXECUTE_1)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_PLAN))

			slave = buf[P_DATA1];
			//Assign data structure based on slave:
			if(slave == 0)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			index = P_DATA1+1;
			exec_s_ptr->gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.z = (int16_t) REBUILD_UINT16(buf, &index);

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

		#endif	//BOARD_TYPE_FLEXSEA_PLAN

	#else

		(void)buf;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

//****************************************************************************
// Antiquated function(s) - will be deleted soon:
//****************************************************************************

/*
//Transmission of a CTRL_SPECIAL_5 command: Ankle 2DOF
uint32_t tx_cmd_ctrl_special_5(uint8_t receiver, uint8_t cmd_type, uint8_t *buf, uint32_t len, \
								uint8_t slave, uint8_t controller, int16_t ctrl_i, int16_t ctrl_o)
{
	uint8_t tmp0 = 0, tmp1 = 0, tmp2 = 0, tmp3 = 0;
	uint32_t bytes = 0;

	#if(defined BOARD_TYPE_FLEXSEA_MANAGE)

	//Structure pointer. Points to exec1 by default.
	struct execute_s *exec_s_ptr = &exec1;

	#endif	//(defined BOARD_TYPE_FLEXSEA_MANAGE)

	//Fresh payload string:
	prepare_empty_payload(board_id, receiver, buf, len);

	//Command:
	buf[P_CMDS] = 1;                     //1 command in string

	if(cmd_type == CMD_READ)
	{
		buf[P_CMD1] = CMD_R(CMD_SPC5);

		//Arguments:
		buf[P_DATA1] = slave;
		buf[P_DATA1 + 1] = controller;
		uint16_to_bytes((uint16_t)ctrl_i, &tmp0, &tmp1);
		buf[P_DATA1 + 2] = tmp0;
		buf[P_DATA1 + 3] = tmp1;
		uint16_to_bytes((uint16_t)ctrl_o, &tmp0, &tmp1);
		buf[P_DATA1 + 4] = tmp0;
		buf[P_DATA1 + 5] = tmp1;

		bytes = P_DATA1 + 6;		//Bytes is always last+1
	}
	else if(cmd_type == CMD_WRITE)
	{
		//In that case Write is only used for the Reply

		buf[P_CMD1] = CMD_W(CMD_SPC5);
		buf[P_DATA1] = slave;

		#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Assign data structure based on slave:
		if(slave == 0)
		{
			exec_s_ptr = &exec1;
		}
		else
		{
			exec_s_ptr = &exec2;
		}

		//Arguments
		uint16_to_bytes((uint16_t)exec_s_ptr->gyro.x, &tmp0, &tmp1);
		buf[P_DATA1 + 1] = tmp0;
		buf[P_DATA1 + 2] = tmp1;
		uint16_to_bytes((uint16_t)exec_s_ptr->gyro.y, &tmp0, &tmp1);
		buf[P_DATA1 + 3] = tmp0;
		buf[P_DATA1 + 4] = tmp1;
		uint16_to_bytes((uint16_t)exec_s_ptr->gyro.z, &tmp0, &tmp1);
		buf[P_DATA1 + 5] = tmp0;
		buf[P_DATA1 + 6] = tmp1;

		uint16_to_bytes((uint16_t)exec_s_ptr->accel.x, &tmp0, &tmp1);
		buf[P_DATA1 + 7] = tmp0;
		buf[P_DATA1 + 8] = tmp1;
		uint16_to_bytes((uint16_t)exec_s_ptr->accel.y, &tmp0, &tmp1);
		buf[P_DATA1 + 9] = tmp0;
		buf[P_DATA1 + 10] = tmp1;
		uint16_to_bytes((uint16_t)exec_s_ptr->accel.z, &tmp0, &tmp1);
		buf[P_DATA1 + 11] = tmp0;
		buf[P_DATA1 + 12] = tmp1;

		uint16_to_bytes(exec_s_ptr->strain, &tmp0, &tmp1);
		buf[P_DATA1 + 13] = tmp0;
		buf[P_DATA1 + 14] = tmp1;

		uint16_to_bytes(exec_s_ptr->analog[0], &tmp0, &tmp1);
		buf[P_DATA1 + 15] = tmp0;
		buf[P_DATA1 + 16] = tmp1;

		uint16_to_bytes(exec_s_ptr->analog[1], &tmp0, &tmp1);
		buf[P_DATA1 + 17] = tmp0;
		buf[P_DATA1 + 18] = tmp1;

		uint32_to_bytes((uint32_t)exec_s_ptr->enc_display, &tmp0, &tmp1, &tmp2, &tmp3);
		buf[P_DATA1 + 19] = tmp0;
		buf[P_DATA1 + 20] = tmp1;
		buf[P_DATA1 + 21] = tmp2;
		buf[P_DATA1 + 22] = tmp3;

		uint16_to_bytes((uint16_t)exec_s_ptr->current, &tmp0, &tmp1);
		buf[P_DATA1 + 23] = tmp0;
		buf[P_DATA1 + 24] = tmp1;

		buf[P_DATA1 + 25] = exec_s_ptr->volt_batt;
		buf[P_DATA1 + 26] = exec_s_ptr->volt_int;
		buf[P_DATA1 + 27] = exec_s_ptr->temp;
		buf[P_DATA1 + 28] = exec_s_ptr->status1;
		buf[P_DATA1 + 29] = exec_s_ptr->status2;

		bytes = P_DATA1 + 30;     //Bytes is always last+1
		#endif	//BOARD_TYPE_FLEXSEA_MANAGE

		#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		//Arguments:
		uint16_to_bytes((uint16_t)imu.gyro.x, &tmp0, &tmp1);
		buf[P_DATA1 + 1] = tmp0;
		buf[P_DATA1 + 2] = tmp1;
		uint16_to_bytes((uint16_t)imu.gyro.y, &tmp0, &tmp1);
		buf[P_DATA1 + 3] = tmp0;
		buf[P_DATA1 + 4] = tmp1;
		uint16_to_bytes((uint16_t)imu.gyro.z, &tmp0, &tmp1);
		buf[P_DATA1 + 5] = tmp0;
		buf[P_DATA1 + 6] = tmp1;

		uint16_to_bytes((uint16_t)imu.accel.x, &tmp0, &tmp1);
		buf[P_DATA1 + 7] = tmp0;
		buf[P_DATA1 + 8] = tmp1;
		uint16_to_bytes((uint16_t)imu.accel.y, &tmp0, &tmp1);
		buf[P_DATA1 + 9] = tmp0;
		buf[P_DATA1 + 10] = tmp1;
		uint16_to_bytes((uint16_t)imu.accel.z, &tmp0, &tmp1);
		buf[P_DATA1 + 11] = tmp0;
		buf[P_DATA1 + 12] = tmp1;

		uint16_to_bytes(strain_read(), &tmp0, &tmp1);
		buf[P_DATA1 + 13] = tmp0;
		buf[P_DATA1 + 14] = tmp1;

		uint16_to_bytes(read_analog(0), &tmp0, &tmp1);
		buf[P_DATA1 + 15] = tmp0;
		buf[P_DATA1 + 16] = tmp1;

		uint16_to_bytes(read_analog(1), &tmp0, &tmp1);
		buf[P_DATA1 + 17] = tmp0;
		buf[P_DATA1 + 18] = tmp1;

		uint32_to_bytes((uint32_t)refresh_enc_display(), &tmp0, &tmp1, &tmp2, &tmp3);
		buf[P_DATA1 + 19] = tmp0;
		buf[P_DATA1 + 20] = tmp1;
		buf[P_DATA1 + 21] = tmp2;
		buf[P_DATA1 + 22] = tmp3;

		uint16_to_bytes((uint16_t)ctrl.current.actual_val, &tmp0, &tmp1);
		buf[P_DATA1 + 23] = tmp0;
		buf[P_DATA1 + 24] = tmp1;

		buf[P_DATA1 + 25] = safety_cop.v_vb;
		buf[P_DATA1 + 26] = safety_cop.v_vg;
		buf[P_DATA1 + 27] = safety_cop.temperature;
		buf[P_DATA1 + 28] = safety_cop.status1;
		buf[P_DATA1 + 29] = safety_cop.status2;

		bytes = P_DATA1 + 30;     //Bytes is always last+1

		#endif	//BOARD_TYPE_FLEXSEA_EXECUTE
	}
	else
	{
		//Invalid
		flexsea_error(SE_INVALID_READ_TYPE);
		bytes = 0;
	}

	return bytes;
}

//Reception of a CMD_SPECIAL_5 command
void rx_cmd_special_5(uint8_t *buf)
{
	uint32_t numb = 0;
	uint8_t slave = 0;
	int16_t tmp_wanted_current = 0, tmp_open_spd = 0;

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

	//Structure pointer.
	struct execute_s *exec_s_ptr = &exec1;
	//struct spc5_s *spc5_s_ptr;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

	if(IS_CMD_RW(buf[P_CMD1]) == READ)
	{
		//Received a Read command from our master.

		#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		slave = buf[P_DATA1];

		//Update controller:
		control_strategy(buf[P_DATA1 + 1]);

		//Only change the setpoint if we are in current control mode:
		if(ctrl.active_ctrl == CTRL_CURRENT)
		{
			tmp_wanted_current = BYTES_TO_UINT16(buf[P_DATA1 + 2], buf[P_DATA1 + 3]);
			ctrl.current.setpoint_val = tmp_wanted_current;
		}
		else if(ctrl.active_ctrl == CTRL_OPEN)
		{
			tmp_open_spd = (int16_t) BYTES_TO_UINT16(buf[P_DATA1 + 4], buf[P_DATA1 + 5]);
			motor_open_speed_1(tmp_open_spd);
		}

		//Generate the reply:
		numb = tx_cmd_ctrl_special_5(buf[P_XID], CMD_WRITE, tmp_payload_xmit, PAYLOAD_BUF_LEN, slave, 0, 0, 0);
		numb = comm_gen_str(tmp_payload_xmit, comm_str_485_1, numb);
		numb = COMM_STR_BUF_LEN;	//Fixed length for now to accomodate the DMA

		//Delayed response:
		rs485_reply_ready(comm_str_485_1, (numb));

		#ifdef USE_USB
		usb_puts(comm_str_485_1, (numb));
		#endif

		#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

		#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Decode its data:
		//===============
		slave = buf[P_DATA1];
		//...

		//Generate the reply:
		//===================

		numb = tx_cmd_ctrl_special_5(buf[P_XID], CMD_WRITE, tmp_payload_xmit, PAYLOAD_BUF_LEN, \
									slave, 0, 0, 0);
		numb = comm_gen_str(tmp_payload_xmit, comm_str_spi, numb);
		numb = COMM_STR_BUF_LEN;	//Fixed length for now to accomodate the DMA
		flexsea_send_serial_master(PORT_USB, comm_str_spi, numb);	//Same comment here - ToDo fix
		//(the SPI driver will grab comm_str_spi directly)

		#endif	//BOARD_TYPE_FLEXSEA_MANAGE

		#ifdef BOARD_TYPE_FLEXSEA_PLAN
		//No code (yet), you shouldn't be here...
		flexsea_error(SE_CMD_NOT_PROGRAMMED);
		#endif	//BOARD_TYPE_FLEXSEA_PLAN
	}
	else if(IS_CMD_RW(buf[P_CMD1]) == WRITE)
	{
		//Two options: from Master of from slave (a read reply)

		if(sent_from_a_slave(buf))
		{
			//We received a reply to our read request

			#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
			//No code (yet), you shouldn't be here...
			flexsea_error(SE_CMD_NOT_PROGRAMMED);
			#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

			#if((defined BOARD_TYPE_FLEXSEA_MANAGE))
			slave = buf[P_XID];
			//Assign data structure based on slave:
			if(slave == FLEXSEA_EXECUTE_1)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}
			#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

			#if((defined BOARD_TYPE_FLEXSEA_PLAN))
			slave = buf[P_DATA1];

			//Assign data structure based on slave:
			if(slave == 0)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}
			#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

			#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			//Decode its data:
			//===============

			//Store values:

			exec_s_ptr->gyro.x = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+1], buf[P_DATA1+2]));
			exec_s_ptr->gyro.y = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+3], buf[P_DATA1+4]));
			exec_s_ptr->gyro.z = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+5], buf[P_DATA1+6]));

			exec_s_ptr->accel.x = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+7], buf[P_DATA1+8]));
			exec_s_ptr->accel.y = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+9], buf[P_DATA1+10]));
			exec_s_ptr->accel.z = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+11], buf[P_DATA1+12]));

			exec_s_ptr->strain = (BYTES_TO_UINT16(buf[P_DATA1+13], buf[P_DATA1+14]));
			exec_s_ptr->analog[0] = (BYTES_TO_UINT16(buf[P_DATA1+15], buf[P_DATA1+16]));
			exec_s_ptr->analog[1] = (BYTES_TO_UINT16(buf[P_DATA1+17], buf[P_DATA1+18]));

			exec_s_ptr->enc_display = (int32_t) (BYTES_TO_UINT32(buf[P_DATA1+19], buf[P_DATA1+20], \
										buf[P_DATA1+21], buf[P_DATA1+22]));

			exec_s_ptr->current = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+23], buf[P_DATA1+24]));

			exec_s_ptr->volt_batt = buf[P_DATA1+25];
			exec_s_ptr->volt_int = buf[P_DATA1+26];
			exec_s_ptr->temp = buf[P_DATA1+27];
			exec_s_ptr->status1 = buf[P_DATA1+28];
			exec_s_ptr->status2 = buf[P_DATA1+29];

			#endif	//BOARD_TYPE_FLEXSEA_MANAGE
		}
		else
		{
			//Master is writing a value to this board

			#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

			//ToDo call relevant functions ****

			#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

			#ifdef BOARD_TYPE_FLEXSEA_MANAGE
			//No code (yet), you shouldn't be here...
			flexsea_error(SE_CMD_NOT_PROGRAMMED);
			#endif	//BOARD_TYPE_FLEXSEA_MANAGE

			#ifdef BOARD_TYPE_FLEXSEA_PLAN
			//No code (yet), you shouldn't be here...
			flexsea_error(SE_CMD_NOT_PROGRAMMED);
			#endif	//BOARD_TYPE_FLEXSEA_PLAN
		}
	}
}
*/

#ifdef __cplusplus
}
#endif
