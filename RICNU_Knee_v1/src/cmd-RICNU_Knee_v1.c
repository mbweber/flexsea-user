/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/RICNU_Knee_v1' RIC/NU Knee
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
	[This file] cmd-RICNU_Knee_v1: Custom commands for this project
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

//Command: CMD_RICNU. Type: R/W.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType,uint16_t *len,\
					uint8_t offset, uint8_t controller, \
					int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
					int16_t g2, int16_t g3)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = READ;

	//Data:
	shBuf[index++] = offset;
	shBuf[index++] = controller;
	SPLIT_32(setpoint, shBuf, &index);
	shBuf[index++] = setGains;
	SPLIT_16(g0, shBuf, &index);
	SPLIT_16(g1, shBuf, &index);
	SPLIT_16(g2, shBuf, &index);
	SPLIT_16(g3, shBuf, &index);

	//Payload length:
	(*len) = index;
}

//Command: CMD_RICNU. Type: R.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = READ;

	//Data:
	shBuf[index++] = offset;

	//Payload length:
	(*len) = index;
}

//Command: CMD_RICNU. Type: W.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		//Arguments:
		if(offset == 0)
		{
			SPLIT_16((uint16_t)imu.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)imu.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)imu.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.z, shBuf, &index);
			SPLIT_32((uint32_t)exec1.enc_motor, shBuf, &index);
			SPLIT_32((uint32_t)exec1.enc_joint, shBuf, &index);
			SPLIT_16((uint16_t)ctrl.current.actual_val, shBuf, &index);
		}
		else if(offset == 1)
		{
			//Compressed Strain:

			shBuf[index++] = strain1.compressedBytes[0];
			shBuf[index++] = strain1.compressedBytes[1];
			shBuf[index++] = strain1.compressedBytes[2];
			shBuf[index++] = strain1.compressedBytes[3];
			shBuf[index++] = strain1.compressedBytes[4];
			shBuf[index++] = strain1.compressedBytes[5];
			shBuf[index++] = strain1.compressedBytes[6];
			shBuf[index++] = strain1.compressedBytes[7];
			shBuf[index++] = strain1.compressedBytes[8];
			//Include other variables here (ToDo)
		}
		else
		{
			//Deal with other offsets here...
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	//Payload length:
	(*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_ricnu_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;

	//Temporary variables
	uint8_t offset = 0;
	uint8_t tmpController = 0, tmpSetGains = 0;
	int32_t tmpSetpoint = 0;
	int16_t tmpGain[4] = {0,0,0,0};

	//Decode data received:
	index = P_DATA1;
	offset = buf[index++];
	tmpController = buf[index++];
	tmpSetpoint = (int32_t)REBUILD_UINT32(buf, &index);
	tmpSetGains = buf[index++];
	tmpGain[0] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[1] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[2] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[3] = (int16_t)REBUILD_UINT16(buf, &index);

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

	//Act on the decoded data:
	rx_cmd_ricnu_Action1(tmpController, tmpSetpoint, tmpSetGains, tmpGain[0],
							tmpGain[1], tmpGain[2], tmpGain[3]);
	#endif

	//Generate the reply:
//	numb = tx_cmd_ricnu_w(TX_CMD_DEFAULT, offset, 0, 0, 0, 0, 0, 0);	//ToDo
//	COMM_GEN_STR_DEFAULT
//	flexsea_send_serial_master(myPort, myData, myLen);	//ToDo
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_ricnu_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;

	(void)info;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
		(void)buf;
		flexsea_error(SE_CMD_NOT_PROGRAMMED);
		return;
	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

		struct execute_s *ex = &exec1;
		struct ricnu_s *rn = &ricnu_1;

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			rn->ex.gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex.enc_motor = (int32_t) REBUILD_UINT32(buf, &index);
			rn->ex.enc_joint = (int32_t) REBUILD_UINT32(buf, &index);
			rn->ex.current = (int16_t) REBUILD_UINT16(buf, &index);
		}
		else if(offset == 1)
		{
			rn->st.compressedBytes[0] = buf[index++];
			rn->st.compressedBytes[1] = buf[index++];
			rn->st.compressedBytes[2] = buf[index++];
			rn->st.compressedBytes[3] = buf[index++];
			rn->st.compressedBytes[4] = buf[index++];
			rn->st.compressedBytes[5] = buf[index++];
			rn->st.compressedBytes[6] = buf[index++];
			rn->st.compressedBytes[7] = buf[index++];
			rn->st.compressedBytes[8] = buf[index++];

			//Include other variables here (ToDo)
		}
		else
		{
			//...
		}

		//Copy RICNU to Exec:
		*ex = rn->ex;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

//Gets called when our Master Writes to us
void rx_cmd_ricnu_w(uint8_t *buf, uint8_t *info)
{
	//Master Write isn't implemented for this command.

	(void)buf;
	(void)info;
	flexsea_error(SE_CMD_NOT_PROGRAMMED);
}

//Command = rx_cmd_ricnu, section = READ
void rx_cmd_ricnu_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3)
{
	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

	//Update controller (if needed):
	control_strategy(controller);

	//Only change the setpoint if we are in current control mode:
	if(ctrl.active_ctrl == CTRL_CURRENT)
	{
		ctrl.current.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.current.gain.g0 = g0;
			ctrl.current.gain.g1 = g1;
		}
	}
	else if(ctrl.active_ctrl == CTRL_OPEN)
	{
		motor_open_speed_1(setpoint);
	}
	else if(ctrl.active_ctrl == CTRL_POSITION)
	{
		ctrl.position.setp = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.position.gain.g0 = g0;
			ctrl.position.gain.g1 = g1;
		}
	}
	else if (ctrl.active_ctrl == CTRL_IMPEDANCE)
	{
		ctrl.impedance.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.impedance.gain.g0 = g0;
			ctrl.impedance.gain.g1 = g1;
			ctrl.current.gain.g0 = g2;
			ctrl.current.gain.g1 = g3;
		}
	}

	#else

		//Unused - deal with variables to avoid warnings
		(void) controller;
		(void)setpoint;
		(void)setGains;
		(void)g0;
		(void)g1;
		(void)g2;
		(void)g3;

	#endif
}

//==================
//Hum, do we still need this? Keeping it for now to avoid breaking stuff... (ToDo)

//TODO move to user files
//Transmission of a READ_ALL_RICNU command
uint32_t tx_cmd_data_read_all_ricnu(uint8_t receiver, uint8_t cmd_type, uint8_t *buf, uint32_t len)
{
	uint8_t tmp0 = 0, tmp1 = 0, tmp2 = 0, tmp3 = 0;
	uint32_t bytes = 0;

	//Fresh payload string:
	prepare_empty_payload(board_id, receiver, buf, len);

	//Command:
	buf[P_CMDS] = 1;                     //1 command in string

	if(cmd_type == CMD_READ)
	{
		buf[P_CMD1] = CMD_R(CMD_READ_ALL_RICNU);

		bytes = P_CMD1 + 1;     //Bytes is always last+1
	}
	else if(cmd_type == CMD_WRITE)
	{
		//In that case Write is only used for the Reply

		buf[P_CMD1] = CMD_W(CMD_READ_ALL_RICNU);

		#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		//Arguments:
		uint16_to_bytes((uint16_t)imu.gyro.x, &tmp0, &tmp1);
		buf[P_DATA1 + 0] = tmp0;
		buf[P_DATA1 + 1] = tmp1;
		uint16_to_bytes((uint16_t)imu.gyro.y, &tmp0, &tmp1);
		buf[P_DATA1 + 2] = tmp0;
		buf[P_DATA1 + 3] = tmp1;
		uint16_to_bytes((uint16_t)imu.gyro.z, &tmp0, &tmp1);
		buf[P_DATA1 + 4] = tmp0;
		buf[P_DATA1 + 5] = tmp1;

		uint16_to_bytes((uint16_t)imu.accel.x, &tmp0, &tmp1);
		buf[P_DATA1 + 6] = tmp0;
		buf[P_DATA1 + 7] = tmp1;
		uint16_to_bytes((uint16_t)imu.accel.y, &tmp0, &tmp1);
		buf[P_DATA1 + 8] = tmp0;
		buf[P_DATA1 + 9] = tmp1;
		uint16_to_bytes((uint16_t)imu.accel.z, &tmp0, &tmp1);
		buf[P_DATA1 + 10] = tmp0;
		buf[P_DATA1 + 11] = tmp1;

		//Motor encoder, multi-turns
		uint32_to_bytes((uint32_t)exec1.enc_motor, &tmp0, &tmp1, &tmp2, &tmp3);
		buf[P_DATA1 + 12] = tmp0;
		buf[P_DATA1 + 13] = tmp1;
		buf[P_DATA1 + 14] = tmp2;
		buf[P_DATA1 + 15] = tmp3;

		//Joint encoder, limited to 1 rotation
		uint32_to_bytes((uint32_t)exec1.enc_joint, &tmp0, &tmp1, &tmp2, &tmp3);
		buf[P_DATA1 + 16] = tmp0;
		buf[P_DATA1 + 17] = tmp1;
		buf[P_DATA1 + 18] = tmp2;
		buf[P_DATA1 + 19] = tmp3;

		uint16_to_bytes((uint16_t)ctrl.current.actual_val, &tmp0, &tmp1);
		buf[P_DATA1 + 20] = tmp0;
		buf[P_DATA1 + 21] = tmp1;

		//Compressed Strain:
		buf[P_DATA1 + 22] = strain1.compressedBytes[0];
		buf[P_DATA1 + 23] = strain1.compressedBytes[1];
		buf[P_DATA1 + 24] = strain1.compressedBytes[2];
		buf[P_DATA1 + 25] = strain1.compressedBytes[3];
		buf[P_DATA1 + 26] = strain1.compressedBytes[4];
		buf[P_DATA1 + 27] = strain1.compressedBytes[5];
		buf[P_DATA1 + 28] = strain1.compressedBytes[6];
		buf[P_DATA1 + 29] = strain1.compressedBytes[7];
		buf[P_DATA1 + 30] = strain1.compressedBytes[8];

		bytes = P_DATA1 + 31;     //Bytes is always last+1

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

//TODO move to user files
//Reception of a READ_ALL_RICNU command
void rx_cmd_data_read_all_ricnu(uint8_t *buf)
{
	uint8_t numb = 0, sampling = 0;

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

	//Structure pointer. Points to ricnu_1 by default.
	struct execute_s *exec_s_ptr = &exec1;
	struct ricnu_s *ricnu_s_ptr = &ricnu_1;
	//struct executeD_s *execD_s_ptr = &execD1;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

	if(IS_CMD_RW(buf[P_CMD1]) == READ)
	{
		//Received a Read command from our master, prepare a reply:

		#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		//Generate the reply:
		numb = tx_cmd_data_read_all_ricnu(buf[P_XID], CMD_WRITE, tmp_payload_xmit, PAYLOAD_BUF_LEN);
		numb = comm_gen_str(tmp_payload_xmit, comm_str_485_1, numb);
		numb = COMM_STR_BUF_LEN;	//Fixed length for now to accomodate the DMA

		//Delayed response:
		rs485_reply_ready(comm_str_485_1, (numb));

		#ifdef USE_USB
		usb_puts(comm_str_485_1, (numb));
		#endif

		#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

		#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//ToDo

		#endif	//BOARD_TYPE_FLEXSEA_EXECUTE
	}
	else if(IS_CMD_RW(buf[P_CMD1]) == WRITE)
	{
		//Two options: from Master of from slave (a read reply)

		//Decode data:
		//...

		if(sent_from_a_slave(buf))
		{
			//We received a reply to our read request

			#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			//Store values:

			ricnu_s_ptr->ex.gyro.x = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+0], buf[P_DATA1+1]));
			ricnu_s_ptr->ex.gyro.y = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+2], buf[P_DATA1+3]));
			ricnu_s_ptr->ex.gyro.z = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+4], buf[P_DATA1+5]));

			ricnu_s_ptr->ex.accel.x = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+6], buf[P_DATA1+7]));
			ricnu_s_ptr->ex.accel.y = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+8], buf[P_DATA1+9]));
			ricnu_s_ptr->ex.accel.z = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+10], buf[P_DATA1+11]));

			ricnu_s_ptr->ex.enc_motor = (int32_t) (BYTES_TO_UINT32(buf[P_DATA1+12], buf[P_DATA1+13], \
																		buf[P_DATA1+14], buf[P_DATA1+15]));
			ricnu_s_ptr->ex.enc_joint = (int32_t) (BYTES_TO_UINT32(buf[P_DATA1+16], buf[P_DATA1+17], \
																		buf[P_DATA1+18], buf[P_DATA1+19]));

			ricnu_s_ptr->ex.current = (int16_t) (BYTES_TO_UINT16(buf[P_DATA1+20], buf[P_DATA1+21]));

			ricnu_s_ptr->st.compressedBytes[0] = buf[P_DATA1 + 22];
			ricnu_s_ptr->st.compressedBytes[1] = buf[P_DATA1 + 23];
			ricnu_s_ptr->st.compressedBytes[2] = buf[P_DATA1 + 24];
			ricnu_s_ptr->st.compressedBytes[3] = buf[P_DATA1 + 25];
			ricnu_s_ptr->st.compressedBytes[4] = buf[P_DATA1 + 26];
			ricnu_s_ptr->st.compressedBytes[5] = buf[P_DATA1 + 27];
			ricnu_s_ptr->st.compressedBytes[6] = buf[P_DATA1 + 28];
			ricnu_s_ptr->st.compressedBytes[7] = buf[P_DATA1 + 29];
			ricnu_s_ptr->st.compressedBytes[8] = buf[P_DATA1 + 30];

			#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			//Plan uses the Super Structure to save decoded values:
			#if(defined BOARD_TYPE_FLEXSEA_PLAN)
			*exec_s_ptr = ricnu_s_ptr->ex;
			#endif //#(defined BOARD_TYPE_FLEXSEA_PLAN)
		}
		else
		{
			//Master is writing a value to this board

			#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

			//Nothing to do for now

			#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

			#ifdef BOARD_TYPE_FLEXSEA_MANAGE

			//Nothing to do for now

			#endif	//BOARD_TYPE_FLEXSEA_EXECUTE
		}
	}
}

//===

#ifdef __cplusplus
}
#endif
