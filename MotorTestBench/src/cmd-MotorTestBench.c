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
#include "flexsea_pid_controller.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Variable(s)
//****************************************************************************
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

	#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
	extern pid_controller torqueController;
	#elif(ACTIVE_SUBPROJECT == SUBPROJECT_B)
	extern pid_controller positionController;
	#endif

#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Function(s)
//****************************************************************************

void unpackResponse(void* result, uint8_t* buf, uint16_t* index, uint16_t lengthInBytes)
{
	uint8_t* output = (uint8_t*) result;
	int i;
	for(i = 0; i < lengthInBytes; i++)
	{
		output[i] = buf[(*index) + i];
	}
	(*index) += lengthInBytes;
}

void fillBufferData(uint8_t *shBuf, uint16_t *index, void* data, uint16_t lengthInBytes)
{
    uint8_t* unsignedDataPointer = (uint8_t*)(data);
    int i;
    for(i = 0; i < lengthInBytes; i++)
    {
        shBuf[(*index)+i] = unsignedDataPointer[i];
    }
    (*index)+=lengthInBytes;
}

//#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

void handleMotorTbReply(motor_dto_reply response, struct execute_s* exec_ptr)
{
    if(exec_ptr)
    {
        exec_ptr->strain = response.strain;
        exec_ptr->analog[0] = response.analog0;
        exec_ptr->analog[1] = response.analog1;
        exec_ptr->enc_display = response.encoder;
        exec_ptr->current = response.current;

        exec_ptr->volt_batt = response.v_vb;
        exec_ptr->volt_int = response.v_vg;
        exec_ptr->temp = response.temperature;
        exec_ptr->status1 = response.status1;
        exec_ptr->status2 = response.status2;
    }
}

//#endif

#ifdef BOARD_TYPE_FLEXSEA_MANAGE
inline motor_dto_reply generateMotorDtoReply(struct execute_s* execPtr)
{
    motor_dto_reply result;

    result.encoder = execPtr->enc_display;
    result.strain = execPtr->strain;
    result.analog0 = execPtr->analog[0];
    result.analog1 = execPtr->analog[1];

    result.v_vb = execPtr->volt_batt;
    result.v_vg = execPtr->volt_int;

    result.temperature = execPtr->temp;
    result.status1 = execPtr->status1;
    result.status2 = execPtr->status2;

    return result;
}
#endif

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
inline motor_dto_reply generateMotorDtoReply()
{
	motor_dto_reply result;

	result.encoder = exec1.enc_control_ang;
	result.strain = strain_read();
	result.analog0 = read_analog(0);
	result.analog1 = read_analog(1);
	result.current = ctrl.current.actual_val;
	result.v_vb = safety_cop.v_vb;
	result.v_vg = safety_cop.v_vg;
	result.temperature = safety_cop.temperature;
	result.status1 = safety_cop.status1;
	result.status2 = safety_cop.status2;

	return result;
}
#endif

void tx_cmd_motortb_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave)
{
	//Variable(s) & command:
	uint16_t index = 0;
	int32_t *execDataPtr;
	(*cmd) = CMD_MOTORTB;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = slave;

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE
        int i;

		//Assign data structure based on slave:
		if(slave == 0 || slave == 1)
		{
			//Offsets 0 & 1 are for Execute:
            struct execute_s *exec_s_ptr = NULL;

			if(slave == 0)
			{
				execDataPtr = motortb.ex1;
				exec_s_ptr = &exec1;
			}
			else
			{
				execDataPtr = motortb.ex2;
				exec_s_ptr = &exec2;
			}
            motor_dto_reply dto = generateMotorDtoReply(exec_s_ptr);
            uint8_t dtoLengthInBytes = sizeof(motor_dto_reply);

            fillBufferData(shBuf, &index, &dto, dtoLengthInBytes);

            for(i=0; i<4; i++)
                SPLIT_32((uint32_t)execDataPtr[i], shBuf, &index);

		}
		else if(slave == 2) //Offset 2: Battery board and Manage status
		{
			//Battery:
			for(i=0; i<8; i++)
				shBuf[index++] = batt1.rawBytes[i];

			//Manage:
			for(i=0; i<4; i++)
				SPLIT_16((uint16_t)motortb.mn1[i], shBuf, &index);
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		volatile int temp = 0;
		if(motortb_flagsOut & 0x02)
		{
			temp = motortb_flagsOut*20;
		}
		motor_dto_reply response = generateMotorDtoReply();
		uint16_t lengthInBytes = sizeof(motor_dto_reply);
		fillBufferData(shBuf, &index, &response, lengthInBytes);

        #if(ACTIVE_SUBPROJECT == SUBPROJECT_A)
            SPLIT_32((uint32_t)torqueController.setpoint, shBuf, &index);
            SPLIT_32((uint32_t)torqueController.controlValue, shBuf, &index);
        #elif(ACTIVE_SUBPROJECT == SUBPROJECT_B)
            SPLIT_32((uint32_t)positionController.setpoint, shBuf, &index);
            SPLIT_32((uint32_t)positionController.controlValue, shBuf, &index);
        #endif

        shBuf[index++] = motortb_flagsOut;
        motortb_flagsOut = 0;

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	//Payload length:
	(*len) = index;
}

void tx_cmd_motortb_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t offset, \
							motor_dto* dto)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_MOTORTB;
	(*cmdType) = CMD_READ;

	shBuf[index++] = offset;

	uint16_t lengthInBytes = sizeof(motor_dto);
    fillBufferData(shBuf, &index, dto, lengthInBytes);

	//Payload length:
	(*len) = index;
}

void rx_cmd_motortb_rw(uint8_t *buf, uint8_t *info)
{
	uint8_t offset = 0;
	offset = buf[P_DATA1];

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		uint16_t index = P_DATA1;
		offset = buf[index++];

		motor_dto dto;
		uint16_t lengthInBytes = sizeof(dto);

		if(offset == 0 || offset == 1)
		{
			unpackResponse(&dto, buf, &index, lengthInBytes);
		}
		
		volatile int x = 0;
		if(dto.gaitCycleFlag)
			x = 2;
		if(dto.gaitCycleFlag == 0x02)
			x--;
		
		motortb_flagsIn = dto.gaitCycleFlag;
		
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
        int32_t *execDataPtr;

        #if((defined BOARD_TYPE_FLEXSEA_MANAGE))
        execControllerState_t* ctrlStatePtr = &exec1ControllerState;

        slave = buf[P_XID];
        //Assign data structure based on slave:
        if(slave == FLEXSEA_EXECUTE_1)
        {
            offset = 0;
            execDataPtr = motortb.ex1;
            exec_s_ptr = &exec1;
            ctrlStatePtr = &exec1ControllerState;
        }
        else
        {
            offset = 1;
            execDataPtr = motortb.ex2;
            exec_s_ptr = &exec2;
            ctrlStatePtr = &exec2ControllerState;
        }

        #endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

        #if((defined BOARD_TYPE_FLEXSEA_PLAN))

            offset = buf[P_DATA1];
            //Assign data structure based on slave:
            if(offset == 0)
            {
                execDataPtr = motortb.ex1;
                exec_s_ptr = &exec1;
            }
            else if(offset == 1)
            {
                execDataPtr = motortb.ex2;
                exec_s_ptr = &exec2;
            }

        #endif	//((defined BOARD_TYPE_FLEXSEA_PLAN))

        index = P_DATA1+1;
        motor_dto_reply response;
        uint8_t testFlag = -1;
        if(offset == 0 || offset == 1)
        {
            uint16_t lengthInBytes = sizeof(motor_dto_reply);
            unpackResponse(&response, buf, &index, lengthInBytes);
            handleMotorTbReply(response, exec_s_ptr);

            #if(defined BOARD_TYPE_FLEXSEA_PLAN)
                int i;
                for(i=0;i<4;i++)
                    execDataPtr[i] = REBUILD_UINT32(buf, &index);
            #elif(defined BOARD_TYPE_FLEXSEA_MANAGE)
                    ctrlStatePtr->setpoint = REBUILD_UINT32(buf, &index);
                    ctrlStatePtr->actual = REBUILD_UINT32(buf, &index);
            #endif
        }
        #ifdef BOARD_TYPE_FLEXSEA_MANAGE
        if(offset == 0)
        {
            exec1CtrlStateReady = 1;
            if(testFlag)
                exec1TestState = NONE;
        }
        else if(offset == 1)
        {
            exec2CtrlStateReady = 1;
            if(testFlag)
                exec2TestState = NONE;
        }
        #endif
        else if(offset == 2)
        {
            int i;
            for(i=0; i<8; i++)
                batt1.rawBytes[i] = buf[index++];

            for(i=0; i<4; i++)
                motortb.mn1[0] = (int16_t) REBUILD_UINT16(buf, &index);
        }
    #else

        (void)buf;

    #endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

#ifdef __cplusplus
}
#endif
