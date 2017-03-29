#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

#include <string.h>
#include <flexsea.h>
#include <flexsea_system.h>
#include "flexsea_dataformats.h"
#include "../inc/flexsea_cmd_user.h"
#include "../inc/dynamic_user_structs.h"

/* This works for Plan - Execute only
    Requirements:

    - send labels to GUI
    - send data to GUI
    - keep msg under max size

	- TODO: allow GUI to tell execute which fields it wants set
	- (one time message in tx_..._w, which would tell us which offsets to send)
*/

volatile struct DynamicUserData_s
{
    int a;
    int b;
	int c;
	uint16_t d;
};

DynamicUserData_t dynamicUserData;
/*
#define DYNAMIC_USER_NUM_FIELDS  2
const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {FORMAT_32S, FORMAT_32S};
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b"};
// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.
*/


#define DYNAMIC_USER_NUM_FIELDS  4
const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {FORMAT_32S, FORMAT_32S, FORMAT_32S, FORMAT_16U};
// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b", "c", "d"};



static uint8_t fieldFlags[DYNAMIC_USER_NUM_FIELDS] = {0};

/*
	Don't touch the code below
*/

void tx_cmd_user_dyn_sendMetaData(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_WRITE;

    uint16_t index = 0;

    shBuf[index++] = SEND_METADATA;
    //Number of fields
    shBuf[index++] = DYNAMIC_USER_NUM_FIELDS;

    //type of each field
    int i;
    for(i = 0; i < DYNAMIC_USER_NUM_FIELDS; i++)
    {
        shBuf[index++] = fieldTypes[i];
    }

    //label of each field
    uint16_t labelLength;
    int j;
    for(i = 0; i < DYNAMIC_USER_NUM_FIELDS; i++)
    {
        labelLength = strlen(fieldLabels[i]);
        //label length
        shBuf[index++] = labelLength;
        //label
        for(j = 0; j < labelLength; j++)
        {
            shBuf[index++] = fieldLabels[i][j];
        }
    }

    *len = index;
}

void tx_cmd_user_dyn_sendFieldFlags(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_WRITE;    

    uint16_t index = 0;

    shBuf[index++] = SEND_FIELD_FLAGS;
    //Number of fields
	index = index + packFieldFlags(shBuf + index, DYNAMIC_USER_NUM_FIELDS, fieldFlags);
	
    *len = index;
}

float quickSin(int y)
{
	y %= 360;
	float x = y > 180 ? -360+y : y;

	x = x * 314 / 18000;

	float x3 = x*x*x;
	float x5 = x3*x*x;
	float x7 = x5*x*x;
	float x9 = x7*x*x;

	return x - x3/6 + x5/120 - x7/5040 + x9/362880;
}

void tx_cmd_user_dyn_sendData(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_WRITE;
    uint16_t index = 0;

	static int x = 0;
	x++;
	dynamicUserData.a = 100 * quickSin(x);
	dynamicUserData.b = 100 * quickSin(x/2);

    shBuf[index++] = SEND_DATA;
    //we save a byte to write total number of dynamicUserData bytes written
    uint16_t indexOfTotalBytesWritten = index++;

    uint8_t* writeOut = (uint8_t*)(&dynamicUserData);
    uint8_t length = sizeof(dynamicUserData);


    int i, j, fieldLength, fieldOffset = 0, totalBytes = 0;
    for(i = 0; i < DYNAMIC_USER_NUM_FIELDS; i++)
    {
    	fieldLength = 1;
    	if(fieldTypes[i] < 8 && FORMAT_SIZE_MAP[fieldTypes[i]] > 0)
    		fieldLength = FORMAT_SIZE_MAP[fieldTypes[i]];

    	if(fieldFlags[i])
    	{
    		for(j = 0; j < fieldLength && fieldOffset+j < length; j++)
    		{
    			shBuf[index++] = writeOut[fieldOffset + j];
    			totalBytes++;
    		}
    	}

    	fieldOffset += fieldLength;
    }

    shBuf[indexOfTotalBytesWritten] = totalBytes;

    *len = index;
}

void rx_cmd_user_dyn_r(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA1;
    uint8_t shouldSendMetaData = buf[index++];
    (void)buf;

    if(shouldSendMetaData == SEND_METADATA)
        tx_cmd_user_dyn_sendMetaData(TX_N_DEFAULT);
    else
    {
        tx_cmd_user_dyn_sendData(TX_N_DEFAULT);

    }

    packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_user_dyn_w(uint8_t *buf, uint8_t *info)
{
    (void)info;
    if(!unpackFieldFlags(buf + P_DATA1, fieldFlags, DYNAMIC_USER_NUM_FIELDS))
	{
		//respond
		tx_cmd_user_dyn_sendFieldFlags(TX_N_DEFAULT);
		packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
	}
}

void init_flexsea_payload_ptr_dynamic()
{
    flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_READ] = &rx_cmd_user_dyn_r;
    flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_WRITE] = &rx_cmd_user_dyn_w;
}

#endif	//BOARD_TYPE_FLEXSEA_EXECUTE
