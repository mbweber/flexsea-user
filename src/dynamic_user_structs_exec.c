#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

#include "../inc/dynamic_user_structs.h"
#include <string.h>
#include "flexsea_dataformats.h"
#include "../inc/flexsea_cmd_user.h"
#include <flexsea_system.h>

/* This works for Plan - Execute only
    Requirements:

    - send labels to GUI
    - send data to GUI
    - keep msg under max size

	- TODO: allow GUI to tell execute which fields it wants set
	- (one time message in tx_..._w, which would tell us which offsets to send)
*/

struct DynamicUserData_s
{
    int a;
    int b;
	int c;
};

DynamicUserData_t dynamicUserData;

#define DYNAMIC_USER_NUM_FIELDS  2
const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {FORMAT_32S, FORMAT_32S};
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b"};
// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.

/*

#define DYNAMIC_USER_NUM_FIELDS  3
const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {FORMAT_8U, FORMAT_32S, FORMAT_32S};
// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b", "c"};

*/
//static uint8_t fieldSizes[DYNAMIC_USER_NUM_FIELDS] = {0};
//static uint8_t numOffsetsToSend;
//static uint8_t offsetsToSend[DYNAMIC_USER_NUM_FIELDS];

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

    uint8_t* writeOut = (uint8_t*)(&dynamicUserData);
    uint8_t length = sizeof(dynamicUserData);
    int i;
    for(i=0;i<length;i++)
        shBuf[index++] = writeOut[i];

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
        uint8_t numOffsets = buf[index++];
        int i;
        uint8_t offsets[DYNAMIC_USER_NUM_FIELDS];

        for(i = 0; i < numOffsets; i++)
        {
            offsets[i] = buf[index++];
        }
        tx_cmd_user_dyn_sendData(TX_N_DEFAULT);

    }

    packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_user_dyn_w(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA1;
    (void)info;
	(void) buf;

    // we need to make sure that in the case they send more offsets than we can allow,
    // we don't overrun our array. Also we can't accept invalid offsets
    /*
    uint8_t n = buf[index++];
    numOffsetsToSend = n < DYNAMIC_USER_NUM_FIELDS ? n : DYNAMIC_USER_NUM_FIELDS;

    int i;
    n = 0;
    for(i = 0; i < numOffsets && n < DYNAMIC_USER_NUM_FIELDS; i++)
    {
        uint8_t offset = buf[index++];
        if(offset < DYNAMIC_USER_NUM_FIELDS)
        {
            offsetToSend[i] = offset;
            n++;
        }
    }
	*/
}

void init_flexsea_payload_ptr_dynamic()
{
    flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_READ] = &rx_cmd_user_dyn_r;
    flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_WRITE] = &rx_cmd_user_dyn_w;
}

#endif	//BOARD_TYPE_FLEXSEA_EXECUTE
