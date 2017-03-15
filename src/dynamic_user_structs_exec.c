#include "dynamic_user_structs.h"

/* This works for Plan - Execute only
    Requirements:

    - send labels to GUI
    - send data to GUI
    - keep msg under max size

*/

struct DynamicUserData_s
{
    int a;
    int b;
};

const uint8_t DYNAMIC_USER_NUM_FIELDS = 2;
const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {4, 4};
static uint8_t fieldSizes[DYNAMIC_USER_NUM_FIELDS] = {0};

// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b"}; 

static uint8_t numOffsetsToSend;
static uint8_t offsetsToSend[DYNAMIC_USER_NUM_FIELDS];

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
    char* label;
    uint16_t labelLength;
    int j;
    for(i = 0; i < DYNAMIC_USER_NUM_FIELDS; i++)
    {
        label = fieldLabels[i];
        labelLength = strlen(label);
        //label length
        shBuf[index++] = labelLength;
        //label
        for(j = 0; j < labelLength; j++)
        {
            shBuf[index++] = label[i];
        }
    }

    *len = index;
}

void tx_cmd_user_dyn_sendData(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, uint8_t numOffsets, uint8_t* offsets)
{
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_WRITE;
    uint16_t index = 0;

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
    uint16_t index = P_DATA;
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
        tx_cmd_user_dyn_sendData(TX_N_DEFAULT), numOffsets, offsets;
        
    }

    packAndSend(P_AND_SEND_DEFAULT);
}

void rx_cmd_user_dyn_w(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA;
    (void)info;

    // we need to make sure that in the case they send more offsets than we can allow, 
    // we don't overrun our array. Also we can't accept invalid offsets
    
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
}

// void rx_cmd_user_dyn_rw(uint8_t *buf, uint8_t *info)
// {
//     uint16_t index = P_DATA;

//     uint8_t* readInto = (uint8_t*)(&dynamicUserData);
//     uint8_t length = sizeof(dynamicUserData);
//     int i;
//     for(i=0;i<length;i++)
//         readInto[i] = buf[index++];

//     tx_cmd_user_dyn_sendData();
// }

void tx_cmd_user_dyn_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
                        uint8_t sendMetaData, uint8_t numOffsets, uint8_t* offsets)
{
    numOffsets = numOffsets > 0 ? numOffsets : 0;
    
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_READ;

    uint16_t index = 0;
    shBuf[index++] = sendMetaData;
    shBuf[index++] = numOffsets;
    int i;
    for(i = 0; i < numOffsets; i++)
    {
        shBuf[index++] = offsets[i];
    }

    *len = index;
}

// void tx_cmd_user_dyn_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
//                         uint8_t )
// {
//     *cmd = CMD_USER_DYNAMIC;
//     *cmdType = CMD_WRITE;

//     uint16_t index = 0;
//     shBuf[index++] = sendMetaData;
//     *len = index;
// }

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA;

    uint8_t sentMetaData = (buf[index++] == SEND_METADATA);
    if(sentMetaData)
    {

    }

}
