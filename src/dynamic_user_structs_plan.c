#include "dynamic_user_structs.h"
#include "stdlib.h"


/* This works for Plan - Execute only
    Requirements:

    - send labels to GUI
    - send data to GUI
    - keep msg under max size

*/

void* getMemory(void* ptr, int size)
{
    if(ptr)
        ptr = realloc(ptr, size);
    else
        ptr = malloc(size);
    return ptr;
}

uint8_t sizeOfFieldType(uint8_t format)
{
    return 4;
}

void rx_metaData(uint8_t *buf, uint16_t index)
{
    dynamicUser_numFields = buf[index++];

    dynamicUser_fieldTypes =    (uint8_t*) getMemory(dynamicUser_fieldTypes, dynamicUser_numFields);
    dynamicUser_fieldLengths =  (uint8_t*) getMemory(dynamicUser_fieldLengths, dynamicUser_numFields);
    dynamicUser_labelLengths =  (uint8_t*) getMemory(dynamicUser_labelLengths, dynamicUser_numFields);

    // need to check if dynamicUser_fieldLabels is already an array of allocated memory pointers.
    // otherwise we might get a memory leak
    dynamicUser_fieldLabels =   (char**) getMemory(dynamicUser_fieldLabels, dynamicUser_numFields * sizeof(char*) );

    int i; 
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        uint8_t fieldType = buf[index++];

        dynamicUser_fieldTypes[i] = fieldType;
        dynamicUser_fieldLengths[i] = sizeOfFieldType(fieldType);
    }

    int j;
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        uint8_t labelLength = buf[index++];
        dynamicUser_labelLengths[i] = labelLength;
        dynamicUser_fieldLabels[i] = (char*) malloc(labelLength);

        for(j = 0; j < labelLength; j++)
        {
            dynamicUser_fieldLabels[i][j] = buf[index++];
        }
    }
}
void rx_data(uint8_t *shBuf, uint16_t index)
{
    if(!dynamicUser_fieldTypes) return;

    uint8_t* readIn = (uint8_t*)(&dynamicUserData);

    uint16_t length = 0;
    int i;
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        length += dynamicUser_fieldTypes[i];
    }

    for(i = 0; i < length; i++)

    int i;
    for(i=0;i<length;i++)
        shBuf[index++] = writeOut[i];

    *len = index;
}

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA;

    uint8_t sentMetaData = (buf[index++] == SEND_METADATA);
    if(sentMetaData)
    {
        rx_metaData(buf, index);
    }
    else 
    {
        rx_data(buf, index);   
    }
}

void rx_cmd_user_dyn_r(uint8_t *buf, uint8_t *info)
{
    uint16_t index = P_DATA;
    uint8_t shouldSendMetaData = buf[index++];
    (void)buf;

    if(shouldSendMetaData == SEND_METADATA)
        tx_cmd_user_dyn_sendMetaData(TX_N_DEFAULT);
    else if(shouldSendMetaData == SEND_DATA)
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


