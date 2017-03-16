#include "dynamic_user_structs.h"
#include <stdlib.h>
#include "flexsea_system.h"
#ifdef __cplusplus
	extern "C" {
#endif

/* This works for Plan - Execute only
    Requirements:

    - send labels to GUI
    - send data to GUI
    - keep msg under max size

*/
uint8_t newMetaDataAvailable = 0;
uint8_t newDataAvailable = 0;

uint8_t dynamicUser_numFields = 0;
uint8_t* dynamicUser_data = NULL;
uint8_t* dynamicUser_fieldTypes = NULL;
uint8_t* dynamicUser_fieldLengths = NULL;
uint8_t* dynamicUser_labelLengths = NULL;
char** dynamicUser_labels = NULL;  

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


    //parse number of fields
	uint8_t numFields = buf[index++];
	if(numFields != dynamicUser_numFields)
	{
		if(dynamicUser_labels)
		{
			int i;
			for(i=0; i<dynamicUser_numFields;i++)
			{
				char* label = dynamicUser_labels[i];
				if(label) { free(label); }
				label = NULL;
			}
		}

		dynamicUser_fieldTypes =    (uint8_t*) getMemory(dynamicUser_fieldTypes, dynamicUser_numFields);
		dynamicUser_fieldLengths =  (uint8_t*) getMemory(dynamicUser_fieldLengths, dynamicUser_numFields);
		dynamicUser_labelLengths =  (uint8_t*) getMemory(dynamicUser_labelLengths, dynamicUser_numFields);
		dynamicUser_labels =        (char**)   getMemory(dynamicUser_labels, dynamicUser_numFields*sizeof(char*));
	}
	dynamicUser_numFields = numFields;

    //parse field types
    int i; 
	static int lastLength = -1;
	int length = 0;
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        uint8_t fieldType = buf[index++];
        dynamicUser_fieldTypes[i] = fieldType;
        dynamicUser_fieldLengths[i] = sizeOfFieldType(fieldType);
		length += dynamicUser_fieldLengths[i];
    }
	if(length != lastLength)
		dynamicUser_data =  (uint8_t*) getMemory(dynamicUser_data, length);

    //parse label of each field
    int j;
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        //parse label length
        uint8_t labelLength = buf[index++];
        dynamicUser_labelLengths[i] = labelLength;
        //allocate for label length
		dynamicUser_labels[i] = (char*) malloc(labelLength);
        //parse  label
        for(j = 0; j < labelLength; j++)
        {
			dynamicUser_labels[i][j] = buf[index++];
        }
	}
    newMetaDataAvailable = 1;
}
void rx_data(uint8_t *shBuf, uint16_t index)
{
    if(!dynamicUser_fieldTypes) return;
	if(!dynamicUser_data) return;

    uint16_t length = 0;
    int i;
    for(i = 0; i < dynamicUser_numFields; i++)
    {
        length += sizeOfFieldType(dynamicUser_fieldTypes[i]);
    }

	uint8_t* readIn = (uint8_t*)(dynamicUser_data);
    for(i = 0; i < length; i++)
    {
        readIn[i] = shBuf[index++];
    }

    newDataAvailable = 1;
}

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = P_DATA1;

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

void tx_cmd_user_dyn_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
                        uint8_t sendMetaData)
{
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_READ;

    uint16_t index = 0;
    shBuf[index++] = sendMetaData;
    *len = index;
}

void tx_cmd_user_dyn_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, \
                        uint8_t numOffsets, uint8_t* offsets)
{
    numOffsets = numOffsets > 0 ? numOffsets : 0;
    
    *cmd = CMD_USER_DYNAMIC;
    *cmdType = CMD_READ;

    uint16_t index = 0;
    shBuf[index++] = numOffsets;
    int i;
    for(i = 0; i < numOffsets; i++)
    {
        shBuf[index++] = offsets[i];
    }

    *len = index;
}

void init_flexsea_payload_ptr_dynamic()
{
    flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_REPLY] = &rx_cmd_user_dyn_rr;
}    

#ifdef __cplusplus
	}
#endif
