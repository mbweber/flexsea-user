#ifdef BOARD_TYPE_FLEXSEA_PLAN

#include <stdlib.h>
#include <flexsea.h>
#include <dynamic_user_structs.h>
#include <flexsea_system.h>
#include "flexsea_dataformats.h"
#include <string.h>
#include <flexsea_cmd_user.h>

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
uint8_t waitingOnFieldFlags = 0;

int dynamicUser_slaveId = -1;
uint8_t dynamicUser_numFields = 0;
uint8_t* dynamicUser_data = NULL;
uint8_t* dynamicUser_fieldTypes = NULL;
uint8_t* dynamicUser_fieldLengths = NULL;
uint8_t* dynamicUser_labelLengths = NULL;
char** dynamicUser_labels = NULL;

uint8_t* dynamicUser_fieldFlagsPlan = NULL;
uint8_t* dynamicUser_fieldFlagsExec = NULL;

void* getMemory(void* ptr, int size)
{
	if(ptr)
        ptr = realloc(ptr, size);
	else
	{
		ptr = malloc(size);
		memset(ptr, 0, size);
	}

	return ptr;
}

uint8_t sizeOfFieldType(uint8_t format)
{
	if(format > FORMAT_8S) //for unknown format we just over allocate I guess
		return 8;

	return FORMAT_SIZE_MAP[format];
}

void rx_metaData(uint8_t *buf, uint16_t index)
{
	dynamicUser_slaveId = buf[P_XID];

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

		dynamicUser_fieldTypes =    (uint8_t*) getMemory(dynamicUser_fieldTypes, sizeof(uint8_t)*numFields);
		dynamicUser_fieldLengths =  (uint8_t*) getMemory(dynamicUser_fieldLengths, sizeof(uint8_t)*numFields);
		dynamicUser_labelLengths =  (uint8_t*) getMemory(dynamicUser_labelLengths, sizeof(uint8_t)*numFields);
		dynamicUser_labels =        (char**)   getMemory(dynamicUser_labels, numFields*sizeof(char*));
		int i;
		for(i=0; i < numFields; i++)
			dynamicUser_labels[i] = NULL;

		dynamicUser_fieldFlagsExec =  (uint8_t*) getMemory(dynamicUser_fieldFlagsExec, sizeof(uint8_t)*numFields);
		dynamicUser_fieldFlagsPlan =  (uint8_t*) getMemory(dynamicUser_fieldFlagsPlan, sizeof(uint8_t)*numFields);

		if(numFields > dynamicUser_numFields)
        {
            memset(dynamicUser_fieldTypes + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
            memset(dynamicUser_fieldLengths + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
            memset(dynamicUser_labelLengths + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
            memset(dynamicUser_fieldFlagsExec + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
			memset(dynamicUser_fieldFlagsPlan + dynamicUser_numFields, 0, numFields - dynamicUser_numFields);
		}
	}

	dynamicUser_numFields = numFields;

	//parse field types
	int i;
	int length = 0;
	for(i = 0; i < dynamicUser_numFields; i++)
	{
		uint8_t fieldType = buf[index++];
		if(dynamicUser_fieldTypes[i] != fieldType)
		   dynamicUser_fieldTypes[i] = fieldType;
		dynamicUser_fieldLengths[i] = sizeOfFieldType(fieldType);
		length += dynamicUser_fieldLengths[i];
	}

	static int lastLength = -1;
	if(length != lastLength)
	{
		dynamicUser_data =  (uint8_t*) getMemory(dynamicUser_data, length);
        memset(dynamicUser_data, 0, length);
//        if(length > lastLength)
		lastLength = length;
	}

	//parse label of each field
	int j;
	for(i = 0; i < dynamicUser_numFields; i++)
	{
		//parse label length
		uint8_t labelLength = buf[index++];

		//allocate for label length
		if(labelLength != dynamicUser_labelLengths[i] || !dynamicUser_labels[i])
		{
			dynamicUser_labelLengths[i] = labelLength;
			dynamicUser_labels[i] = (char*) getMemory(dynamicUser_labels[i], labelLength);
		}

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

	uint8_t totalBytesToRead = shBuf[index++];

	int i, j, fieldLength, fieldOffset = 0, totalBytesRead = 0;
	for(i = 0; i < dynamicUser_numFields; i++)
	{
		fieldLength = 1;
		if(dynamicUser_fieldTypes[i] < 8 && FORMAT_SIZE_MAP[dynamicUser_fieldTypes[i]] > 0)
			fieldLength = FORMAT_SIZE_MAP[dynamicUser_fieldTypes[i]];

		if(dynamicUser_fieldFlagsExec[i])
		{
			for(j = 0; j < fieldLength && totalBytesRead < totalBytesToRead; j++)
			{
				dynamicUser_data[fieldOffset + j] = shBuf[index++];
				totalBytesRead++;
			}
		}
		else
		{
			for(j = 0; j < fieldLength; j++)
			{
				dynamicUser_data[fieldOffset + j] = 0;
			}
		}

		fieldOffset += fieldLength;
	}

	newDataAvailable = 1;
}

void rx_cmd_user_dyn_rr(uint8_t *buf, uint8_t *info)
{
	(void)info;

	uint16_t index = P_DATA1;

	uint8_t flag = buf[index++];
	if(flag == SEND_METADATA)
	{
		rx_metaData(buf, index);
	}
	else if(flag == SEND_FIELD_FLAGS)
	{
		if(!unpackFieldFlags(buf+index, dynamicUser_fieldFlagsExec, dynamicUser_numFields))
			waitingOnFieldFlags = 0;
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

void tx_cmd_user_dyn_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{

	*cmd = CMD_USER_DYNAMIC;
	*cmdType = CMD_WRITE;

	uint16_t index = packFieldFlags(shBuf, dynamicUser_numFields, dynamicUser_fieldFlagsPlan);
	waitingOnFieldFlags = 1;
	*len = index;
}

void init_flexsea_payload_ptr_dynamic()
{
	flexsea_payload_ptr[CMD_USER_DYNAMIC][RX_PTYPE_REPLY] = &rx_cmd_user_dyn_rr;
}

#ifdef __cplusplus
	}
#endif

#endif 	//BOARD_TYPE_FLEXSEA_PLAN
