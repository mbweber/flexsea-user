#include "dynamic_user_structs.h"

#ifdef __cplusplus
    extern "C" {
#endif

struct DynamicUserData_s
{
    int a;
    int b;
};

const uint8_t DYNAMIC_USER_NUM_FIELDS = 2;

const uint8_t fieldTypes[DYNAMIC_USER_NUM_FIELDS] = {4, 4};

// Keep your label names short, each character is a byte, which takes a lot of memory to send in one packet
// If label names are too long we may fail to send meta data info to plan.
const char* fieldLabels[DYNAMIC_USER_NUM_FIELDS] = {"a", "b"}; 


#ifdef BOARD_TYPE_FLEXSEA_PLAN

uint8_t* dynamicUser_data = NULL;

uint8_t dynamicUser_numFields = 0;
uint8_t* dynamicUser_fieldTypes = NULL;
uint8_t* dynamicUser_fieldLengths = NULL;
uint8_t* dynamicUser_labelLengths = NULL;
char** dynamicUser_fieldLabels = NULL;    

#endif

#ifdef __cplusplus
}
#endif