#ifdef __cplusplus
extern "C" {
#endif

#include "flexsea-user_test-all.h"

//Definitions and variables used by some/all tests:

#define TEST_PL_LEN		4

void test_tx_cmd_ctrl_special_5(void)
{
	uint8_t rid = FLEXSEA_EXECUTE_1;
	uint8_t slave = 17, controller = CTRL_OPEN;
	int16_t pwm = -1234, current = 12500;
	int16_t res_pwm = 0, res_current = 0;
	uint8_t cmdCode = CMD_SPC5, cmdType = CMD_READ;
	uint16_t len = TEST_PL_LEN, numb = 0;
//	uint8_t myPayloadData[TEST_PL_LEN] = {0xAA, 0xBB, 0xCC, 0xDD};
	uint8_t outputBuf[48];
	uint16_t index = 0;

//	numb = tx_cmd(myPayloadData, cmdCode, cmdType, len, FLEXSEA_EXECUTE_1, outputBuf);

	numb = tx_cmd_ctrl_special_5(rid, cmdType, outputBuf, PAYLOAD_BUF_LEN, \
								slave, controller, current, pwm);

	TEST_ASSERT_EQUAL(1, outputBuf[P_CMDS]);
	TEST_ASSERT_EQUAL(CMD_R(cmdCode), outputBuf[P_CMD1]);
	TEST_ASSERT_EQUAL(slave, outputBuf[P_DATA1]);
	TEST_ASSERT_EQUAL(controller, outputBuf[P_DATA1+1]);

	//Test for PWM value:
	index = P_DATA1 + 2;
	res_current = (int16_t) REBUILD_UINT16(outputBuf, &index);
	res_pwm = (int16_t) REBUILD_UINT16(outputBuf, &index);
	TEST_ASSERT_EQUAL(current, res_current);
	TEST_ASSERT_EQUAL(pwm, res_pwm);

//	TEST_ASSERT_EQUAL_UINT8_ARRAY(myPayloadData, &outputBuf[P_DATA1], TEST_PL_LEN);
//	TEST_ASSERT_EQUAL((P_DATA1 + TEST_PL_LEN), numb);
}

void test_cmd_user(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_tx_cmd_ctrl_special_5);

	UNITY_END();
}

#ifdef __cplusplus
}
#endif
