#include "test_LoRaModem.hpp"
#include <unity.h>
#define TAG "MODEM_TEST"

void MockLoRaModem::runAllTests()
{
	LOG::ENABLE();
	RUN_TEST(testOptMode);
}

void MockLoRaModem::testOptMode()
{
	LOG::TEST(TAG, "Testing optmode function");
	// Test 1: Using LongRangeMode only.
	{
		Register reg(REG::OPMODE, REG_MODE::READ_WRITE, 0x01, 0);
		// Calling with LORA (which is 1). (1 & 1)<<7 gives 128 (0x80).
		uint8_t result = reg.setOptMode<REG::OPMODE>(LongRangeMode::LORA);
		TEST_ASSERT_EQUAL_UINT8(0x80, result);

		// Calling with RX; assuming RX is defined as 5.
		result = reg.setOptMode<REG::OPMODE>(TransceiverModes::RX);
		TEST_ASSERT_EQUAL_UINT8(0x85, result);
	}
	// Test 2: Using the ConfigureOptMode structure.
	{
		Register reg(REG::OPMODE, REG_MODE::READ_WRITE, 0x01, 0);
		ConfigureOptMode config = {
			LongRangeMode::LORA, // LORA → (1 << 7) = 128
			AccessSharedReg::ACCESS_FSK, // ACCESS_FSK → (1 << 6) = 64
			TransceiverModes::TX // TX → 3
		};
		uint8_t result = reg.setOptMode<REG::OPMODE>(config);
		// Expected: 128 | 64 | 3 = 195
		TEST_ASSERT_EQUAL_UINT8(195, result);
	}
}
