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
		result = reg.setOptMode<REG::OPMODE>(TransceiverModes::SLEEP);
		TEST_ASSERT_EQUAL_UINT8(0x80, result);
		result = reg.setOptMode<REG::OPMODE>(TransceiverModes::STANDBY);
		TEST_ASSERT_EQUAL_UINT8(0x81, result);
		result = reg.setOptMode<REG::OPMODE>(TransceiverModes::FSRx);
		TEST_ASSERT_EQUAL_UINT8(0x84, result);
		result = reg.setOptMode<REG::OPMODE>(LongRangeMode::FSK_OOK);
		TEST_ASSERT_EQUAL_UINT8(0x04, result);
	}
	// Test 2: Using LongRangeMode only.
	{
		Register reg(REG::OPMODE, REG_MODE::READ_WRITE, 0x01, 0);

		// Calling with RX; assuming RX is defined as 5.
		uint8_t result = reg.setOptMode<REG::OPMODE, LoRaChip::ChipModel::SX1277>(
			LoRaChip::LowFreqMode::LOW_FREQUENCY_MODE);

		TEST_ASSERT_EQUAL_UINT8(0x08, result);
	}
	// Test 4: Using the ConfigureOptMode structure.
	{
		Register reg(REG::OPMODE, REG_MODE::READ_WRITE, 0x01, 0);
		ConfigureOptMode<Model::SX1276> config = {
			LongRangeMode::LORA, // LORA → (1 << 7) = 128
			AccessSharedReg::ACCESS_FSK, // ACCESS_FSK → (1 << 6) = 64
			LowFreqMode::HIGH_FREQUENCY_MODE,
			TransceiverModes::TX // TX → 3
		};
		uint8_t result = reg.setOptMode<REG::OPMODE>(config);
		// Expected: 128 | 64 | 3 = 195
		TEST_ASSERT_EQUAL_UINT8(195, result);
	}
}
