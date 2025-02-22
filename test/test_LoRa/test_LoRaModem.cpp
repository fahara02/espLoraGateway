#include "test_LoRaModem.hpp"
#include <unity.h>
#define TAG "MODEM_TEST"

void MockLoRaModem::runAllTests()
{
	LOG::ENABLE();
	RUN_TEST(testOptMode);
	RUN_TEST(testBandwidth);
	RUN_TEST(testModemConfig1);
}

void MockLoRaModem::testOptMode()
{
	LOG::TEST(TAG, "Testing optmode function");
	// Test 1: Using LongRangeMode only.
	{
		Register reg(ChipModel::SX1276, REG::OPMODE);
		// Calling with LORA (which is 1). (1 & 1)<<7 gives 128 (0x80).and default value is 1
		uint8_t result = reg.updateOptMode(Field_OptMode::LongRangeMode, LongRangeMode::LORA);
		TEST_ASSERT_EQUAL_UINT8(0x81, result);

		// Calling with RX;
		result = reg.updateOptMode(Field_OptMode::TransceiverModes, TransceiverModes::RX);
		TEST_ASSERT_EQUAL_UINT8(0x85, result);
		result = reg.updateOptMode(Field_OptMode::TransceiverModes, TransceiverModes::SLEEP);
		TEST_ASSERT_EQUAL_UINT8(0x80, result);
		result = reg.updateOptMode(Field_OptMode::TransceiverModes, TransceiverModes::STANDBY);
		TEST_ASSERT_EQUAL_UINT8(0x81, result);
		result = reg.updateOptMode(Field_OptMode::TransceiverModes, TransceiverModes::FSRx);
		TEST_ASSERT_EQUAL_UINT8(0x84, result);
		result = reg.updateOptMode(Field_OptMode::LongRangeMode, LongRangeMode::FSK_OOK);
		TEST_ASSERT_EQUAL_UINT8(0x04, result);
	}
	// Test 2: Using Lowfrequency mode only.
	{
		Register reg(ChipModel::SX1276, REG::OPMODE);

		// Calling with LOW_FREQUENCY_MODE=1; i.e 1000 = 8 and default value is 1 so 1001=9
		uint8_t result =
			reg.updateOptMode(Field_OptMode::LowFreqMode, LowFreqMode::LOW_FREQUENCY_MODE);

		TEST_ASSERT_EQUAL_UINT8(0x09, result);
	}
	// Test 4: Using the ConfigureOptMode structure.
	{
		Register reg(ChipModel::SX1276, REG::OPMODE);
		optModeSetting<Model::SX1276> config = {
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
void MockLoRaModem::testBandwidth()
{
	LOG::TEST(TAG, "Testing BandWidth function");
	MockLoRaModem modem;

	// Test for SX1276 (BW_76)
	using BandwidthType = typename LoRa::LoRaModem<LoRa::ChipModel::SX1276>::Bandwidth;
	BandwidthType bw = BandwidthType::BW_250_KHZ;

	uint8_t result76 =
		modem.setModemConfig1<Field_ModemConfig1::Bandwidth, BandwidthType>(bw, false);
	uint8_t expected76 = static_cast<uint8_t>(SignalBandwidth_76::BW_250_KHZ) << 4;

	TEST_ASSERT_EQUAL_UINT8(expected76, result76);
}

void MockLoRaModem::testModemConfig1()
{
	LOG::TEST(TAG, "Testing ModmeConfig1 function");
	MockLoRaModem modem;
	{
		Register reg(ChipModel::SX1276, REG::MODEM_CONFIG1);
		ModemConfig1Setting<Model::SX1276> config = {
			Bandwidth::BW_7_8_KHZ, // BandWidth→ (0 << 4) = 0
			CodingRate::ERROR_CODING_4_5, // Coding Rate → (1 << 1) = 1
			HeaderMode::EXPLICIT // header->(1<<0)

		};
		uint8_t result = reg.setModemConfig1<REG::MODEM_CONFIG1>(config);
		// Expected: 00000010 = 2
		TEST_ASSERT_EQUAL_UINT8(2, result);
	}
}
