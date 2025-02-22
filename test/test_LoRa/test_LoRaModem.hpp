#ifndef TEST_LORA_MODEM_HPP
#define TEST_LORA_MODEM_HPP

#include "LoRaModem.hpp"
#include "LoRaRegisters.hpp"
#include "Logger.hpp"

using namespace LoRa;

class MockLoRaModem : public LoRaModem<ChipModel::SX1276>
{
  public:
	MockLoRaModem() : LoRaModem(BoardModel::ESP32_TTGO, 0x20, LoRaBands::EU433, 0)
	{
	}
	using Model = LoRa::ChipModel;

	static void runAllTests();

  private:
	static void testOptModeRegister();
	static void testOptModeModem();
	static void testBandwidth();
	static void testModemConfig1();
};
#endif