#ifndef TEST_LORA_MODEM_HPP
#define TEST_LORA_MODEM_HPP

#include "LoRaModem.hpp"
#include "LoRaRegisters.hpp"
#include "Logger.hpp"

using namespace LoRa;
using namespace LoRaChip;

class MockLoRaModem : public LoRaModem
{
  public:
	using Model = LoRaChip::ChipModel;
	static void runAllTests();

  private:
	static void testOptMode();
};
#endif