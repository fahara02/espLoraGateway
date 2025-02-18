#ifndef SPI_BUS_HPP
#define SPI_BUS_HPP

#include "SPI.h"
#include "Lock.hpp"
#include "LoRaConfigs.hpp"
#include "LoraBoards.hpp"
#include "driver/gpio.h"

namespace Core
{
class SPIBus
{
  public:
	static SPIBus& getInstance(uint8_t addr)
	{
		static SPIBus instance{addr};
		return instance;
	}

	void setPins(LoRa::LoRaPins& pins);
	void setupSPI(unsigned long clock, uint8_t bitorder, uint8_t mode);
	uint8_t readRegister(uint8_t addr);
	void writeRegister(uint8_t addr, uint8_t value);
	void writeBuffer(uint8_t addr, uint8_t* buf, uint8_t len);

  private:
	SPIBus(uint8_t addr);
	void init();
	void beginTransaction();
	void endTransaction();
	static void initMutex();

	static SemaphoreHandle_t spiMutex;
	uint8_t address_;
	int miso_;
	int mosi_;
	int sck_;
	int ss_;
	unsigned long clock_;
	SPISettings settings_;
};

} // namespace Core

#endif
