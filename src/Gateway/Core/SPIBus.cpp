#include "SPIBus.hpp"
#include "Logger.hpp"

namespace Core
{
SemaphoreHandle_t SPIBus::spiMutex = nullptr;

SPIBus::SPIBus(uint8_t addr) :
	address_{addr}, miso_{-1}, mosi_{-1}, sck_{-1}, ss_{-1}, clock_{1000000}
{
	initMutex();
	init();
}

void SPIBus::initMutex()
{
	static bool initialized = false;
	if(!initialized)
	{
		spiMutex = xSemaphoreCreateMutex();
		if(spiMutex == nullptr)
		{
			LOG::ERROR("SPI_BUS", "Failed to create SPI mutex");
		}
		else
		{
			initialized = true;
		}
	}
}

void SPIBus::init()
{
	settings_ = SPISettings(clock_, MSBFIRST, SPI_MODE0);
	gpio_set_direction((gpio_num_t)ss_, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)ss_, 1); // Set SS high initially
	SPI.begin(sck_, miso_, mosi_, ss_); // Initialize SPI with default pins (can be changed later)
}

void SPIBus::setPins(LoRa::LoRaPins& pins)
{
	if(pins.spipins && pins.spipins.value().isDefault)
	{
		sck_ = pins.spipins.value().SCK;
		mosi_ = pins.spipins.value().MOSI;
		miso_ = pins.spipins.value().MISO;
	}

	ss_ = pins.ss;

	// Reinitialize SPI with updated pins
	SPI.end();
	SPI.begin(sck_, miso_, mosi_, ss_);

	// Reconfigure SS pin with ESP-IDF
	gpio_set_direction((gpio_num_t)ss_, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)ss_, 1);
}

void SPIBus::setupSPI(unsigned long clock, uint8_t bitorder, uint8_t mode)
{
	clock_ = clock;
	settings_ = SPISettings(clock, bitorder, mode);
}

void SPIBus::beginTransaction()
{
	if(spiMutex)
	{
		xSemaphoreTake(spiMutex, portMAX_DELAY);
	}
	SPI.beginTransaction(settings_);
	gpio_set_level((gpio_num_t)ss_, 0); // Pull SS low
}

void SPIBus::endTransaction()
{
	gpio_set_level((gpio_num_t)ss_, 1); // Pull SS high
	SPI.endTransaction();
	if(spiMutex)
	{
		xSemaphoreGive(spiMutex);
	}
}

uint8_t SPIBus::readRegister(uint8_t addr)
{
	beginTransaction();
	SPI.transfer(addr & 0x7F); // Read command (bit 7 = 0)
	uint8_t value = static_cast<uint8_t>(SPI.transfer(0x00));
	endTransaction();
	return value;
}

void SPIBus::writeRegister(uint8_t addr, uint8_t value)
{
	beginTransaction();
	SPI.transfer((addr | 0x80) & 0xFF); // Write command (bit 7 = 1)
	SPI.transfer(value & 0xFF);
	endTransaction();
}
void SPIBus::writeBuffer(uint8_t addr, uint8_t* buf, uint8_t len)
{
	beginTransaction();
	if(len >= 24)
	{
		len = 24;
		LOG::WARNING("SPIBus", " writeBuffer:: len > 24");
	}
	if(LoRa::BUFFER_BYTES == 1)
	{
		SPI.transfer((uint8_t*)buf, len);
	}
	else
	{
		for(int i = 0; i < len; i++)
		{
			SPI.transfer(buf[i]);
		}
	}
	endTransaction();
	if(LoRa::Monitor)
	{
	}
}
} // namespace Core
