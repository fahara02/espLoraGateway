#ifndef GLOBAL_CONFIGS_HPP
#define GLOBAL_CONFIGS_HPP
#include <Preferences.h>
namespace Core
{
class Config
{
  private:
	Preferences* _prefs;
	int16_t _tcpPort;
	int16_t _modbusPort;
	uint32_t _tcpTimeout;
	unsigned long _modbusBaudRate;
	uint32_t _modbusConfig;
	int8_t _modbusRtsPin;
	unsigned long _serialBaudRate;
	uint32_t _serialConfig;

  public:
	Config();
	void begin(Preferences* prefs);
	uint16_t getTcpPort();
	void setTcpPort(uint16_t value);
	uint16_t getModbusPort();
	void setModbusPort(uint16_t value);
	uint32_t getTcpTimeout();
	void setTcpTimeout(uint32_t value);
	uint32_t getModbusConfig();
	unsigned long getModbusBaudRate();
	void setModbusBaudRate(unsigned long value);
	uint8_t getModbusDataBits();
	void setModbusDataBits(uint8_t value);
	uint8_t getModbusParity();
	void setModbusParity(uint8_t value);
	uint8_t getModbusStopBits();
	void setModbusStopBits(uint8_t value);
	int8_t getModbusRtsPin();
	void setModbusRtsPin(int8_t value);
	uint32_t getSerialConfig();
	unsigned long getSerialBaudRate();
	void setSerialBaudRate(unsigned long value);
	uint8_t getSerialDataBits();
	void setSerialDataBits(uint8_t value);
	uint8_t getSerialParity();
	void setSerialParity(uint8_t value);
	uint8_t getSerialStopBits();
	void setSerialStopBits(uint8_t value);
};
} // namespace Core
#endif