#ifndef LORA_BOARDS_HPP
#define LORA_BOARDS_HPP
#include <cstdint>
#include "etl/optional.h"
#include "LoRaDefines.hpp"

namespace LoRa
{
#define ETL_NO_STL

struct SPIPins
{
	uint8_t SCK;
	uint8_t MISO;
	uint8_t MOSI;
	bool isDefault = false;
};

struct GPSPins
{
	uint8_t RX;
	uint8_t TX;
	bool isAvailable = false;
};
struct LoRaPins
{
	uint8_t dio0;
	uint8_t dio1;
	uint8_t dio2;
	uint8_t ss;
	uint8_t rst;
	etl::optional<SPIPins> spipins;
	etl::optional<GPSPins> gpspins;
};
class LoRaBoard
{
  public:
	static LoRaPins getPinConfig(BoardModel boardType)
	{
		switch(boardType)
		{
			case BoardModel::Hallard:
				return {15, 15, 15, 16, 0, SPIPins{14, 12, 13, true}};
			case BoardModel::ComResult:
				return {5, 4, 0, 15, 0, etl::nullopt};
			case BoardModel::ESP32_Wemos:
				return {26, 26, 26, 18, 14, SPIPins{5, 19, 27, true}};
			case BoardModel::ESP32_TTGO:
				return {26, 33, 32, 18, 14, SPIPins{5, 19, 27, true}, GPSPins{15, 12, true}};
			case BoardModel::HelTec_Wifi:
				return {26, 35, 34, 18, 14, SPIPins{5, 19, 27, true}};
			default:
				return {0, 0, 0, 0, 0};
		}
	}
};
} // namespace LoRa
#endif