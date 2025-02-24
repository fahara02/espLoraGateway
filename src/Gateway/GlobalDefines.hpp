#ifndef GLOBAL_DEFINES_HPP
#define GLOBAL_DEFINES_HPP
#include "stdint.h"

enum class ChipModel
{
	RFM95,
	SX1272,
	SX1273,
	SX1276,
	SX1277,
	SX1278,
	SX1279,
	SX1261,
	SX1262,
};
enum class Series : uint8_t
{
	None = 0,
	RFM,
	SM62,
	SM72,
	SM76,
};

struct Result
{
	bool success = false;
	uint8_t value = 0;
};

#endif