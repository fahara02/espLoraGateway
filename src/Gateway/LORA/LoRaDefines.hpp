#ifndef LORA_DEFINES_HPP
#define LORA_DEFINES_HPP
#include "LoRaConfigs.hpp"
#include "etl/unordered_map.h"
#include "etl/array.h"
#include "GlobalDefines.hpp"
namespace LoRa
{
enum class LoRaDevices
{
	CLASS_A_ALL_END_DEVICES,
	CLASS_B_BEACON,
	CLASS_C_CONTINOUS_LISTEN
};

enum class BoardModel
{
	Hallard,
	ComResult,
	ESP32_Wemos,
	ESP32_TTGO,
	HelTec_Wifi
};
enum class SpreadFactor : uint8_t
{
	SF6 = 6,
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12
};
enum class ReceiverState : uint8_t
{
	S_INIT = 0,
	S_SCAN,
	S_CAD,
	S_RX,
	S_TX,
	S_TXDONE
};
enum class StatisticsLevel : uint8_t
{
	NONE,
	MESSAGE_STAT,
	MESSAGE_STAT_LOCAL_SERVER,
	GATEWAY_STAT_BASE,
	GATEWAY_STAT_SF,
	GATEWAY_STAT_CHANNEL
};
enum class LoRaBands : uint16_t
{
	EU863_870 = 0,
	EU433,
	US902_928,
	AU915_928,
	CN470_510,
	KR920_923,
	IN865_867,
	AS920_923, // Used in Japan, Malaysia, Singapore
	AS923_925 // Used in Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam
};

struct FrequencyPlan
{
	// Upstream messages
	uint32_t upFreq; // 4 bytes unsigned int32 Frequency
	uint16_t upBW; // 2 bytes e.g. 4/5
	uint8_t upLo; // 1 byte Spreading Factor
	uint8_t upHi; // 1 byte Spreading Factor
	// Downstream messages
	uint32_t dwnFreq; // 4 bytes unsigned int32 Frequency
	uint16_t dwnBW; // 2 bytes BW Specification
	uint8_t dwnLo; // 1 byte Spreading Factor
	uint8_t dwnHi; // 1 byte Spreading Factor
};

using LoRaFreqTable = etl::array<FrequencyPlan, LORA_MAX_CHANNEL>;
const etl::unordered_map<LoRaBands, LoRaFreqTable, LORA_MAX_REGION> LoRaFrequencies = {
	{LoRaBands::EU863_870,
	 {{{868100000, 125, 7, 12, 868100000, 125, 7, 12},
	   {868300000, 125, 7, 12, 868300000, 125, 7, 12},
	   {868500000, 125, 7, 12, 868500000, 125, 7, 12},
	   {867100000, 125, 7, 12, 867100000, 125, 7, 12},
	   {867300000, 125, 7, 12, 867300000, 125, 7, 12},
	   {867500000, 125, 7, 12, 867500000, 125, 7, 12},
	   {867700000, 125, 7, 12, 867700000, 125, 7, 12},
	   {867900000, 125, 7, 12, 867900000, 125, 7, 12},
	   {868800000, 125, 7, 12, 868800000, 125, 7, 12},
	   {0, 0, 0, 0, 869525000, 125, 9, 9}}}},

	{LoRaBands::EU433,
	 {{{433175000, 125, 7, 12, 433175000, 125, 7, 12},
	   {433375000, 125, 7, 12, 433375000, 125, 7, 12},
	   {433575000, 125, 7, 12, 433575000, 125, 7, 12},
	   {433775000, 125, 7, 12, 433775000, 125, 7, 12},
	   {433975000, 125, 7, 12, 433975000, 125, 7, 12},
	   {434175000, 125, 7, 12, 434175000, 125, 7, 12},
	   {434375000, 125, 7, 12, 434375000, 125, 7, 12},
	   {434575000, 125, 7, 12, 434575000, 125, 7, 12},
	   {434775000, 125, 7, 12, 434775000, 125, 7, 12},
	   {0, 0, 0, 0, 0, 0, 0, 0}}}},

	{LoRaBands::US902_928,
	 {{{903900000, 125, 7, 10, 923300000, 500, 7, 12},
	   {904100000, 125, 7, 10, 923900000, 500, 7, 12},
	   {904300000, 125, 7, 10, 924500000, 500, 7, 12},
	   {904500000, 125, 7, 10, 925100000, 500, 7, 12},
	   {904700000, 125, 7, 10, 925700000, 500, 7, 12},
	   {904900000, 125, 7, 10, 926300000, 500, 7, 12},
	   {905100000, 125, 7, 10, 926900000, 500, 7, 12},
	   {905300000, 125, 7, 10, 927500000, 500, 7, 12},
	   {904600000, 500, 8, 8, 0, 0, 0, 0}}}},

	{LoRaBands::AU915_928,
	 {{{916800000, 125, 7, 10, 916800000, 125, 7, 12},
	   {917000000, 125, 7, 10, 917000000, 125, 7, 12},
	   {917200000, 125, 7, 10, 917200000, 125, 7, 12},
	   {917400000, 125, 7, 10, 917400000, 125, 7, 12},
	   {917600000, 125, 7, 10, 917600000, 125, 7, 12},
	   {917800000, 125, 7, 10, 917800000, 125, 7, 12},
	   {918000000, 125, 7, 10, 918000000, 125, 7, 12},
	   {918200000, 125, 7, 10, 918200000, 125, 7, 12},
	   {917500000, 500, 8, 8, 0, 0, 0, 0}}}},

	{LoRaBands::CN470_510,
	 {{{486300000, 125, 7, 12, 486300000, 125, 7, 12},
	   {486500000, 125, 7, 12, 486500000, 125, 7, 12},
	   {486700000, 125, 7, 12, 486700000, 125, 7, 12},
	   {486900000, 125, 7, 12, 486900000, 125, 7, 12},
	   {487100000, 125, 7, 12, 487100000, 125, 7, 12},
	   {487300000, 125, 7, 12, 487300000, 125, 7, 12},
	   {487500000, 125, 7, 12, 487500000, 125, 7, 12},
	   {487700000, 125, 7, 12, 487700000, 125, 7, 12}}}},

	{LoRaBands::KR920_923,
	 {{{922100000, 125, 7, 12, 922100000, 125, 7, 12},
	   {922300000, 125, 7, 12, 922300000, 125, 7, 12},
	   {922500000, 125, 7, 12, 922500000, 125, 7, 12},
	   {922700000, 125, 7, 12, 922700000, 125, 7, 12},
	   {922900000, 125, 7, 12, 922900000, 125, 7, 12},
	   {923100000, 125, 7, 12, 923100000, 125, 7, 12},
	   {923300000, 125, 7, 12, 923300000, 125, 7, 12}}}},
	{LoRaBands::AS920_923,
	 {{{923200000, 125, 7, 12, 923200000, 125, 7, 12},
	   {923400000, 125, 7, 12, 923400000, 125, 7, 12},
	   {922200000, 125, 7, 12, 922200000, 125, 7, 12},
	   {922400000, 125, 7, 12, 922400000, 125, 7, 12},
	   {922600000, 125, 7, 12, 922600000, 125, 7, 12},
	   {922800000, 125, 7, 12, 922800000, 125, 7, 12},
	   {923000000, 125, 7, 12, 923000000, 125, 7, 12},
	   {922000000, 125, 7, 12, 922000000, 125, 7, 12},
	   {922100000, 250, 7, 7, 0, 0, 0, 0},
	   {921800000, 0, 0, 0, 921800000, 0, 0, 0}}}},

	{LoRaBands::AS923_925,
	 {{{923200000, 125, 7, 12, 923200000, 125, 7, 12},
	   {923400000, 125, 7, 12, 923400000, 125, 7, 12},
	   {923600000, 125, 7, 12, 923600000, 125, 7, 12},
	   {923800000, 125, 7, 12, 923800000, 125, 7, 12},
	   {924000000, 125, 7, 12, 924000000, 125, 7, 12},
	   {924200000, 125, 7, 12, 924200000, 125, 7, 12},
	   {924400000, 125, 7, 12, 924400000, 125, 7, 12},
	   {924600000, 125, 7, 12, 924600000, 125, 7, 12},
	   {924500000, 250, 7, 7, 0, 0, 0, 0},
	   {924800000, 0, 0, 0, 924800000, 0, 0, 0}}}},

	{LoRaBands::IN865_867,
	 {{{865062500, 125, 7, 12, 865062500, 125, 7, 12},
	   {865402500, 125, 7, 12, 865402500, 125, 7, 12},
	   {865985000, 125, 7, 12, 865985000, 125, 7, 12},
	   {0, 0, 0, 0, 866550000, 125, 10, 10}}}}};

} // namespace LoRa

#endif