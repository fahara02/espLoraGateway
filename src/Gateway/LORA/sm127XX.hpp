#ifndef SM127XX_HPP
#define SM127XX_HPP
#include <cstdint>
#include "etl/type_traits.h"
#include "etl/optional.h"

namespace LoRa
{
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

template<ChipModel Model>
static constexpr bool is_sx1276_plus_v = Model == ChipModel::SX1276 || Model == ChipModel::SX1277 ||
										 Model == ChipModel::SX1278 || Model == ChipModel::SX1279;

template<ChipModel Model>
static constexpr bool is_sx1272_plus_v = Model == ChipModel::SX1272 || Model == ChipModel::SX1273;
template<ChipModel Model>
static constexpr bool is_sx162_plus_v = Model == ChipModel::SX1261 || Model == ChipModel::SX1262;
template<ChipModel Model>
static constexpr bool is_rfm_plus_v = Model == ChipModel::RFM95;

constexpr bool isSx1276Plus(ChipModel m)
{
	return m == ChipModel::SX1276 || m == ChipModel::SX1277 || m == ChipModel::SX1278 ||
		   m == ChipModel::SX1279;
}

constexpr bool isSx1272Plus(ChipModel m)
{
	return m == ChipModel::SX1272 || m == ChipModel::SX1273;
}

constexpr bool isSx126Plus(ChipModel m)
{
	return m == ChipModel::SX1261 || m == ChipModel::SX1262;
}

constexpr bool isRfmPlus(ChipModel m)
{
	return m == ChipModel::RFM95;
}

enum class ChipSeries : uint8_t
{
	None = 0,
	RFM = 1 << 0,
	SM62 = 1 << 1,
	SM72 = 1 << 2,
	SM76 = 1 << 3,
};

constexpr ChipSeries operator|(ChipSeries a, ChipSeries b)
{
	return static_cast<ChipSeries>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

constexpr bool hasFlag(ChipSeries series, ChipSeries flag)
{
	return (static_cast<uint8_t>(series) & static_cast<uint8_t>(flag)) != 0;
}

constexpr ChipSeries getChipSeries(ChipModel model)
{
	switch(model)
	{
		case ChipModel::RFM95:
			return ChipSeries::RFM;
		case ChipModel::SX1272:
		case ChipModel::SX1273:
			return ChipSeries::SM72;
		case ChipModel::SX1276:
		case ChipModel::SX1277:
		case ChipModel::SX1278:
		case ChipModel::SX1279:
			return ChipSeries::SM76;
		case ChipModel::SX1261:
		case ChipModel::SX1262:
			return ChipSeries::SM62;
		default:
			return ChipSeries::None;
	}
}

enum class LongRangeMode : uint8_t
{
	FSK_OOK = 0,
	LORA = 1,
};

enum class AccessSharedReg : uint8_t
{
	ACCESS_LORA = 0x0,
	ACCESS_FSK = 0x1,
};
enum class LowFreqMode : uint8_t
{
	HIGH_FREQUENCY_MODE = 0x0,
	LOW_FREQUENCY_MODE = 0x1,
};

enum class TransceiverModes : uint8_t
{
	SLEEP = 0x0,
	STANDBY = 0x1,
	FSTx = 0x2, // Frequency Synthesis TX
	TX = 0x3, // Transmit
	FSRx = 0x4, // Frequency Synthesis RX
	RX = 0x5, // Receive Continous
	RX_SINGLE = 0x6, // Receive Single
	CAD = 0x7 // Channel Activity Detection
};

struct NoLowFreqMode
{
};

template<ChipModel Model>
struct ConfigureOptMode
{
	LongRangeMode long_range;
	AccessSharedReg shared_reg;
	// Select LowFreqMode for SX1276 (or similar models), or NoLowFreqMode otherwise.
	typename etl::conditional<is_sx1276_plus_v<Model>, LowFreqMode, NoLowFreqMode>::type low_freq;
	TransceiverModes transceiver;
};

enum class CodingRate : uint8_t
{
	ERROR_CODING_4_5 = 0x1,
	ERROR_CODING_4_6 = 0x2,
	ERROR_CODING_4_7 = 0x3,
	ERROR_CODING_4_8 = 0x4,
};

enum class HeaderMode : uint8_t
{
	EXPLICIT = 0,
	IMPLICIT = 1,
};
enum class CRCMode : uint8_t
{
	DISABLE = 0,
	ENABLE = 1,
};
enum class LowDataRateOptimize : uint8_t
{
	DISABLE = 0,
	ENABLE = 1,
};

enum class SignalBandwidth_72 : uint8_t
{
	// SX1272/73 bandwidth settings (2 bits)
	BW_125_KHZ = 0b00, // 125 kHz
	BW_250_KHZ = 0b01, // 250 kHz
	BW_500_KHZ = 0b10, // 500 kHz
	BW_RESERVED = 0b11 // Reserved
};

enum class SignalBandwidth_76 : uint8_t
{
	// SX1276/77/78/79 bandwidth settings (4 bits)
	BW_7_8_KHZ = 0b0000, // 7.8 kHz
	BW_10_4_KHZ = 0b0001, // 10.4 kHz
	BW_15_6_KHZ = 0b0010, // 15.6 kHz
	BW_20_8_KHZ = 0b0011, // 20.8 kHz
	BW_31_25_KHZ = 0b0100, // 31.25 kHz
	BW_41_7_KHZ = 0b0101, // 41.7 kHz
	BW_62_5_KHZ = 0b0110, // 62.5 kHz
	BW_125_KHZ = 0b0111, // 125 kHz
	BW_250_KHZ = 0b1000, // 250 kHz
	BW_500_KHZ = 0b1001, // 500 kHz
};

template<ChipModel Model>
struct SignalBandWidth
{
	using Type = typename etl::conditional<
		is_sx1272_plus_v<Model>, SignalBandwidth_72,
		typename etl::conditional<is_sx1276_plus_v<Model>, SignalBandwidth_76, void>::type>::type;
};

enum class Field_ModemConfig1 : uint8_t
{
	Bandwidth,
	CodingRate,
	HeaderMode,
	CRC,
	LowDataOptimization
};

template<ChipModel Model>
struct Setting_ModemConfig1
{
	using BandwidthType = typename SignalBandWidth<Model>::Type;
	BandwidthType bw;
	CodingRate coding_rate;
	HeaderMode header_mode;

	// Only available for SX1272/SX1273:
	etl::conditional_t<isSx1272Plus(Model), CRCMode, void> mode;
	etl::conditional_t<isSx1272Plus(Model), LowDataRateOptimize, void> ldro;
};

struct ConfigParams
{
	uint8_t shift;
	uint8_t mask;
};

template<typename FieldType>
struct FieldConfig
{
	FieldType field; // Field identifier (e.g. ModemConfig1Field, or another enum)
	ChipSeries chip_series;
	ConfigParams params; // The bit shift and mask for this field
};

using ModemConfig1FieldConfig = FieldConfig<Field_ModemConfig1>;

} // namespace LoRa
#endif