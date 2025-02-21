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

template<ChipModel Model>
struct ConfigureOptMode;

template<>
struct ConfigureOptMode<ChipModel::SX1272>
{
	LongRangeMode long_range;
	AccessSharedReg shared_reg;
	TransceiverModes transceiver;
};

template<>
struct ConfigureOptMode<ChipModel::SX1276>
{
	LongRangeMode long_range;
	AccessSharedReg shared_reg;
	LowFreqMode low_freq;
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

template<ChipModel Model, typename Enable = void>
struct SignalBandWidth
{
	using Type = void; // Default case, no shaping
};

template<ChipModel Model>
struct SignalBandWidth<Model, typename etl::enable_if<(is_sx1272_plus_v<Model>)>::type>
{
	using Type = SignalBandwidth_72;
};

template<ChipModel Model>
struct SignalBandWidth<Model, typename etl::enable_if<(is_sx1276_plus_v<Model>)>::type>
{
	using Type = SignalBandwidth_76;
};
template<ChipModel Model, typename Enable = void>
struct ModemConfig1;

template<ChipModel Model>
struct ModemConfig1<Model, typename std::enable_if<is_sx1272_plus_v<Model>>::type>
{
	using BandwidthType = typename SignalBandWidth<Model>::Type;

	BandwidthType bw;
	CodingRate coding_rate;
	HeaderMode header_mode;
	CRCMode mode; // Enabled only for SX1272/SX1273
	LowDataRateOptimize ldro; // Enabled only for SX1272/SX1273
};

template<ChipModel Model>
struct ModemConfig1<Model, typename std::enable_if<is_sx1276_plus_v<Model>>::type>
{
	using BandwidthType = typename SignalBandWidth<Model>::Type;

	BandwidthType bw;
	CodingRate coding_rate;
	HeaderMode header_mode;
};

} // namespace LoRa
#endif