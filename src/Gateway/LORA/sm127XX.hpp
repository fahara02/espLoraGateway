#ifndef SM127XX_HPP
#define SM127XX_HPP
#include <cstdint>
#include "etl/type_traits.h"
#include "etl/optional.h"
#include "etl/array.h"
#include <array>
#include <utility.h>
#include "GlobalDefines.hpp"
#include "Util.hpp"

namespace LoRa
{

enum class unusedType : uint8_t
{
};

enum class MODE
{
	R, // Read
	W, // Write
	RW, // ReadWrite
	SC, // Set toClear
	T, // Trigger
	RWT, // ReadWrite Trigger
	ANY
};
enum class REG
{
	FIFO,
	OPMODE,
	FRF_MSB,
	FRF_MID,
	FRF_LSB,
	PAC,
	PARAMP,
	OCP,
	LNA,
	FIFO_ADDR_PTR,
	FIFO_TX_BASE_AD,
	FIFO_RX_BASE_AD,
	FIFO_RX_CURRENT_ADDR,
	IRQ_FLAGS_MASK,
	IRQ_FLAGS,
	RX_BYTES_NB,
	PKT_SNR_VALUE,
	PKT_RSSI,
	HOP_CHANNEL,
	MODEM_CONFIG1,
	MODEM_CONFIG2,
	SYMB_TIMEOUT_LSB,
	PREAMBLE_MSB,
	PREAMBLE_LSB,
	PAYLOAD_LENGTH,
	MAX_PAYLOAD_LENGTH,
	HOP_PERIOD,
	FIFO_RX_BYTE_ADDR_PTR,
	MODEM_CONFIG3,
	PPM_CORRECTION,
	FREQ_ERROR_MSB,
	FREQ_ERROR_MID,
	FREQ_ERROR_LSB,
	RSSI_WIDEBAND,
	DETECT_OPTIMIZE,
	INVERTIQ,
	DET_TRESH,
	SYNC_WORD,
	INVERTIQ2,
	TEMP,
	DIO_MAPPING_1,
	DIO_MAPPING_2,
	VERSION,
	PADAC
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

constexpr bool isSx126Plus(ChipModel m) { return m == ChipModel::SX1261 || m == ChipModel::SX1262; }

constexpr bool isRfmPlus(ChipModel m) { return m == ChipModel::RFM95; }

enum class optField : uint8_t
{
	LongRangeMode,
	AccessSharedReg,
	LowFreqMode,
	TransceiverModes
};
enum class pacField : uint8_t
{
	PaSelect,
	MaxPower,
	OutputPower
};
enum class PASelect : uint8_t
{
	RFIO_PIN = 0,
	PA_BOOST = 1

};
enum class dBLimit_72 : uint8_t
{
	POWER_LIMIT_PS0 = 13,
	POWER_LIMIT_PS1 = 20
};
enum class dBLimit_76 : uint8_t
{
	POWER_LIMIT_PS0 = 14,
	POWER_LIMIT_PS1 = 20
};

template<ChipModel Model>
struct dBLimitType
{
	using Type = typename etl::conditional_t<
		is_sx1272_plus_v<Model>, dBLimit_72,
		typename etl::conditional_t<is_sx1276_plus_v<Model>, dBLimit_76, unusedType>>;
};

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

enum class config1Field : uint8_t
{
	Bandwidth,
	CodingRate,
	HeaderMode,
	CRC,
	LowDataOptimization
};

struct ConfigParams
{
	uint8_t shift;
	uint8_t mask;
};

template<typename FieldEnum>
struct SettingBase
{
	REG settingFor;

	struct configGroups
	{
		FieldEnum fieldEnum;
		MODE mode;
		Series series;
		ConfigParams params;
		uint8_t reset;
	};

  public:
	constexpr SettingBase(REG reg) : settingFor(reg) {}
};
template<ChipModel Model>
struct optModeSetting : public SettingBase<optField>
{
	LongRangeMode long_range;
	AccessSharedReg shared_reg;
	// Select LowFreqMode for SX1276 (or similar models), or NoLowFreqMode otherwise.
	typename etl::conditional<is_sx1276_plus_v<Model>, LowFreqMode, NoLowFreqMode>::type low_freq;
	TransceiverModes transceiver;

	constexpr optModeSetting() :
		SettingBase<optField>(REG::OPMODE), long_range(LongRangeMode::FSK_OOK), // Default value
		shared_reg(AccessSharedReg::ACCESS_FSK), low_freq(LowFreqMode::LOW_FREQUENCY_MODE),
		transceiver(TransceiverModes::STANDBY)
	{
	}

	constexpr optModeSetting(
		LongRangeMode lr, AccessSharedReg sr,
		typename etl::conditional<is_sx1276_plus_v<Model>, LowFreqMode, NoLowFreqMode>::type lf,
		TransceiverModes tx) :
		SettingBase<optField>(REG::OPMODE), long_range(lr), shared_reg(sr), low_freq(lf),
		transceiver(tx)
	{
	}
};

template<ChipModel Model>
struct PaConfigSetting : public SettingBase<pacField>
{
	using dBLimit = typename dBLimitType<Model>::Type;

	constexpr PaConfigSetting() :
		SettingBase<pacField>(REG::PAC), PaSelect(PASelect::RFIO_PIN), OutPutPower(0),
		powerLimit_dB(getMaxPowerLimit(PaSelect))
	{
	}
	constexpr PaConfigSetting(PASelect pa) :
		SettingBase<pacField>(REG::PAC), PaSelect(pa), OutPutPower(0),
		powerLimit_dB(getMaxPowerLimit(PaSelect))
	{
	}

	constexpr PASelect getPASelect() { return PaSelect; }
	constexpr uint8_t getMaxPower() { return MaxPower; }
	constexpr uint8_t getPOut() { return OutPutPower; }
	uint8_t getSetting()
	{
		uint8_t setting = 0;

		if constexpr(is_sx1276_plus_v<Model>) // SM76 Series
		{
			setting |= (static_cast<uint8_t>(PaSelect) << 7);
			setting |= ((MaxPower & 0x07) << 4); // Bits 6-4
			setting |= (OutPutPower & 0x0F); // Bits 3-0
		}
		else if constexpr(is_sx1272_plus_v<Model>) // SM72 Series
		{
			setting |= (static_cast<uint8_t>(PaSelect) << 7);
			setting |= (OutPutPower & 0x0F); // Bits 3-0
		}

		return setting;
	}

	void setPaSelect(PASelect pa)
	{
		PaSelect = pa;
		powerLimit_dB = getMaxPowerLimit(PaSelect);
	}
	int setOutPutPower(float output_dB)
	{

		int result = -1;

		for(uint8_t max_power_reg = 0; max_power_reg <= 7; max_power_reg++)
		{

			uint8_t candidate_pmax = getPmax_dB(max_power_reg);
			if(candidate_pmax <= powerLimit_dB && output_dB <= candidate_pmax)
			{

				result = calculateOutPutPower(max_power_reg, output_dB);
				if(result != -1)
				{

					if(result >= 0 && result <= 15)
					{
						OutPutPower = static_cast<uint8_t>(result);
						return OutPutPower;
					}
				}
			}
		}
		return result;
	}

  private:
	PASelect PaSelect;
	uint8_t MaxPower = 0x04;
	uint8_t OutPutPower;
	// dB unit data
	uint8_t powerLimit_dB;

	static constexpr uint8_t getMaxPowerLimit(PASelect pa)
	{

		return static_cast<uint8_t>(pa == PASelect::RFIO_PIN ? dBLimit::POWER_LIMIT_PS0 :
															   dBLimit::POWER_LIMIT_PS1);
	}

	float getPmax_dB(uint8_t max_power_reg)
	{
		Series chip_Series = Util::GetSeries<Model>();
		return chip_Series == Series::SM76 ? static_cast<int>(10.8 + 0.6 * max_power_reg) :
											 powerLimit_dB;
	}

	int calculateOutPutPower(uint8_t MaxPower_Reg, float required_output_dB)
	{
		int result = -1;
		float pMax_dB = 0;

		Series chip_Series = Util::GetSeries<Model>();
		if(chip_Series == Series::SM76)
		{
			pMax_dB = getPmax_dB(MaxPower_Reg);
			result = (PaSelect == PASelect::RFIO_PIN) ? (required_output_dB - pMax_dB + 15) :
														(required_output_dB - 2);
		}
		else
		{
			result = (PaSelect == PASelect::RFIO_PIN) ? (required_output_dB - 2) :
														(required_output_dB + 1);
		}

		return result;
	}
};

template<ChipModel Model>
struct ModemConfig1Setting : public SettingBase<config1Field>
{
	using BandwidthType = typename SignalBandWidth<Model>::Type;
	BandwidthType bw;
	CodingRate coding_rate;
	HeaderMode header_mode;

	// Only available for SX1272/SX1273, using std::optional to avoid void issues
	etl::optional<CRCMode> mode;
	etl::optional<LowDataRateOptimize> ldro;

	constexpr ModemConfig1Setting() :
		SettingBase<config1Field>(REG::MODEM_CONFIG1),
		bw(static_cast<BandwidthType>(0)), // Default bandwidth
		coding_rate(CodingRate::ERROR_CODING_4_5), header_mode(HeaderMode::IMPLICIT),
		mode(etl::nullopt), ldro(etl::nullopt)
	{
	}

	constexpr ModemConfig1Setting(BandwidthType bw_, CodingRate cr, HeaderMode hm) :
		SettingBase<config1Field>(REG::MODEM_CONFIG1), bw(bw_), coding_rate(cr), header_mode(hm),
		mode(etl::nullopt), ldro(etl::nullopt)
	{
	}

	// Constructor for SX1272/SX1273 models (with `mode` and `ldro`).
	template<typename T = void, typename etl::enable_if_t<isSx1272Plus(Model), T>* = nullptr>
	constexpr ModemConfig1Setting(BandwidthType bw_, CodingRate cr, HeaderMode hm, CRCMode m,
								  LowDataRateOptimize ldr) :
		SettingBase<config1Field>(REG::MODEM_CONFIG1), bw(bw_), coding_rate(cr), header_mode(hm),
		mode(m), ldro(ldr)
	{
	}
};
enum class defaultField : uint8_t
{
};
template<REG reg>
struct FieldTypeForReg
{
	using Type = defaultField;
};

template<>
struct FieldTypeForReg<REG::OPMODE>
{
	using Type = optField;
};

template<>
struct FieldTypeForReg<REG::MODEM_CONFIG1>
{
	using Type = config1Field;
};

template<REG reg>
using FieldTypeForReg_t = typename FieldTypeForReg<reg>::Type;

} // namespace LoRa
#endif