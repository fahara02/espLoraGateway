#ifndef LORA_REGISTERS_HPP
#define LORA_REGISTERS_HPP

#include "sm127XX.hpp"
#include "etl/array.h"
#include "etl/optional.h"

#include "Logger.hpp"
#define LORA_REG "LORA_REGISTER"

namespace LoRa
{
enum class REG_MODE
{
	READ_ONLY,
	WRITE_ONLY,
	READ_WRITE,
	SET_TO_CLEAR,
	TRIGGER,
	READ_WRITE_TRIGGER,
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

struct RegInfo
{
	REG reg;
	uint8_t address;
	REG_MODE mode;
	uint8_t defaultValue;
	const void* fieldConfigs;
	size_t fieldConfigCount;
};

constexpr OptModeFieldConfig OptModeFieldConfigs[] = {
	// LongRangeMode:
	{Field_OptMode::LongRangeMode, ChipSeries::SM72, {7, 0b10000000}},
	{Field_OptMode::LongRangeMode, ChipSeries::SM76, {7, 0b10000000}},
	// AccessShared Reg:
	{Field_OptMode::AccessSharedReg, ChipSeries::SM72, {6, 0b01000000}},
	{Field_OptMode::AccessSharedReg, ChipSeries::SM76, {6, 0b01000000}},
	// LowFreqMode (only defined for Sm76 series):
	{Field_OptMode::LowFreqMode, ChipSeries::SM76, {3, 0b00001000}},
	// TransceiverModes
	{Field_OptMode::TransceiverModes, ChipSeries::SM72, {0, 0b00000111}},
	{Field_OptMode::TransceiverModes, ChipSeries::SM76, {0, 0b00000111}},

};

constexpr ModemConfig1FieldConfig modemConfig1FieldConfigs[] = {
	// Bandwidth:
	{Field_ModemConfig1::Bandwidth, ChipSeries::SM72, {6, 0b11000000}},
	{Field_ModemConfig1::Bandwidth, ChipSeries::SM76, {4, 0b11110000}},
	// Coding Rate:
	{Field_ModemConfig1::CodingRate, ChipSeries::SM72, {3, 0b00111000}},
	{Field_ModemConfig1::CodingRate, ChipSeries::SM76, {1, 0b00001110}},
	// HeaderMode
	{Field_ModemConfig1::HeaderMode, ChipSeries::SM72, {2, 0b00000100}},
	{Field_ModemConfig1::HeaderMode, ChipSeries::SM76, {0, 0b00000001}},
	// CRC (only defined for Sm72 series):
	{Field_ModemConfig1::CRC, ChipSeries::SM72, {1, 0b00000010}},
	// Low Data Optimization (only defined for Sm72 series):
	{Field_ModemConfig1::LowDataOptimization, ChipSeries::SM72, {0, 0b00000001}}};

struct Register
{
	const ChipModel model;
	const REG reg;
	const REG_MODE mode;
	const uint8_t address;

	template<ChipModel Model>
	using Bandwidth = typename SignalBandWidth<Model>::Type;

	constexpr Register() :
		model(ChipModel::SX1276), reg(REG::FIFO), mode(REG_MODE::READ_WRITE), address(0), value_(0)
	{
	}
	constexpr Register(ChipModel m, REG r) :
		model(m), reg(r), mode(getRegMode(r)), address(getRegAddress(r)),
		value_(getDefaultValue(r, m))
	{
	}
	constexpr uint8_t getRegAddress(REG r)
	{
		const RegInfo* info = lookupRegInfo(r);
		return info ? info->address : 0xFF;
	}
	constexpr uint8_t getDefaultValue(REG r, ChipModel m)
	{
		const RegInfo* info = lookupRegInfo(r);
		return info ? info->defaultValue : 0xFF;
	}
	constexpr REG_MODE getRegMode(REG r)
	{
		const RegInfo* info = lookupRegInfo(r);
		return info ? info->mode : REG_MODE::ANY;
	}

	template<REG Reg, typename Field>
	uint8_t updateRegisterField(const Field field, uint8_t new_value)
	{
		const ChipSeries series = isSx1272Plus(model) ? ChipSeries::SM72 : ChipSeries::SM76;

		ConfigParams params = getFieldConfigParams(Reg, field, series);

		uint8_t shifted_value = new_value << params.shift;
		updateBits(params.mask, shifted_value);

		return value_;
	}

	template<typename ValueType>

	uint8_t updateOptMode(const Field_OptMode field, ValueType new_value)
	{
		return updateRegisterField<REG::OPMODE, Field_OptMode>(field,
															   static_cast<uint8_t>(new_value));
	}

	template<typename ValueType>
	uint8_t updateModemConfig(const Field_ModemConfig1 field, ValueType new_value)
	{
		return updateRegisterField<REG::MODEM_CONFIG1, Field_ModemConfig1>(
			field, static_cast<uint8_t>(new_value));
	}

	template<REG T>
	typename etl::enable_if<T == REG::OPMODE, uint8_t>::type setOptMode(const LongRangeMode mode)
	{
		value_ = (value_ & ~(1 << 7)) | ((static_cast<uint8_t>(mode) & 0b1) << 7);
		return value_;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<(T == REG::OPMODE && is_sx1276_plus_v<Model>), uint8_t>::type
		setOptMode(const LowFreqMode mode)
	{
		return value_ = (value_ & ~(1 << 3)) | (static_cast<uint8_t>(mode) & 0b1) << 3;
	}

	template<REG T>
	typename etl::enable_if<(T == REG::OPMODE), uint8_t>::type
		setOptMode(const TransceiverModes mode)
	{
		value_ = (value_ & ~0b111) | (static_cast<uint8_t>(mode) & 0b111);
		return value_;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::OPMODE, uint8_t>::type
		setOptMode(const Setting_OptMode<Model>& optmode)
	{
		value_ = (value_ & ~((1 << 7) | (1 << 6) | (1 << 3) | 0b111));

		value_ |= ((static_cast<uint8_t>(optmode.long_range) & 0b1) << 7) |
				  ((static_cast<uint8_t>(optmode.shared_reg) & 0b1) << 6);

		if constexpr(is_sx1276_plus_v<Model>)
		{
			value_ |= (static_cast<uint8_t>(optmode.low_freq) & 0b1) << 3;
		}

		value_ |= (static_cast<uint8_t>(optmode.transceiver) & 0b111); // Bits [2:0]

		return value_;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type
		configureModem(const Setting_ModemConfig1<Model>& config)
	{
		// Clear only the bits being modified while preserving others
		if constexpr(is_sx1276_plus_v<Model>)
		{
			// SX1276+ models (4 fields)
			value_ &= ~((0b1111 << 4) | (0b111 << 1) | (0b1));

			value_ |= (static_cast<uint8_t>(config.bw) << 4) | // Bits [7:4]
					  (static_cast<uint8_t>(config.coding_rate) << 1) | // Bits [3:1]
					  (static_cast<uint8_t>(config.header_mode)); // Bit [0]
		}
		else if constexpr(Model == ChipModel::SX1272 || Model == ChipModel::SX1273)
		{
			// SX1272/73 models (6 fields)
			value_ &= ~((0b11 << 6) | (0b111 << 3) | (0b1 << 2) | (0b1 << 1) | (0b1));
			value_ |= (static_cast<uint8_t>(config.bw) << 6) | // Bits [7:6]
					  (static_cast<uint8_t>(config.coding_rate) << 3) | // Bits [5:3]
					  (static_cast<uint8_t>(config.header_mode) << 2) | // Bit [2]
					  (static_cast<uint8_t>(config.mode) << 1) | // Bit [1]
					  (static_cast<uint8_t>(config.ldro) & 0b1); // Bit [0]
		}

		return value_;
	}

	void reset()
	{
		value_ = getDefaultValue(reg, model);
	}
	uint8_t getValue() const
	{
		return value_;
	}
	void setValue(uint8_t v)
	{
		value_ = v;
	}
	void setField(uint8_t mask, uint8_t newValue)
	{
	}
	void updateBits(uint8_t mask, uint8_t newValue)
	{
		value_ = (value_ & ~mask) | (newValue & mask);
	}

	// Common helper that performs the shared functionality.

  private:
	uint8_t value_;

	static constexpr etl::array<RegInfo, REG_COUNT> regTable{
		{{REG::FIFO, 0x00, REG_MODE::READ_WRITE, 0x00},
		 {REG::OPMODE, 0x01, REG_MODE::READ_WRITE, 0x01, OptModeFieldConfigs,
		  sizeof(OptModeFieldConfigs) / sizeof(OptModeFieldConfigs[0])},
		 {REG::FRF_MSB, 0x06, REG_MODE::READ_WRITE, 0x6C},
		 {REG::FRF_MID, 0x07, REG_MODE::READ_WRITE, 0x80},
		 {REG::FRF_LSB, 0x08, REG_MODE::READ_WRITE, 0x00},
		 {REG::PAC, 0x09, REG_MODE::READ_WRITE, 0x00},
		 {REG::PARAMP, 0x0A, REG_MODE::READ_WRITE, 0x00},
		 {REG::OCP, 0x0B, REG_MODE::READ_WRITE, 0x00},
		 {REG::LNA, 0x0C, REG_MODE::READ_WRITE, 0x00},
		 {REG::FIFO_ADDR_PTR, 0x0D, REG_MODE::READ_WRITE, 0x00},
		 {REG::FIFO_TX_BASE_AD, 0x0E, REG_MODE::READ_WRITE, 0x00},
		 {REG::FIFO_RX_BASE_AD, 0x0F, REG_MODE::READ_WRITE, 0x00},
		 {REG::FIFO_RX_CURRENT_ADDR, 0x10, REG_MODE::READ_ONLY, 0x00},
		 {REG::IRQ_FLAGS_MASK, 0x11, REG_MODE::READ_WRITE, 0x00},
		 {REG::IRQ_FLAGS, 0x12, REG_MODE::SET_TO_CLEAR, 0x00},
		 {REG::RX_BYTES_NB, 0x13, REG_MODE::READ_ONLY, 0x00},
		 {REG::PKT_SNR_VALUE, 0x19, REG_MODE::READ_ONLY, 0x00},
		 {REG::PKT_RSSI, 0x1A, REG_MODE::READ_ONLY, 0x00},
		 {REG::HOP_CHANNEL, 0x1C, REG_MODE::READ_ONLY, 0x00},
		 {REG::MODEM_CONFIG1, 0x1D, REG_MODE::READ_WRITE, 0x00, modemConfig1FieldConfigs,
		  sizeof(modemConfig1FieldConfigs) / sizeof(modemConfig1FieldConfigs[0])},
		 {REG::MODEM_CONFIG2, 0x1E, REG_MODE::READ_WRITE, 0x00},
		 {REG::SYMB_TIMEOUT_LSB, 0x1F, REG_MODE::READ_WRITE, 0x00},
		 {REG::PREAMBLE_MSB, 0x20, REG_MODE::READ_WRITE, 0x00},
		 {REG::PREAMBLE_LSB, 0x21, REG_MODE::READ_WRITE, 0x00},
		 {REG::PAYLOAD_LENGTH, 0x22, REG_MODE::READ_WRITE, 0x00},
		 {REG::MAX_PAYLOAD_LENGTH, 0x23, REG_MODE::READ_WRITE, 0x00},
		 {REG::HOP_PERIOD, 0x24, REG_MODE::READ_WRITE, 0x00},
		 {REG::FIFO_RX_BYTE_ADDR_PTR, 0x25, REG_MODE::READ_WRITE, 0x00},
		 {REG::MODEM_CONFIG3, 0x26, REG_MODE::READ_WRITE, 0x00},
		 {REG::PPM_CORRECTION, 0x27, REG_MODE::READ_WRITE, 0x00},
		 {REG::FREQ_ERROR_MSB, 0x28, REG_MODE::READ_ONLY, 0x00},
		 {REG::FREQ_ERROR_MID, 0x29, REG_MODE::READ_ONLY, 0x00},
		 {REG::FREQ_ERROR_LSB, 0x2A, REG_MODE::READ_ONLY, 0x00},
		 {REG::RSSI_WIDEBAND, 0x2C, REG_MODE::READ_ONLY, 0x00},
		 {REG::DETECT_OPTIMIZE, 0x31, REG_MODE::READ_WRITE, 0x00},
		 {REG::INVERTIQ, 0x33, REG_MODE::READ_WRITE, 0x00},
		 {REG::DET_TRESH, 0x37, REG_MODE::READ_WRITE, 0x00},
		 {REG::SYNC_WORD, 0x39, REG_MODE::READ_WRITE, 0x00},
		 {REG::INVERTIQ2, 0x3B, REG_MODE::READ_WRITE, 0x00},
		 {REG::TEMP, 0x3C, REG_MODE::READ_ONLY, 0x00},
		 {REG::DIO_MAPPING_1, 0x40, REG_MODE::READ_WRITE, 0x00},
		 {REG::DIO_MAPPING_2, 0x41, REG_MODE::READ_WRITE, 0x00},
		 {REG::VERSION, 0x42, REG_MODE::READ_ONLY, 0x00},
		 {REG::PADAC, 0x4D, REG_MODE::READ_WRITE, 0x00}}};
	constexpr const RegInfo* lookupRegInfo(REG r)
	{
		for(const auto& info: regTable)
		{
			if(info.reg == r)
			{
				return &info;
			}
		}
		return nullptr;
	}

	template<typename FieldType>
	constexpr ConfigParams getFieldConfigParams(REG r, FieldType field, ChipSeries chipSeries)
	{
		const RegInfo* info = lookupRegInfo(r);
		if(info && info->fieldConfigs)
		{
			// Cast the void* to the correct type
			const FieldConfig<FieldType>* configs =
				static_cast<const FieldConfig<FieldType>*>(info->fieldConfigs);

			for(size_t i = 0; i < info->fieldConfigCount; ++i)
			{
				if(configs[i].field == field && configs[i].chip_series == chipSeries)
				{
					return configs[i].params;
				}
			}
		}
		// Return a default value if no match is found
		return {0, 0};
	}
};
template<ChipModel Model>
class LoRaRegisters
{
  public:
	constexpr LoRaRegisters() :
		registers{
			Register(Model, REG::FIFO),
			Register(Model, REG::OPMODE),
			Register(Model, REG::FRF_MSB),
			Register(Model, REG::FRF_MID),
			Register(Model, REG::FRF_LSB),
			Register(Model, REG::PAC),
			Register(Model, REG::PARAMP),
			Register(Model, REG::OCP),
			Register(Model, REG::LNA),
			Register(Model, REG::FIFO_ADDR_PTR),
			Register(Model, REG::FIFO_TX_BASE_AD),
			Register(Model, REG::FIFO_RX_BASE_AD),
			Register(Model, REG::FIFO_RX_CURRENT_ADDR),
			Register(Model, REG::IRQ_FLAGS_MASK),
			Register(Model, REG::IRQ_FLAGS),
			Register(Model, REG::RX_BYTES_NB),
			Register(Model, REG::PKT_SNR_VALUE),
			Register(Model, REG::PKT_RSSI),
			Register(Model, REG::HOP_CHANNEL),
			Register(Model, REG::MODEM_CONFIG1),
			Register(Model, REG::MODEM_CONFIG2),
			Register(Model, REG::SYMB_TIMEOUT_LSB),
			Register(Model, REG::PREAMBLE_MSB),
			Register(Model, REG::PREAMBLE_LSB),
			Register(Model, REG::PAYLOAD_LENGTH),
			Register(Model, REG::MAX_PAYLOAD_LENGTH),
			Register(Model, REG::HOP_PERIOD),
			Register(Model, REG::FIFO_RX_BYTE_ADDR_PTR),
			Register(Model, REG::MODEM_CONFIG3),
			Register(Model, REG::PPM_CORRECTION),
			Register(Model, REG::FREQ_ERROR_MSB),
			Register(Model, REG::FREQ_ERROR_MID),
			Register(Model, REG::FREQ_ERROR_LSB),
			Register(Model, REG::RSSI_WIDEBAND),
			Register(Model, REG::DETECT_OPTIMIZE),
			Register(Model, REG::INVERTIQ),
			Register(Model, REG::DET_TRESH),
			Register(Model, REG::SYNC_WORD),
			Register(Model, REG::INVERTIQ2),
			Register(Model, REG::TEMP),
			Register(Model, REG::DIO_MAPPING_1),
			Register(Model, REG::DIO_MAPPING_2),
			Register(Model, REG::VERSION),
			Register(Model, REG::PADAC),
		}
	{
	}

	etl::optional<Register> getRegister(REG reg) const
	{
		for(const auto& r: registers)
		{
			if(r.reg == reg)
			{
				return r;
			}
		}
		return etl::nullopt;
	}

  private:
	etl::array<Register, REG_COUNT> registers;
};

} // namespace LoRa

#endif // LORA_REGISTERS_HPP
