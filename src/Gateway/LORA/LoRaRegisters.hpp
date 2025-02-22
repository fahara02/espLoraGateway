#ifndef LORA_REGISTERS_HPP
#define LORA_REGISTERS_HPP

#include "sm127XX.hpp"
#include "etl/array.h"
#include "etl/optional.h"

#include "Logger.hpp"
#define LORA_REG "LORA_REGISTER"

namespace LoRa
{

struct RegInfo
{
	REG reg;
	uint8_t address;
	MODE mode; // generalised Mode register level
	uint8_t defaultValue;
	const void* fieldConfigs;
	size_t fieldConfigCount;
};

struct Register
{
	const ChipModel model;
	const REG reg;
	const MODE mode;
	const uint8_t address;

	template<ChipModel Model>
	using Bandwidth = typename SignalBandWidth<Model>::Type;
	using OptModeField = typename SettingBase<optField>::FieldsConfig;
	using ModeConfig1Field = typename SettingBase<config1Field>::FieldsConfig;

	constexpr Register() :
		model(ChipModel::SX1276), reg(REG::FIFO), mode(MODE::RW), address(0), value_(0)
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
	constexpr MODE getRegMode(REG r)
	{
		const RegInfo* info = lookupRegInfo(r);
		return info ? info->mode : MODE::ANY;
	}

	template<typename Field>
	uint8_t updateRegisterField(const Field field, uint8_t new_value)
	{
		const Series series = isSx1272Plus(model) ? Series::SM72 : Series::SM76;

		ConfigParams params = getFieldConfigParams(reg, field, series);

		uint8_t shifted_value = new_value << params.shift;
		updateBits(params.mask, shifted_value);

		return value_;
	}

	template<typename ValueType>

	uint8_t updateOptMode(const optField field, ValueType new_value)
	{
		return updateRegisterField<optField>(field, static_cast<uint8_t>(new_value));
	}

	template<typename ValueType>
	uint8_t updateModemConfig(const config1Field field, ValueType new_value)
	{
		return updateRegisterField<config1Field>(field, static_cast<uint8_t>(new_value));
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::OPMODE, uint8_t>::type
		setOptMode(const optModeSetting<Model>& optmode)
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
		setModemConfig1(const ModemConfig1Setting<Model>& config)
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

	void updateBits(uint8_t mask, uint8_t newValue)
	{
		value_ = (value_ & ~mask) | (newValue & mask);
	}

  private:
	uint8_t value_;

	static constexpr etl::array<OptModeField, 7> optModeFields = {
		OptModeField{optField::LongRangeMode, MODE::RW, Series::SM72, {7, 0b10000000}, 0},
		OptModeField{optField::LongRangeMode, MODE::RW, Series::SM76, {7, 0b10000000}, 0},
		OptModeField{optField::AccessSharedReg, MODE::RW, Series::SM72, {6, 0b01000000}, 0},
		OptModeField{optField::AccessSharedReg, MODE::RW, Series::SM76, {6, 0b01000000}, 0},
		// Only for SM76
		OptModeField{optField::LowFreqMode, MODE::RW, Series::SM76, {3, 0b00001000}, 0x1},
		OptModeField{optField::TransceiverModes, MODE::RWT, Series::SM72, {0, 0b00000111}, 0x1},
		OptModeField{optField::TransceiverModes, MODE::RWT, Series::SM76, {0, 0b00000111}, 0x1},
	};

	static constexpr etl::array<ModeConfig1Field, 8> modemConfig1Fields = {
		// Bandwidth:
		ModeConfig1Field{config1Field::Bandwidth, MODE::RW, Series::SM72, {6, 0b11000000}, 0x0},
		ModeConfig1Field{config1Field::Bandwidth, MODE::RW, Series::SM76, {4, 0b11110000}, 0x0},
		// Coding Rate:
		ModeConfig1Field{config1Field::CodingRate, MODE::RW, Series::SM72, {3, 0b00111000}, 0x0},
		{config1Field::CodingRate, MODE::RW, Series::SM76, {1, 0b00001110}, 0x0},
		// HeaderMode
		ModeConfig1Field{config1Field::HeaderMode, MODE::RW, Series::SM72, {2, 0b00000100}, 0x0},
		ModeConfig1Field{config1Field::HeaderMode, MODE::RW, Series::SM76, {0, 0b00000001}, 0x0},
		// CRC (only defined for Sm72 series):
		ModeConfig1Field{config1Field::CRC, MODE::RW, Series::SM72, {1, 0b00000010}, 0x0},
		// Low Data Optimization (only defined for Sm72 series):
		ModeConfig1Field{
			config1Field::LowDataOptimization, MODE::RW, Series::SM72, {0, 0b00000001}, 0x0}};

	static constexpr etl::array<RegInfo, REG_COUNT> regTable = {
		{{REG::FIFO, 0x00, MODE::RW, 0x00},
		 {REG::OPMODE, 0x01, MODE::RW, 0x09, static_cast<const void*>(optModeFields.data()),
		  optModeFields.size()},
		 {REG::FRF_MSB, 0x06, MODE::RW, 0x6C},
		 {REG::FRF_MID, 0x07, MODE::RW, 0x80},
		 {REG::FRF_LSB, 0x08, MODE::RW, 0x00},
		 {REG::PAC, 0x09, MODE::RW, 0x00},
		 {REG::PARAMP, 0x0A, MODE::RW, 0x00},
		 {REG::OCP, 0x0B, MODE::RW, 0x00},
		 {REG::LNA, 0x0C, MODE::RW, 0x00},
		 {REG::FIFO_ADDR_PTR, 0x0D, MODE::RW, 0x00},
		 {REG::FIFO_TX_BASE_AD, 0x0E, MODE::RW, 0x00},
		 {REG::FIFO_RX_BASE_AD, 0x0F, MODE::RW, 0x00},
		 {REG::FIFO_RX_CURRENT_ADDR, 0x10, MODE::R, 0x00},
		 {REG::IRQ_FLAGS_MASK, 0x11, MODE::RW, 0x00},
		 {REG::IRQ_FLAGS, 0x12, MODE::SC, 0x00},
		 {REG::RX_BYTES_NB, 0x13, MODE::R, 0x00},
		 {REG::PKT_SNR_VALUE, 0x19, MODE::R, 0x00},
		 {REG::PKT_RSSI, 0x1A, MODE::R, 0x00},
		 {REG::HOP_CHANNEL, 0x1C, MODE::R, 0x00},
		 {REG::MODEM_CONFIG1, 0x1D, MODE::RW, 0x00,
		  static_cast<const void*>(modemConfig1Fields.data()), modemConfig1Fields.size()},
		 {REG::MODEM_CONFIG2, 0x1E, MODE::RW, 0x00},
		 {REG::SYMB_TIMEOUT_LSB, 0x1F, MODE::R, 0x00},
		 {REG::PREAMBLE_MSB, 0x20, MODE::RW, 0x00},
		 {REG::PREAMBLE_LSB, 0x21, MODE::RW, 0x00},
		 {REG::PAYLOAD_LENGTH, 0x22, MODE::RW, 0x00},
		 {REG::MAX_PAYLOAD_LENGTH, 0x23, MODE::RW, 0x00},
		 {REG::HOP_PERIOD, 0x24, MODE::RW, 0x00},
		 {REG::FIFO_RX_BYTE_ADDR_PTR, 0x25, MODE::RW, 0x00},
		 {REG::MODEM_CONFIG3, 0x26, MODE::RW, 0x00},
		 {REG::PPM_CORRECTION, 0x27, MODE::RW, 0x00},
		 {REG::FREQ_ERROR_MSB, 0x28, MODE::R, 0x00},
		 {REG::FREQ_ERROR_MID, 0x29, MODE::R, 0x00},
		 {REG::FREQ_ERROR_LSB, 0x2A, MODE::R, 0x00},
		 {REG::RSSI_WIDEBAND, 0x2C, MODE::R, 0x00},
		 {REG::DETECT_OPTIMIZE, 0x31, MODE::RW, 0x00},
		 {REG::INVERTIQ, 0x33, MODE::RW, 0x00},
		 {REG::DET_TRESH, 0x37, MODE::RW, 0x00},
		 {REG::SYNC_WORD, 0x39, MODE::RW, 0x00},
		 {REG::INVERTIQ2, 0x3B, MODE::RW, 0x00},
		 {REG::TEMP, 0x3C, MODE::R, 0x00},
		 {REG::DIO_MAPPING_1, 0x40, MODE::RW, 0x00},
		 {REG::DIO_MAPPING_2, 0x41, MODE::RW, 0x00},
		 {REG::VERSION, 0x42, MODE::R, 0x00},
		 {REG::PADAC, 0x4D, MODE::RW, 0x00}}};
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
	constexpr ConfigParams getFieldConfigParams(REG r, FieldType field, Series chipSeries)
	{
		const RegInfo* info = lookupRegInfo(r);
		if(info && info->fieldConfigs)
		{
			// Cast the void* to the correct type
			using FieldConfig = SettingBase<FieldType>::FieldsConfig;
			const FieldConfig* configs = static_cast<const FieldConfig*>(info->fieldConfigs);

			for(size_t i = 0; i < info->fieldConfigCount; ++i)
			{
				if(configs[i].fieldEnum == field && configs[i].series == chipSeries)
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
	static LoRaRegisters& getInstance()
	{
		static LoRaRegisters instance;
		return instance;
	}

	Register* getRegister(REG reg)
	{
		for(auto& r: registers)
		{
			if(r.reg == reg)
			{
				return &r;
			}
		}
		return nullptr;
	}

  private:
	// Private constructor to enforce Singleton pattern
	LoRaRegisters() :
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

	// Prevent copying and moving
	LoRaRegisters(const LoRaRegisters&) = delete;
	LoRaRegisters& operator=(const LoRaRegisters&) = delete;

  private:
	etl::array<Register, REG_COUNT> registers;
};

} // namespace LoRa

#endif // LORA_REGISTERS_HPP
