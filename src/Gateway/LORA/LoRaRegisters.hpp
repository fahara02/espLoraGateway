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
	PADAC,
	PADAC_SX1272,
	PADAC_SX1276
};

struct Register
{
	const ChipModel model;
	const REG reg;
	const REG_MODE mode;
	const uint8_t address;
	struct ConfigParams
	{
		uint8_t shift;
		uint8_t mask;
	};
	template<ChipModel Model>
	using Bandwidth = typename SignalBandWidth<Model>::Type;
	constexpr Register() :
		model(ChipModel::SX1276), reg(REG::FIFO), mode(REG_MODE::READ_WRITE), address(0), value(0)
	{
	}
	constexpr Register(ChipModel m, REG r) :
		model(m), reg(r), mode(getRegMode(r)), address(getRegAddress(r)),
		value(getDefaultValue(r, m))
	{
	}

	constexpr uint8_t getRegAddress(REG r)
	{
		switch(r)
		{
			case REG::FIFO:
				return 0x00;
			case REG::OPMODE:
				return 0x01;
			case REG::FRF_MSB:
				return 0x06;
			case REG::FRF_MID:
				return 0x07;
			case REG::FRF_LSB:
				return 0x08;
			case REG::PAC:
				return 0x09;
			case REG::PARAMP:
				return 0x0A;
			case REG::OCP:
				return 0x0B;
			case REG::LNA:
				return 0x0C;
			case REG::FIFO_ADDR_PTR:
				return 0x0D;
			case REG::FIFO_TX_BASE_AD:
				return 0x0E;
			case REG::FIFO_RX_BASE_AD:
				return 0x0F;
			case REG::FIFO_RX_CURRENT_ADDR:
				return 0x10;
			case REG::IRQ_FLAGS_MASK:
				return 0x11;
			case REG::IRQ_FLAGS:
				return 0x12;
			case REG::RX_BYTES_NB:
				return 0x13;
			case REG::PKT_SNR_VALUE:
				return 0x19;
			case REG::PKT_RSSI:
				return 0x1A;
			case REG::HOP_CHANNEL:
				return 0x1C;
			case REG::MODEM_CONFIG1:
				return 0x1D;
			case REG::MODEM_CONFIG2:
				return 0x1E;
			case REG::SYMB_TIMEOUT_LSB:
				return 0x1F;
			case REG::PREAMBLE_MSB:
				return 0x20;
			case REG::PREAMBLE_LSB:
				return 0x21;
			case REG::PAYLOAD_LENGTH:
				return 0x22;
			case REG::MAX_PAYLOAD_LENGTH:
				return 0x23;
			case REG::HOP_PERIOD:
				return 0x24;
			case REG::FIFO_RX_BYTE_ADDR_PTR:
				return 0x25;
			case REG::MODEM_CONFIG3:
				return 0x26;
			case REG::PPM_CORRECTION:
				return 0x27;
			case REG::FREQ_ERROR_MSB:
				return 0x28;
			case REG::FREQ_ERROR_MID:
				return 0x29;
			case REG::FREQ_ERROR_LSB:
				return 0x2A;
			case REG::RSSI_WIDEBAND:
				return 0x2C;
			case REG::DETECT_OPTIMIZE:
				return 0x31;
			case REG::INVERTIQ:
				return 0x33;
			case REG::DET_TRESH:
				return 0x37;
			case REG::SYNC_WORD:
				return 0x39;
			case REG::INVERTIQ2:
				return 0x3B;
			case REG::TEMP:
				return 0x3C;
			case REG::DIO_MAPPING_1:
				return 0x40;
			case REG::DIO_MAPPING_2:
				return 0x41;
			case REG::VERSION:
				return 0x42;
			case REG::PADAC:
				return 0x4D;
		}
		return 0xFF; // Invalid register (shouldn't happen)
	}
	constexpr uint8_t getDefaultValue(REG r, ChipModel m)
	{
		switch(r)
		{
			case REG::FRF_MSB:
				return 0x6C;
			case REG::FRF_MID:
				return 0x80;
			case REG::FRF_LSB:
				return 0x00;
			default:
				return 0x00;
		}
	}
	constexpr REG_MODE getRegMode(REG r)
	{
		switch(r)
		{
			case REG::FIFO:
				return REG_MODE::READ_WRITE;
			case REG::OPMODE:
				return REG_MODE::READ_WRITE;
			case REG::FRF_MSB:
				return REG_MODE::READ_WRITE;
			case REG::FRF_MID:
				return REG_MODE::READ_WRITE;
			case REG::FRF_LSB:
				return REG_MODE::READ_WRITE;
			case REG::PAC:
				return REG_MODE::READ_WRITE;
			case REG::PARAMP:
				return REG_MODE::READ_WRITE;
			case REG::OCP:
				return REG_MODE::READ_WRITE;
			case REG::LNA:
				return REG_MODE::READ_WRITE;
			case REG::FIFO_ADDR_PTR:
				return REG_MODE::READ_WRITE;
			case REG::FIFO_TX_BASE_AD:
				return REG_MODE::READ_WRITE;
			case REG::FIFO_RX_BASE_AD:
				return REG_MODE::READ_WRITE;
			case REG::FIFO_RX_CURRENT_ADDR:
				return REG_MODE::READ_ONLY;
			case REG::IRQ_FLAGS_MASK:
				return REG_MODE::READ_WRITE;
			case REG::IRQ_FLAGS:
				return REG_MODE::SET_TO_CLEAR;
			case REG::RX_BYTES_NB:
				return REG_MODE::READ_ONLY;
			case REG::PKT_SNR_VALUE:
				return REG_MODE::READ_ONLY;
			case REG::PKT_RSSI:
				return REG_MODE::READ_ONLY;
			case REG::HOP_CHANNEL:
				return REG_MODE::READ_ONLY;
			case REG::MODEM_CONFIG1:
				return REG_MODE::READ_WRITE;
			case REG::MODEM_CONFIG2:
				return REG_MODE::READ_WRITE;
			case REG::SYMB_TIMEOUT_LSB:
				return REG_MODE::READ_WRITE;
			case REG::PREAMBLE_MSB:
				return REG_MODE::READ_WRITE;
			case REG::PREAMBLE_LSB:
				return REG_MODE::READ_WRITE;
			case REG::PAYLOAD_LENGTH:
				return REG_MODE::READ_WRITE;
			case REG::MAX_PAYLOAD_LENGTH:
				return REG_MODE::READ_WRITE;
			case REG::HOP_PERIOD:
				return REG_MODE::READ_WRITE;
			case REG::FIFO_RX_BYTE_ADDR_PTR:
				return REG_MODE::READ_WRITE;
			case REG::MODEM_CONFIG3:
				return REG_MODE::READ_WRITE;
			case REG::PPM_CORRECTION:
				return REG_MODE::READ_WRITE;
			case REG::FREQ_ERROR_MSB:
				return REG_MODE::READ_ONLY;
			case REG::FREQ_ERROR_MID:
				return REG_MODE::READ_ONLY;
			case REG::FREQ_ERROR_LSB:
				return REG_MODE::READ_ONLY;
			case REG::RSSI_WIDEBAND:
				return REG_MODE::READ_ONLY;
			case REG::DETECT_OPTIMIZE:
				return REG_MODE::READ_WRITE;
			case REG::INVERTIQ:
				return REG_MODE::READ_WRITE;
			case REG::DET_TRESH:
				return REG_MODE::READ_WRITE;
			case REG::SYNC_WORD:
				return REG_MODE::READ_WRITE;
			case REG::INVERTIQ2:
				return REG_MODE::READ_WRITE;
			case REG::TEMP:
				return REG_MODE::READ_ONLY;
			case REG::DIO_MAPPING_1:
				return REG_MODE::READ_WRITE;
			case REG::DIO_MAPPING_2:
				return REG_MODE::READ_WRITE;
			case REG::VERSION:
				return REG_MODE::READ_ONLY;
			case REG::PADAC:
				return REG_MODE::READ_WRITE;
		}
		return REG_MODE::READ_WRITE; // Default case
	}
	// Set operating mode for OPMODE register
	// template<REG T, typename ModeType>
	// typename etl::enable_if<T == REG::OPMODE, uint8_t>::type setOptMode(const ModeType mode)
	// {
	// 	if constexpr(std::is_same_v<ModeType, LongRangeMode>)
	// 	{
	// 		value = (value & ~(1 << 7)) | ((static_cast<uint8_t>(mode) & 0b1) << 7);
	// 	}
	// 	else if constexpr(std::is_same_v<ModeType, TransceiverModes>)
	// 	{
	// 		value = (value & ~0b111) | (static_cast<uint8_t>(mode) & 0b111);
	// 	}
	// 	return value;
	// }
	template<REG T>
	typename etl::enable_if<T == REG::OPMODE, uint8_t>::type setOptMode(const LongRangeMode mode)
	{
		value = (value & ~(1 << 7)) | ((static_cast<uint8_t>(mode) & 0b1) << 7);
		return value;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<(T == REG::OPMODE && is_sx1276_plus_v<Model>), uint8_t>::type
		setOptMode(const LowFreqMode mode)
	{
		return value = (value & ~(1 << 3)) | (static_cast<uint8_t>(mode) & 0b1) << 3;
	}

	template<REG T>
	typename etl::enable_if<(T == REG::OPMODE), uint8_t>::type
		setOptMode(const TransceiverModes mode)
	{
		value = (value & ~0b111) | (static_cast<uint8_t>(mode) & 0b111);
		return value;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::OPMODE, uint8_t>::type
		setOptMode(const ConfigureOptMode<Model>& optmode)
	{
		value = (value & ~((1 << 7) | (1 << 6) | (1 << 3) | 0b111));

		value |= ((static_cast<uint8_t>(optmode.long_range) & 0b1) << 7) |
				 ((static_cast<uint8_t>(optmode.shared_reg) & 0b1) << 6);

		if constexpr(is_sx1276_plus_v<Model>)
		{
			value |= (static_cast<uint8_t>(optmode.low_freq) & 0b1) << 3;
		}

		value |= (static_cast<uint8_t>(optmode.transceiver) & 0b111); // Bits [2:0]

		return value;
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type
		setBandWidth(Bandwidth<Model>& bw)
	{
		ConfigParams params;

		if constexpr(is_sx1272_plus_v<Model>)
		{
			params = {6, 0b11000000}; // Shift = 6, Mask = 0b11000000
		}
		else if constexpr(is_sx1276_plus_v<Model>)
		{
			params = {4, 0b11110000}; // Shift = 4, Mask = 0b11110000
		}
		else
		{
			LOG::ERROR(LORA_REG, "Unsupported chip model for bandwidth setting");
			return 0;
		}

		return updateModemConfig<REG::MODEM_CONFIG1>(static_cast<uint8_t>(bw), params);
	}
	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type setCodingRate(CodingRate rate)
	{
		ConfigParams params;

		if constexpr(is_sx1272_plus_v<Model>)
		{
			params = {3, 0b00111000}; // Shift = 3, Mask = 0b00111000
		}
		else if constexpr(is_sx1276_plus_v<Model>)
		{
			params = {1, 0b00001110}; // Shift = 1, Mask = 0b00001110
		}
		else
		{
			LOG::ERROR(LORA_REG, "Unsupported chip model for bandwidth setting");
			return 0;
		}

		return updateModemConfig<REG::MODEM_CONFIG1>(static_cast<uint8_t>(rate), params);
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type setCRC(CRCMode mode)
	{
		Register::ConfigParams params;

		if constexpr(is_sx1272_plus_v<Model>)
		{
			params = {1, 0b00000010}; // Example values, adjust as needed
		}

		else
		{
			LOG::ERROR(LORA_REG, "Unsupported chip model for CRC setting");
			return 0;
		}

		return updateModemConfig<REG::MODEM_CONFIG1>(static_cast<uint8_t>(mode), params);
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type
		setLowDataOptimization(LowDataRateOptimize mode)
	{
		Register::ConfigParams params;

		if constexpr(is_sx1272_plus_v<Model>)
		{
			params = {0, 0b00000001}; // Example values, adjust as needed
		}

		else
		{
			LOG::ERROR(LORA_REG, "Unsupported chip model for low data optimization");
			return 0;
		}

		return updateModemConfig<REG::MODEM_CONFIG1>(static_cast<uint8_t>(mode), params);
	}

	template<REG T, ChipModel Model>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type
		configureModem(const ModemConfig1<Model>& config)
	{
		// Clear only the bits being modified while preserving others
		if constexpr(is_sx1276_plus_v<Model>)
		{
			// SX1276+ models (4 fields)
			value &= ~((0b1111 << 4) | (0b111 << 1) | (0b1));

			value |= (static_cast<uint8_t>(config.bw) << 4) | // Bits [7:4]
					 (static_cast<uint8_t>(config.coding_rate) << 1) | // Bits [3:1]
					 (static_cast<uint8_t>(config.header_mode)); // Bit [0]
		}
		else if constexpr(Model == ChipModel::SX1272 || Model == ChipModel::SX1273)
		{
			// SX1272/73 models (6 fields)
			value &= ~((0b11 << 6) | (0b111 << 3) | (0b1 << 2) | (0b1 << 1) | (0b1));
			value |= (static_cast<uint8_t>(config.bw) << 6) | // Bits [7:6]
					 (static_cast<uint8_t>(config.coding_rate) << 3) | // Bits [5:3]
					 (static_cast<uint8_t>(config.header_mode) << 2) | // Bit [2]
					 (static_cast<uint8_t>(config.mode) << 1) | // Bit [1]
					 (static_cast<uint8_t>(config.ldro) & 0b1); // Bit [0]
		}

		return value;
	}

	void reset()
	{
		value = getDefaultValue(reg, model);
	}
	uint8_t getValue() const
	{
		return value;
	}
	void setValue(uint8_t v)
	{
		value = v;
	}
	void setField(uint8_t mask, uint8_t newValue)
	{
	}
	void updateBits(uint8_t mask, uint8_t newValue)
	{
		value = (value & ~mask) | (newValue & mask);
	}

  private:
	uint8_t value;
	template<REG T>
	typename etl::enable_if<T == REG::MODEM_CONFIG1, uint8_t>::type
		updateModemConfig(uint8_t value, const ConfigParams& params)
	{
		uint8_t final_value = value << params.shift;
		updateBits(params.mask, final_value);
		return final_value;
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
