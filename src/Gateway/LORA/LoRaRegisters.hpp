#ifndef LORA_REGISTERS_HPP
#define LORA_REGISTERS_HPP

#include "sm127XX.hpp"
#include "etl/array.h"
#include "etl/optional.h"

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
	const REG reg;
	const REG_MODE mode;
	const uint8_t address;
	constexpr Register() : reg(REG::FIFO), mode(REG_MODE::READ_WRITE), address(0), value(0)
	{
	}
	constexpr Register(REG r, REG_MODE m, uint8_t addr, uint8_t v) :
		reg(r), mode(m), address(addr), value(v)
	{
	}

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
		value = 0;
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
};

class LoRaRegisters
{
  public:
	constexpr LoRaRegisters() :
		registers{Register(REG::FIFO, REG_MODE::READ_WRITE, 0x00, 0x00),
				  Register(REG::OPMODE, REG_MODE::READ_WRITE, 0x01, 0x00),
				  Register(REG::FRF_MSB, REG_MODE::READ_WRITE, 0x06, 0x6C),
				  Register(REG::FRF_MID, REG_MODE::READ_WRITE, 0x07, 0x80),
				  Register(REG::FRF_LSB, REG_MODE::READ_WRITE, 0x08, 0x00),
				  Register(REG::PAC, REG_MODE::READ_WRITE, 0x09, 0x00),
				  Register(REG::PARAMP, REG_MODE::READ_WRITE, 0x0A, 0x00),
				  Register(REG::OCP, REG_MODE::READ_WRITE, 0x0B, 0x00),
				  Register(REG::LNA, REG_MODE::READ_WRITE, 0x0C, 0x00),
				  Register(REG::FIFO_ADDR_PTR, REG_MODE::READ_WRITE, 0x0D, 0x00),
				  Register(REG::FIFO_TX_BASE_AD, REG_MODE::READ_WRITE, 0x0E, 0x00),
				  Register(REG::FIFO_RX_BASE_AD, REG_MODE::READ_WRITE, 0x0F, 0x00),
				  Register(REG::FIFO_RX_CURRENT_ADDR, REG_MODE::READ_ONLY, 0x10, 0x00),
				  Register(REG::IRQ_FLAGS_MASK, REG_MODE::READ_WRITE, 0x11, 0x00),
				  Register(REG::IRQ_FLAGS, REG_MODE::SET_TO_CLEAR, 0x12, 0x00),
				  Register(REG::RX_BYTES_NB, REG_MODE::READ_ONLY, 0x13, 0x00),
				  Register(REG::PKT_SNR_VALUE, REG_MODE::READ_ONLY, 0x19, 0x00),
				  Register(REG::PKT_RSSI, REG_MODE::READ_ONLY, 0x1A, 0x00),
				  Register(REG::HOP_CHANNEL, REG_MODE::READ_ONLY, 0x1C, 0x00),
				  Register(REG::MODEM_CONFIG1, REG_MODE::READ_WRITE, 0x1D, 0x00),
				  Register(REG::MODEM_CONFIG2, REG_MODE::READ_WRITE, 0x1E, 0x00),
				  Register(REG::SYMB_TIMEOUT_LSB, REG_MODE::READ_WRITE, 0x1F, 0x00),
				  Register(REG::PREAMBLE_MSB, REG_MODE::READ_WRITE, 0x20, 0x00),
				  Register(REG::PREAMBLE_LSB, REG_MODE::READ_WRITE, 0x21, 0x00),
				  Register(REG::PAYLOAD_LENGTH, REG_MODE::READ_WRITE, 0x22, 0x00),
				  Register(REG::MAX_PAYLOAD_LENGTH, REG_MODE::READ_WRITE, 0x23, 0x00),
				  Register(REG::HOP_PERIOD, REG_MODE::READ_WRITE, 0x24, 0x00),
				  Register(REG::FIFO_RX_BYTE_ADDR_PTR, REG_MODE::READ_WRITE, 0x25, 0x00),
				  Register(REG::MODEM_CONFIG3, REG_MODE::READ_WRITE, 0x26, 0x00),
				  Register(REG::PPM_CORRECTION, REG_MODE::READ_WRITE, 0x27, 0x00),
				  Register(REG::FREQ_ERROR_MSB, REG_MODE::READ_ONLY, 0x28, 0x00),
				  Register(REG::FREQ_ERROR_MID, REG_MODE::READ_ONLY, 0x29, 0x00),
				  Register(REG::FREQ_ERROR_LSB, REG_MODE::READ_ONLY, 0x2A, 0x00),
				  Register(REG::RSSI_WIDEBAND, REG_MODE::READ_ONLY, 0x2C, 0x00),
				  Register(REG::DETECT_OPTIMIZE, REG_MODE::READ_WRITE, 0x31, 0x00),
				  Register(REG::INVERTIQ, REG_MODE::READ_WRITE, 0x33, 0x00),
				  Register(REG::DET_TRESH, REG_MODE::READ_WRITE, 0x37, 0x00),
				  Register(REG::SYNC_WORD, REG_MODE::READ_WRITE, 0x39, 0x00),
				  Register(REG::INVERTIQ2, REG_MODE::READ_WRITE, 0x3B, 0x00),
				  Register(REG::TEMP, REG_MODE::READ_ONLY, 0x3C, 0x00),
				  Register(REG::DIO_MAPPING_1, REG_MODE::READ_WRITE, 0x40, 0x00),
				  Register(REG::DIO_MAPPING_2, REG_MODE::READ_WRITE, 0x41, 0x00),
				  Register(REG::VERSION, REG_MODE::READ_ONLY, 0x42, 0x00),
				  Register(REG::PADAC, REG_MODE::READ_WRITE, 0x4D, 0x00)}
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
	etl::array<Register, 45> registers;
};

} // namespace LoRa

#endif // LORA_REGISTERS_HPP
