#include "LoRaModem.hpp"
namespace LoRa
{

// template<ChipModel Model>
// uint8_t LoRaModem<Model>::setBandWidth(typename LoRaModem<Model>::Bandwidth& bw, bool sendSPI)
// {
// 	auto reg = registers_.getRegister(REG::MODEM_CONFIG1);
// 	uint8_t bw_value = 0;
// 	uint8_t mask = 0;

// 	if constexpr(is_sx1272_plus_v<Model>) // Check if model is SX1272+
// 	{
// 		bw_value = static_cast<uint8_t>(bw) << 6;
// 		mask = 0b11000000; // Mask for bits 7-6
// 	}
// 	else if constexpr(is_sx1276_plus_v<Model>) // Check if model is SX1276+
// 	{
// 		bw_value = static_cast<uint8_t>(bw) << 4;
// 		mask = 0b11110000; // Mask for bits 7-4
// 	}
// 	else
// 	{
// 		LOG::ERROR(MODEM_TAG, "Unsupported chip model for bandwidth setting");
// 		return 0;
// 	}

// 	if(sendSPI)
// 	{
// 		spiBus_.writeRegister(reg->address, bw_value);
// 	}

// 	reg->updateBits(mask, bw_value);
// 	return bw_value;
// }

template<ChipModel Model>
void LoRaModem<Model>::setBitRate(uint8_t sf, uint8_t crc)
{
}
template<ChipModel Model>
void LoRaModem<Model>::setFrequency(uint32_t freq)
{
	opmode<tMode>(tMode::STANDBY);

	auto fMSB = registers_.getRegister(REG::FRF_MSB);
	auto fMID = registers_.getRegister(REG::FRF_MID);
	auto fLSB = registers_.getRegister(REG::FRF_LSB);

	if(!fMSB || !fMID || !fLSB)
	{
		return;
	}

	/*
	 * LoRa Frequency Calculation:
	 *
	 * Formula from datasheet:
	 *      Frf = (f_RF * 2^19) / F_XOSC
	 *
	 * Where:
	 *      f_RF   -> Desired frequency in Hz
	 *      F_XOSC -> Crystal oscillator frequency (typically 32 MHz)
	 *      Frf    -> 24-bit register value to be written
	 *
	 * Implemented as:
	 */
	uint64_t frf = (static_cast<uint64_t>(freq) << 19) / FXOC;
	uint32_t temp_bytes = static_cast<uint32_t>(frf); // Ensuring it's within 24-bit range

	spiBus_.writeRegister(fMSB->address, static_cast<uint8_t>((temp_bytes >> 16) & 0xFF)); // MSB
	spiBus_.writeRegister(fMID->address, static_cast<uint8_t>((temp_bytes >> 8) & 0xFF)); // MID
	spiBus_.writeRegister(fLSB->address, static_cast<uint8_t>((temp_bytes >> 0) & 0xFF)); // LSB
}
template<ChipModel Model>
template<typename Mode>
void LoRaModem<Model>::opmode(const Mode& mode)
{
	auto reg = registers_.getRegister(REG::OPMODE);
	uint8_t setValue = reg->setOptMode<REG::OPMODE>(mode);
	spiBus_.writeRegister(reg->address, setValue & 0xFF);
}
template<ChipModel Model>
void LoRaModem<Model>::hop()
{
	opmode<tMode>(tMode::STANDBY);
}

} // namespace LoRa