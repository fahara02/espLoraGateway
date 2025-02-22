#include "LoRaModem.hpp"
namespace LoRa
{

template<ChipModel Model>
void LoRaModem<Model>::setFrequency(uint32_t freq)
{
	setOptMode(Field_OptMode::TransceiverModes, TransceiverModes::STANDBY);

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
void LoRaModem<Model>::hop()
{
	// opmode<tMode>(tMode::STANDBY);
}

} // namespace LoRa