#include "LoRaModem.hpp"
namespace LoRa
{

template<ChipModel Model>
uint8_t LoRaModem<Model>::setCodingRate(CodingRate rate, bool sendSPI)
{
	Register::ConfigParams params;

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
		LOG::ERROR(MODEM_TAG, "Unsupported chip model for coding rate setting");
		return 0;
	}

	return updateModemConfig(REG::MODEM_CONFIG1, static_cast<uint8_t>(rate), params, sendSPI);
}

template<ChipModel Model>
uint8_t LoRaModem<Model>::setImplicitHeader(HeaderMode mode, bool sendSPI)
{
	Register::ConfigParams params;

	if constexpr(is_sx1272_plus_v<Model>)
	{
		params = {2, 0b00000100}; // Example values, adjust as needed
	}
	else if constexpr(is_sx1276_plus_v<Model>)
	{
		params = {2, 0b00000100}; // Example values, adjust as needed
	}
	else
	{
		LOG::ERROR(MODEM_TAG, "Unsupported chip model for implicit header setting");
		return 0;
	}

	return updateModemConfig(REG::MODEM_CONFIG1, static_cast<uint8_t>(mode), params, sendSPI);
}

template<ChipModel Model>
uint8_t LoRaModem<Model>::setCRC(CRCMode mode, bool sendSPI)
{
	Register::ConfigParams params;

	if constexpr(is_sx1272_plus_v<Model>)
	{
		params = {1, 0b00000010}; // Example values, adjust as needed
	}
	else if constexpr(is_sx1276_plus_v<Model>)
	{
		params = {1, 0b00000010}; // Example values, adjust as needed
	}
	else
	{
		LOG::ERROR(MODEM_TAG, "Unsupported chip model for CRC setting");
		return 0;
	}

	return updateModemConfig(REG::MODEM_CONFIG1, static_cast<uint8_t>(mode), params, sendSPI);
}

template<ChipModel Model>
uint8_t LoRaModem<Model>::setLowDataOptimization(LowDataRateOptimize mode, bool sendSPI)
{
	Register::ConfigParams params;

	if constexpr(is_sx1272_plus_v<Model>)
	{
		params = {0, 0b00000001}; // Example values, adjust as needed
	}
	else if constexpr(is_sx1276_plus_v<Model>)
	{
		params = {0, 0b00000001}; // Example values, adjust as needed
	}
	else
	{
		LOG::ERROR(MODEM_TAG, "Unsupported chip model for low data optimization");
		return 0;
	}

	return updateModemConfig(REG::MODEM_CONFIG3, static_cast<uint8_t>(mode), params, sendSPI);
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