#include "LoRaModem.hpp"
namespace LoRa
{

template<ChipModel Model>
Result LoRaModem<Model>::verifyRegister(REG regType, uint8_t expectedValue)
{
	Result result;
	result.success = false;
	auto reg = registers_.getRegister(regType);
	if(!reg)
		return result;
	Core::Lock lock(mutex_, portMAX_DELAY);
	if(!lock.acquired())
	{
		LOG::ERROR(MODEM_TAG, "Failed to acquire lock for register update");
		return result;
	}

	result.value = spiBus_.readRegister(reg->address);
	result.success = (result.value == expectedValue);
	if(result.success)
	{
		result.value = expectedValue;
	}
	return result;
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
template<ChipModel Model>
Result LoRaModem<Model>::setFrequency(uint32_t freq)
{

	Result result;
	result.success = false;
	result.value = 0;
	auto reg = registers_.getRegister(REG::OPMODE);
	if(!reg)
		return result;
	setOptMode(optField::TransceiverModes, TransceiverModes::STANDBY, true);
	result = verifyRegister(REG::OPMODE, reg->getRegisterField(optField::TransceiverModes));
	if(!result.success)
		return result;

	uint64_t frf = (static_cast<uint64_t>(freq) << 19) / FXOC;
	uint32_t temp_bytes = static_cast<uint32_t>(frf); // Ensuring it's within 24-bit range

	// Write and verify MSB
	uint8_t msb_value = static_cast<uint8_t>((temp_bytes >> 16) & 0xFF);
	updateRegister(REG::FRF_MSB, msb_value, true);
	result = verifyRegister(REG::FRF_MSB, msb_value);
	if(!result.success)
		return result;

	// Write and verify MID
	uint8_t mid_value = static_cast<uint8_t>((temp_bytes >> 8) & 0xFF);
	updateRegister(REG::FRF_MID, mid_value, true);
	result = verifyRegister(REG::FRF_MID, mid_value);
	if(!result.success)
		return result;

	// Write and verify LSB
	uint8_t lsb_value = static_cast<uint8_t>((temp_bytes >> 0) & 0xFF);
	updateRegister(REG::FRF_LSB, lsb_value, true);
	result = verifyRegister(REG::FRF_LSB, lsb_value);
	if(!result.success)
		return result;

	result.success = true;
	return result;
}

template<ChipModel Model>
void LoRaModem<Model>::hop()
{
	// opmode<tMode>(tMode::STANDBY);
}

} // namespace LoRa