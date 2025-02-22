
#ifndef LORA_MODEM_HPP
	#define LORA_MODEM_HPP

	#include "LoRaDefines.hpp"
	#include "LoRaBoards.hpp"
	#include "LoRaStat.hpp"
	#include "SPIBus.hpp"
	#include "LoRaRegisters.hpp"
	#include "Logger.hpp"
	#define MODEM_TAG "LORA-MODEM"
namespace LoRa
{

template<ChipModel Model>
class LoRaModem
{
  public:
	using rMode = LongRangeMode;
	using tMode = TransceiverModes;
	using Bandwidth = typename SignalBandWidth<Model>::Type;

	LoRaModem(BoardModel model, uint8_t address, LoRaBands band, const uint8_t bandIndex) :
		model_(model), address_(address), spiBus_(Core::SPIBus::getInstance(address_)),
		frequencytable_(getLoRaFrequencies(band)),
		frequencyPlan_((frequencytable_ && bandIndex < LORA_MAX_CHANNEL) ?
						   (*frequencytable_)[bandIndex] :
						   FrequencyPlan{}),
		pins_(LoRaBoard::getPinConfig(model_)), registers_(LoRaRegisters<Model>::getInstance())
	{
		init();
	}

	void startReceiver();

	// uint8_t setBandWidth(Bandwidth& bw, bool sendSPI)
	// {
	// 	return updateModemConfig1(static_cast<uint8_t>(bw), Field_ModemConfig1::Bandwidth, sendSPI);
	// }

	void setFrequency(uint32_t freq);
	void setPow(uint8_t pow);

	void hop();
	uint8_t receivePkt(uint8_t* payload);
	bool sendPkt(uint8_t* payLoad, uint8_t payLength);
	int loraWait(struct LoraDown* LoraDown);

	template<typename ValueType>
	uint8_t setOptMode(const optField field, const ValueType new_value, bool sendSPI = false)
	{
		if(field == optField::LongRangeMode)
		{
			// Ensure we have a valid software register reference
			auto reg = registers_.getRegister(REG::OPMODE);
			if(!reg)
			{
				LOG::ERROR(MODEM_TAG, "OPMODE register not found in software registers.");
				return 0;
			}

			uint8_t transceiverMode = reg->getRegisterField(optField::TransceiverModes);
			if(transceiverMode != static_cast<uint8_t>(TransceiverModes::SLEEP))
			{
				LOG::ERROR(MODEM_TAG,
						   "Cannot change LongRangeMode: TransceiverMode is not in SLEEP (Current: "
						   "0x%02X)",
						   transceiverMode);
				return 0;
			}
		}

		return updateRegister(REG::OPMODE, field, new_value, sendSPI);
	}

	// template<typename ValueType>
	// uint8_t setOptMode(optField field, ValueType value, bool sendSPI = false)
	// {
	// 	return updateRegister(REG::OPMODE, field, value, sendSPI);
	// }

	template<typename ValueType>
	uint8_t setModemConfig1(config1Field field, ValueType value, bool sendSPI = false)
	{
		return updateRegister(REG::MODEM_CONFIG1, field, value, sendSPI);
	}
	int getRegValue(REG r) const
	{
		auto reg = registers_.getRegister(r);
		return reg ? reg->getValue() : -1;
	}
	const ChipModel getChipModel() const
	{
		return chipModel_;
	}

  private:
	BoardModel model_;
	ChipModel chipModel_ = Model;
	uint8_t address_;
	Core::SPIBus& spiBus_;
	const LoRaFreqTable* frequencytable_;
	const FrequencyPlan frequencyPlan_;
	LoRaPins pins_;
	LoRaRegisters<Model>& registers_;

	volatile ReceiverState rxState_ = ReceiverState::S_INIT;
	volatile uint8_t event_ = 0;
	uint8_t rssi_;
	uint32_t msgTime_ = 0; // in seconds, Thru nowSeconds, now()
	uint32_t hopTime_ = 0; // in micros()
	etl::array<uint8_t, MAX_LORA_PAYLOAD> payload_;
	StatisticsLevel level_{StatisticsLevel::NONE};
	ModemStats stats_;
	bool initialised_ = false;

	struct LoraDown
	{
		uint32_t tmst; // Timestamp (will ignore time), time to output
		uint32_t tmms; // Timestamp according to GPS (sync required)
		uint32_t time;
		uint32_t freq; // Frequency
		uint32_t usec; // Store the value of microseconds for later printing
		uint16_t fcnt; // Framecount of the requesting LoraUp message
		uint8_t size;
		uint8_t chan; // = <NOT USED>
		uint8_t sf; // through datr
		uint8_t bw; // through datr
		bool ipol;
		uint8_t powe; // transmit power == 14, except when using special channel
		uint8_t crc;
		uint8_t iiq; // message inverted or not for node-node communiction
		uint8_t imme; // Immediate transfer execution
		uint8_t ncrc; // no CRC check
		uint8_t prea; // preamble
		uint8_t rfch; // Antenna "RF chain" used for TX (unsigned integer)
		char* modu; //	"LORA" os "FSCK"
		char* datr; // = "SF12BW125", contains both .sf and .bw parts
		char* codr;

		uint8_t* payLoad;
	} LoraDown;

	// Up buffer (from Lora sensor to UDP)
	// This struct contains all data of the buffer received from devices to gateway

	struct LoraUp
	{
		uint32_t tmst; // Timestamp of message
		uint32_t tmms; // <not used at the moment>
		uint32_t time; // <not used at the moment>
		uint32_t freq; // frequency used in HZ
		uint8_t size; // Length of the message Payload
		uint8_t chan; // Channel "IF" used for RX
		uint8_t sf; // Spreading Factor
		uint16_t fcnt;
		int32_t snr;
		int16_t prssi;
		int16_t rssicorr;
		char* modu; //	"LORA" or "FSCK"
		etl::array<uint8_t, MAX_LORA_PAYLOAD> payload;
	} LoraUp;

	void initLoraModem();
	void initDown(struct LoraDown* LoraDown);
	void txLoraModem(struct LoraDown* LoraDown);
	void rxLoraModem();
	void cadScanner();
	void init()

	{
		if(!initialised_)
		{
			stats_.getStat(StatisticsLevel::MESSAGE_STAT_LOCAL_SERVER)->reset();
			stats_.getStat(StatisticsLevel::GATEWAY_STAT_CHANNEL)->reset();
			initialised_ = true;
		}
	}
	const LoRaFreqTable* getLoRaFrequencies(LoRaBands band)
	{
		auto it = LoRaFrequencies.find(band);
		return (it != LoRaFrequencies.end()) ? &(it->second) : nullptr;
	}

	template<typename RegisterField, typename ValueType>
	uint8_t updateRegister(REG regType, RegisterField field, ValueType value, bool sendSPI)
	{
		auto reg = registers_.getRegister(regType);
		if(!reg)
			return 0; // Handle case where register is not found

		reg->template updateRegisterField(field, static_cast<uint8_t>(value));

		uint8_t updatedValue = reg->getValue(); // Retrieve the modified register value

		if(sendSPI)
		{
			spiBus_.writeRegister(reg->address, updatedValue);
		}
		return updatedValue; // Return the actual modified register value
	}
};

} // namespace LoRa

#endif // LORA_MODEM_HPP

// void ICACHE_RAM_ATTR Interrupt_0();
// void ICACHE_RAM_ATTR Interrupt_1();
// void ICACHE_RAM_ATTR Interrupt_2();