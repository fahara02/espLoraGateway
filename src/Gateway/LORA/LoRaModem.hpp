
#ifndef LORA_MODEM_HPP
#define LORA_MODEM_HPP

#include "LoRaDefines.hpp"
#include "LoRaBoards.hpp"
#include "LoRaStat.hpp"
#include "SPIBus.hpp"
#include "LoRaRegisters.hpp"

namespace LoRa
{
class LoRaModem
{
  public:
	using rMode = LoRaChip::LongRangeMode;
	using tMode = LoRaChip::TransceiverModes;
	LoRaModem(BoardModel model, uint8_t address, LoRaBands band, const uint8_t bandIndex);

	void startReceiver();
	void setBandWidth(uint8_t sf, uint8_t crc);
	void setBitRate(uint8_t sf, uint8_t crc);
	void setFrequency(uint32_t freq);
	void setPow(uint8_t pow);
	template<typename Mode>
	void opmode(const Mode& mode);

	void hop();
	uint8_t receivePkt(uint8_t* payload);
	bool sendPkt(uint8_t* payLoad, uint8_t payLength);
	int loraWait(struct LoraDown* LoraDown);

  private:
	BoardModel model_;
	uint8_t address_;
	Core::SPIBus& spiBus_;
	const LoRaFreqTable* frequencytable_;
	const FrequencyPlan frequencyPlan_;
	LoRaPins pins_;
	LoRaRegisters registers_;

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

	void init();
	void initLoraModem();
	void initDown(struct LoraDown* LoraDown);
	void txLoraModem(struct LoraDown* LoraDown);
	void rxLoraModem();
	void cadScanner();
	const LoRaFreqTable* getLoRaFrequencies(LoRaBands band);

	// void ICACHE_RAM_ATTR Interrupt_0();
	// void ICACHE_RAM_ATTR Interrupt_1();
	// void ICACHE_RAM_ATTR Interrupt_2();
};
} // namespace LoRa

#endif // LORA_MODEM_HPP
