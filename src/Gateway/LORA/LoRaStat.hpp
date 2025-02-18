#ifndef LORA_STAT_HPP
#define LORA_STAT_HPP
#include "etl/optional.h"
#include "etl/array.h"
#include "LoRaConfigs.hpp"
#include "LoRaDefines.hpp"

namespace LoRa
{

struct Stat
{
	virtual void reset() = 0;
	virtual ~Stat() = default;
};

struct StatMessage : public Stat
{
	uint32_t timestamp{0}; // Time since 1970 in seconds
	uint32_t nodeId{0}; // 4-byte DEVaddr (unique to the gateway)
	uint8_t channel{0}; // Channel index in the frequency array
	uint8_t spreadingFactor{0};
	etl::optional<int8_t> rssi{}; // Optional RSSI
	int8_t prssi{0}; // Packet RSSI
	uint8_t direction{0}; // 0 = uplink, 1 = downlink

	// Optional: Store up to 24 bytes of decoded message if LOCAL SERVER enabled
	etl::optional<etl::array<uint8_t, 24>> data;
	etl::optional<uint8_t> dataLength;

	void reset() override
	{
		timestamp = 0;
		nodeId = 0;
		channel = 0;
		spreadingFactor = 0;
		rssi.reset();
		prssi = 0;
		direction = 0;
		data.reset();
		dataLength.reset();
	}

	void enableOptional(StatisticsLevel level)
	{
		if(level >= StatisticsLevel::MESSAGE_STAT_LOCAL_SERVER)
		{
			data.emplace();
			dataLength.emplace(0);
		}
		else
		{
			data.reset();
			dataLength.reset();
		}
	}
};

struct StatGateway : public Stat
{
	uint32_t msgOk{0};
	uint32_t msgTotal{0};
	uint32_t msgDown{0};
	uint32_t msgSensitive{0};

	// Optional spreading factor statistics (SF7-SF12)
	etl::optional<etl::array<uint32_t, 6>> spreadingFactors;

	// Optional per-channel statistics
	struct PerChannelStats
	{
		etl::array<uint32_t, 3> msgOk{};
		etl::array<uint32_t, 3> msgTotal{};
		etl::array<uint32_t, 3> msgDown{};
		etl::array<uint32_t, 3> msgSensitive{};
		etl::array<etl::array<uint32_t, 3>, 6> sfStats{}; // SF7-SF12 per channel

		void reset()
		{
			msgOk.fill(0);
			msgTotal.fill(0);
			msgDown.fill(0);
			msgSensitive.fill(0);
			for(auto& sf: sfStats)
			{
				sf.fill(0);
			}
		}
	};
	etl::optional<PerChannelStats> channelStats;

	etl::optional<uint16_t> bootCount;
	etl::optional<uint16_t> resetCount;

	void reset() override
	{
		msgOk = msgTotal = msgDown = msgSensitive = 0;
		spreadingFactors.reset();
		channelStats.reset();
		bootCount.reset();
		resetCount.reset();
	}

	void enableOptional(StatisticsLevel level)
	{
		if(level >= StatisticsLevel::GATEWAY_STAT_SF)
		{
			spreadingFactors.emplace();
			spreadingFactors->fill(0);
		}
		else
		{
			spreadingFactors.reset();
		}

		if(level >= StatisticsLevel::GATEWAY_STAT_CHANNEL)
		{
			channelStats.emplace();
			channelStats->reset();
		}
		else
		{
			channelStats.reset();
		}

		if(level >= StatisticsLevel::GATEWAY_STAT_BASE)
		{
			bootCount.emplace(0);
			resetCount.emplace(0);
		}
		else
		{
			bootCount.reset();
			resetCount.reset();
		}
	}
};

class ModemStats
{
  public:
	ModemStats() = default;

	void setStatisticsLevel(StatisticsLevel level)
	{
		level_ = level;

		// Enable/disable optional fields based on level
		messageStats_.enableOptional(level);
		gatewayStats_.enableOptional(level);
	}

	Stat* getStat(StatisticsLevel lv)
	{
		switch(lv)
		{
			case StatisticsLevel::MESSAGE_STAT:
			case StatisticsLevel::MESSAGE_STAT_LOCAL_SERVER:
				return &messageStats_;
			case StatisticsLevel::GATEWAY_STAT_BASE:
			case StatisticsLevel::GATEWAY_STAT_SF:
			case StatisticsLevel::GATEWAY_STAT_CHANNEL:
				return &gatewayStats_;
			default:
				return nullptr;
		}
	}

  private:
	StatisticsLevel level_{StatisticsLevel::NONE};
	StatMessage messageStats_;
	StatGateway gatewayStats_;
};

} // namespace LoRa

#endif