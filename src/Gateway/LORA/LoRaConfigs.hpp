#ifndef LORA_CONFIGS_HPP
#define LORA_CONFIGS_HPP

#include <cstdint>
namespace LoRa
{
static constexpr uint16_t LORA_MAX_CHANNEL = 10;
static constexpr uint16_t LORA_MAX_REGION = 9;

static constexpr uint16_t LORA_PAYLOAD_LENGTH = 0x40;
static constexpr uint16_t MAX_LORA_PAYLOAD_LENGTH = 0x40;
static constexpr uint16_t MAX_BUFFER_LENGTH = 24;
static constexpr uint16_t MAX_HOPS = 3;
static constexpr uint16_t RSSI_LIMITS = 35;
static constexpr uint16_t RSSI_WAIT = 6;
static constexpr unsigned long EVENT_WAIT_MS = 15000;
static constexpr unsigned long DONE_WAIT_MS = 1950;
static constexpr unsigned long WAIT_CORRECTION_MS = 20000;
static constexpr unsigned long SPI_SPEED = 8000000;
static constexpr uint16_t MAX_LORA_PAYLOAD = 128;
static constexpr uint64_t FXOC = 32000000;

static constexpr uint16_t BUFFER_BYTES = 2;
static constexpr bool Monitor = true;

} // namespace LoRa
#endif