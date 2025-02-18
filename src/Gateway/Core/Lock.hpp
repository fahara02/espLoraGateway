#ifndef MUTEX_LOCK_HPP
#define MUTEX_LOCK_HPP
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
namespace Core
{
class Lock
{
  public:
	Lock(SemaphoreHandle_t sem, TickType_t timeout) : sem_(sem), acquired_(false)
	{
		if(xSemaphoreTake(sem_, timeout) == pdTRUE)
		{
			acquired_ = true;
		}
		else
		{
			ESP_LOGE("SemLock", "Failed to take mutex");
		}
	}

	~Lock()
	{
		if(acquired_)
		{
			xSemaphoreGive(sem_);
		}
	}

	bool acquired() const
	{
		return acquired_;
	}

	Lock(const Lock&) = delete;
	Lock& operator=(const Lock&) = delete;

  private:
	SemaphoreHandle_t sem_;
	bool acquired_;
};

} // namespace Core

#endif