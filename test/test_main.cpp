#include <Arduino.h>
#include <unity.h>
#include "test_LoRaModem.hpp" // Adjust if needed

// Dummy definitions that Arduino's core main() will call
void setup()
{
	// Optional: delay to allow serial monitor connection
	delay(2000);
	UNITY_BEGIN();
	// Run your test suite(s)
	MockLoRaModem::runAllTests();
	UNITY_END();
}

void loop()
{
	// Do nothing (or add a delay to avoid watchdog issues)
	delay(1000);
}
