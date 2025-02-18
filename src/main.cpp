#ifndef UNIT_TEST
	#include <Arduino.h>

	#include "FS.h"
	#include <LittleFS.h>
	#include "espGateway.hpp"

void setup()
{
	LOG::ENABLE();
	// LOG::INFO("MAIN", "first info");
	Serial.begin(115200);
	delay(500);

	LOG::INFO("Main", "Setup initialized");
}

void loop()
{
	LOG::INFO("Main", "Loop running");
	delay(1000);
}
#endif