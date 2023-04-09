//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
#include <Arduino.h>
#include "Pixy2.h"

Pixy2 pixy;

void test_leds(uint32_t ms)
{
	const uint8_t FADES[] = {0, 10, 35, 60, 100, 190, 255};
	uint16_t i = 0;

	Serial.println("Test LEDs!");
	ms += millis();
	while (millis() < ms) {
		pixy.setLamp(i & 0b10, 0);
		pixy.setLED(FADES[(i + 4) % sizeof(FADES)], FADES[(i + 2) % sizeof(FADES)], FADES[(i + 0) % sizeof(FADES)]);
		i++;
		delay(200);
	}
	pixy.setLED(0, 0, 0);
	pixy.setLamp(0, 0);
}

void test_blocks(uint32_t ms)
{
	Serial.println("Test Blocks!");
	ms += millis();
	while (millis() < ms) {
		pixy.ccc.getBlocks();
		if (pixy.ccc.numBlocks) {
			Serial.print("Detected ");
			Serial.println(pixy.ccc.numBlocks);
			for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
				Serial.print("  block ");
				Serial.print(i);
				Serial.print(": ");
				pixy.ccc.blocks[i].print();
			}
		}
	}
}

void test_lines(uint32_t ms)
{
	char buf[128];

	Serial.println("Test Lines!");
	ms += millis();

	pixy.changeProg("line");
	while (millis() < ms) {
		pixy.line.getAllFeatures();

		// pixy.line.vectors->print();
		for (uint16_t i=0; i<pixy.line.numVectors; i++) {
			sprintf(buf, "line %d: ", i);
			Serial.print(buf);
			pixy.line.vectors[i].print();
		}

		// pixy.line.intersections->print();
		for (uint16_t i=0; i<pixy.line.numIntersections; i++) {
			sprintf(buf, "intersection %d: ", i);
			Serial.print(buf);
			pixy.line.intersections[i].print();
		}

		// pixy.line.barcodes->print();
		for (uint16_t i=0; i<pixy.line.numBarcodes; i++) {
			sprintf(buf, "barcode %d: ", i);
			Serial.print(buf);
			pixy.line.barcodes[i].print();
		}
	}
}

void test_raw_pixels(uint32_t ms)
{
	uint8_t r, g, b;
	uint8_t x = pixy.frameWidth / 2, y = pixy.frameHeight / 2;

	Serial.println("Test Pixels!");
	ms += millis();

	pixy.changeProg("video");
	while (millis() < ms) {
		if (!pixy.video.getRGB(x, y, &r, &g, &b)) {
			Serial.print("red:");
			Serial.print(r);
			Serial.print(" green:");
			Serial.print(g);
			Serial.print(" blue:");
			Serial.println(b);
		}
	}
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Starting...");

	pixy.init();
	Serial.println("Started!");
}

void loop()
{
	test_leds(10000);
	test_blocks(10000);
	test_lines(10000);
	test_raw_pixels(10000);
}
