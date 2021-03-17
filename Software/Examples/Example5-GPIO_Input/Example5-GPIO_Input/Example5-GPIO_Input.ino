/*
  Using the AS7341L 10 channel spectral sensor
  By: Ricardo Ramos
  SparkFun Electronics
  Date: March 17th, 2021
  SparkFun code, firmware, and software is released under the MIT License. Please see LICENSE.md for further details.
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  This example shows how to read AS7341L GPIO input pin, prints out it's value and reflects it's state in the LED
  connected to pin 13.
  
  Hardware Connections:
  - Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  - Plug the sensor onto the shield
  - Feed Qwiic shield GPIO pin with either GND (LOW) or 3.3V (HIGH). 
  *** DO NOT CONNECT GPIO TO 5V - DAMAGE TO THE SENSOR MAY OCCUR ! ***
  - Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>
#include "SparkFun_AS7341L_Library.h"

// Main AS7341L object
SparkFun_AS7341L as7341L;

// Print a friendly error message
void PrintErrorMessage()
{
	switch (as7341L.getLastError())
	{
	case ERROR_AS7341L_I2C_COMM_ERROR:
		Serial.println("Error: AS7341L I2C communication error");
		break;

	case ERROR_PCA9536_I2C_COMM_ERROR:
		Serial.println("Error: PCA9536 I2C communication error");
		break;
    
	case ERROR_AS7341L_MEASUREMENT_TIMEOUT:
		Serial.println("Error: AS7341L measurement timeout");
		break;
    
	default:
		break;
	}
}

void setup()
{
	// Configure Arduino's built in LED as output
	pinMode(LED_BUILTIN, OUTPUT);

	// Initialize serial port at 115200 bps
	Serial.begin(115200);

	// Initialize the I2C port
	Wire.begin();

	// Initialize AS7341L
	boolean result = as7341L.begin();

	// If the board did not properly initialize print an error message and halt the system
	if(result == false)
	{
		PrintErrorMessage();
		Serial.println("Check you connections. System halted !");
		digitalWrite(LED_BUILTIN, LOW); 
		while (true) ;
	}
	
	// Configure GPIO as input
	as7341L.setGpioPinInput();
	
	// Bring AS7341L to the powered up state
	as7341L.enable_AS7341L();
	
	// Turn off the led connected to pin 13
	digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
	// Read GPIO pin value
	bool pin = as7341L.digitalRead();
	
	// Check pin value and print out message/turn the LED on or off accordingly
	if (pin == true)
	{
		Serial.println("Gpio pin is high.");
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else
	{
		Serial.println("Gpio pin is low.");
		digitalWrite(LED_BUILTIN, LOW);
	}
	
	// Wait and start over
	delay(500);
}
