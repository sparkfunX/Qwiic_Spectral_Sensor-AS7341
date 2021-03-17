/*
  Using the AS7341L 10 channel spectral sensor
  By: Ricardo Ramos
  SparkFun Electronics
  Date: March 17th, 2021
  SparkFun code, firmware, and software is released under the MIT License. Please see LICENSE.md for further details.
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  This example shows how to write AS7341L GPIO output pin to drive a LED.
  GPIO is an open drain output so it will not write a HIGH value but only LOW values. 
  In this example, GPIO sinks current through a LED, lighting it when the GPIO pin is grounded.
  The pin will float when the GPIO is written HIGH. If you need to use GPIO as an input for a logic circuit
  you may need to add a pull-up resistor. You can find further information on that in this tutorial:
  https://learn.sparkfun.com/tutorials/pull-up-resistors/all
  
  Hardware Connections:
  - Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  - Plug the sensor onto the shield
  - Connect the GPIO pin to a LED cathode pin. Connect LED anode pin to 3.3V through an 1k resistor.
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
	
	// Configure GPIO as output
	as7341L.setGpioPinOutput();
	
	// Turn off GPIO
	as7341L.digitalWrite(LOW);
	
	// Bring AS7341L to the powered up state
	as7341L.enable_AS7341L();
	
	// Turn off the led connected to pin 13
	digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
	// Turn on the LED by connecting it's cathode to GND through the GPIO pin
	as7341L.digitalWrite(LOW);
	// Wait a little...
	delay(500);
	
	// Turn off the LED by disconnecting it's cathode pin
	as7341L.digitalWrite(HIGH);
	// Wait a little and start over
	delay(500);
}
