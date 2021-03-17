/*
  Using the AS7341L 10 channel spectral sensor
  By: Ricardo Ramos
  SparkFun Electronics
  Date: March 17th, 2021
  SparkFun code, firmware, and software is released under the MIT License. Please see LICENSE.md for further details.
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/

  This example shows how to read all channels (F1 to F8, CLEAR and NIR) and get raw values from the sensor following
  an interrupt from AS7341L.
  
  Hardware Connections:
  - Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  - Plug the sensor onto the shield
  - Connect a jumper wire from the INT pin in the Qwiic shield to pin 2 of your Arduino/Photon/ESP32 board.
  - Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>
#include "SparkFun_AS7341L_Library.h"

// Main AS7341L object
SparkFun_AS7341L as7341L;

// Sample number variable
unsigned int sampleNumber = 0;

// Pin 2 is used as an interrupt input pin
const byte interruptPin = 2;

// Global variable which indicates that there's new data available
volatile bool getData;

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

	// Configure interrupt pin mode
	pinMode(interruptPin, INPUT);
	
	// Enable interrupt service routine to be triggered by the rising edge of interruptPin
	attachInterrupt(digitalPinToInterrupt(interruptPin), interruptServiceRoutine, FALLING);

	// Initialize our interrupt flag as false
	getData = false;
	
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
	
	// Bring AS7341L to the powered up state
	as7341L.enable_AS7341L();

	// Enable interrupt pin
	as7341L.enableInterupt();
	
	// If the board was properly initialized, turn on LED_BUILTIN
	if(result == true)
	  digitalWrite(LED_BUILTIN, HIGH);
	
	// Clear AS7341L interrupt flag so it can be used
	as7341L.clearInterrupt();
	
}

void loop()
{
	// Array which contains all channels raw values
	unsigned int channelReadings[12] = { 0 };  

	// Have AS7341L to start measurement
	bool result = as7341L.readAllChannels(channelReadings);
	
	// Wait until we have a valid measurement
	while(getData != true)
	{}

	// Reset the flag so it can be used again
	getData = false;

	// Check if the read operation was successful and print out results
	if(result == true)
	{
		Serial.println("---------------------------------");
		Serial.print("Sample number: ");
		Serial.println(++sampleNumber);
		Serial.println();
		Serial.print("F1 (415 nm): ");
		Serial.println(channelReadings[0]);
		Serial.print("F2 (445 nm): ");
		Serial.println(channelReadings[1]);
		Serial.print("F3 (480 nm): ");
		Serial.println(channelReadings[2]);
		Serial.print("F4 (515 nm): ");
		Serial.println(channelReadings[3]);
    
		// channelReadings[4] and [5] hold same values as [10] and [11] for CLEAR and NIR, respectively
    
		Serial.print("F5 (555 nm): ");
		Serial.println(channelReadings[6]);
		Serial.print("F6 (590 nm): ");
		Serial.println(channelReadings[7]);
		Serial.print("F7 (630 nm): ");
		Serial.println(channelReadings[8]);
		Serial.print("F8 (680 nm): ");
		Serial.println(channelReadings[9]);
		Serial.print("Clear: ");
		Serial.println(channelReadings[10]);
		Serial.print("NIR: ");
		Serial.println(channelReadings[11]);
		Serial.println();

		// Linger a little...
		delay(1000);
		
		// Clear AS7341L interrupt flag so it can be triggered back again
		as7341L.clearInterrupt();
	}
	else
	{
		// Ooops ! We got an error !
		PrintErrorMessage();
	}
}

// This function will be called whenever AS7341L has available data to be read
void interruptServiceRoutine()
{
	// Just set the flag to indicate that there's data available
	getData = true;
}