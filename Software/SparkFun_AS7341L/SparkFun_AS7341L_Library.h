/*
  This is a library written for the AMS AS7341L 10-Channel Spectral Sensor Frontend
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, March 15th, 2021
  This file declares the core functions available in the AS7341L sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __SPARKFUN_AS7341L_LIBRARY__
#define __SPARKFUN_AS7341L_LIBRARY__

#include "SparkFun_AS7341L_Constants.h"
#include "SparkFun_AS7341L_IO.h"
#include <SparkFun_PCA9536_Arduino_Library.h>		// Get library here: https://github.com/sparkfun/SparkFun_PCA9536_Arduino_Library

class SparkFun_AS7341L
{
private:
	SparkFun_AS7341L_IO as7341_io;
	PCA9536 pca9536_io;
	byte lastError;	
	
	bool whiteLedPowered = false;
	bool IRLedPowered = false;
	
	// Sets F1 to F4 + Clear + NIR to ADCs inputs
	void setMuxLo();
	
	// Sets F5 to F8 + Clear + NIR to ADCs inputs
	void setMuxHi();	

public:
	// Default constructor
	SparkFun_AS7341L() {}
	
	// Initialize AS7341L and PCA9536
	bool begin(byte AS7341L_address = DEFAULT_AS7341L_ADDR, TwoWire& wirePort = Wire);	
	
	// Check if board's devices are connected and responding properly
	bool isConnected();
	
	// Read all channels raw values
	bool readAllChannels(unsigned int* channelData);
	
	// Read a single channel raw value
	unsigned int readSingleChannel(AS7341L_CHANNELS channel);
	
	// Read all channels basic counts. Further information can be found in AN000633, page 7
	bool readAllChannelsBasicCounts(float* channelDataBasicCounts);
	
	// Read a single channel basic count.
	float readSingleChannelBasicCount(AS7341L_CHANNELS channel);
	
	// Enable AS7341L
	void enable_AS7341L();
	
	// Power down AS7341L
	void disable_AS7341L();
	
	// Return last error 
	byte getLastError();
	
	// Set LED driver forward current value
	void setLedDrive(unsigned int current);
	
	// Return LED driver forward current value
	unsigned int getLedDrive();
	
	// Enable power on red LED
	void enablePowerLed();
	
	// Disable power on red LED
	void disablePowerLed();
	
	// Turn white LED on  
	void enableWhiteLed();
	
	// Turn white LED off
	void disableWhiteLed();
	
	// Turn infrared LED on 
	void enableIRLed();
	
	// Turn infrared LED off
	void disableIRLed();
	
	// Set ADC integration time
	void setATIME(byte aTime = 29);
	
	// Set ADC integration steps
	void setASTEP(unsigned int aStep = 599);
	
	// Return ADC integration time
	byte getATIME();
	
	// Get ADC integration steps
	unsigned int getASTEP();
	
	// Set ADC gain
	void setGain(AS7341L_GAIN gain = AS7341L_GAIN::GAIN_X256);
	
	// Return ADC gain
	AS7341L_GAIN getGain();

	// Enables interrupt pin functionality
	void enableInterupt();
	
	// Disables interrupt pin functionality
	void disableInterrupt();
	
	// Clears the interrupt flag
	void clearInterrupt();
	
	// Reads register
	byte readRegister(byte reg);
	
	// Writes register
	void setRegister(byte reg, byte value);
	
	// Sets GPIO as input
	void setGpioPinInput();
	
	// Sets GPIO as output
	void setGpioPinOutput();
	
	// Reads GPIO pin value
	bool digitalRead();
	
	// Inverts GPIO pin output
	void invertGpioOutput(bool isInverted);
	
	// Writes GPIO pin
	void digitalWrite(byte value);
};

#endif // ! __SPARKFUN_AS7341L_LIBRARY__