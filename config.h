/*
  Project Name:   rco2
  Description:    non-secret configuration data
*/

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug parameters
// comment out to turn off; 1 = summary, 2 = verbose
#define DEBUG 2

// Configuration Step 3: simulate hardware inputs, returning random but plausible values
// comment out to turn off
// #define SENSOR_SIMULATE

// Configuration variables that change rarely

// Buttons
const uint8_t buttonD1Pin = 1; // initially LOW
const uint8_t buttonD2Pin = 2; // initially LOW

const int buttonDebounceDelay = 50; // time in milliseconds to debounce button

// Display
const uint8_t displayRotation = 3; // rotation 3 orients 0,0 next to D0 button

// Battery
const float batteryVoltageMinAlert = 3.7;
const float batteryVoltageMaxAlert = 4.2;

// Simulation values
#ifdef SENSOR_SIMULATE
  const uint16_t sensorTempMin =      1500; // will be divided by 100.0 to give floats
  const uint16_t sensorTempMax =      2500;
  const uint16_t sensorHumidityMin =  500; // will be divided by 100.0 to give floats
  const uint16_t sensorHumidityMax =  9500;
  const uint16_t sensorCO2Min =       400;
  const uint16_t sensorCO2Max =       3000;

  const uint16_t batterySimVoltageMin = 370; // will be divided by 100.0 to give floats
  const uint16_t batterySimVoltageMax = 420;
#endif

// CO2 sensor
//sample timing
#ifdef DEBUG
	// number of times SCD40 is read, last read is the sample value
	const uint8_t sensorReadsPerSample =	1;
	// time between samples in seconds
  const uint16_t sensorSampleInterval = 60;
#else
  const uint8_t sensorReadsPerSample =  3;
  const uint16_t sensorSampleInterval = 120;
#endif
const String co2Labels[5]={"Good", "OK", "So-So", "Poor", "Bad"};
// Subjective color scheme using 16 bit ('565') RGB colors a la ST77XX display
const uint16_t co2Highlight[5] = {
    0x07E0,   // GREEN = "Good"
    0x07E0,   // GREEN = "OK"
    0xFFE0,   // YELLOW = "So-So"
    0xFC00,   // ORANGE = "Poor"
    0xF800    // RED = "Bad"
  };

// Hardware
// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;

//                       ***** BOARD SUPPORT *****
// Here is where we define characteristics specific to the various
// supported microcontroller boards, as may be required.  Wherever possible 
// these defined symbols should match those provided by the board for the
// Arduino IDE, and so don't need to be explicitly defined here (just used)
// 
// Only one of the following should be defined for any particular build.  If not
// the compiler will almost certainly complain!
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT // Adafruit ESP32-S2 Reverse TFT (Prod )
  // This board includes a built-in TFT display, with wiring pre-defined for the IDE

#endif // End: ARDUINO_ADAFRUIT_FEATHER_ESP32S3_REVTFT


#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2  // Adafruit ESP32 Feather V2 (Prod ID: 5400)
  // With this board RCO2 relies on a separate TFT breakout board and so  
  // needs declaration of the digital I/O pins used to drive the display
  #define TFT_CS        15
  #define TFT_RST       32 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC        14

  // Other board-specific aspects
  // *) Battery monitor voltage divider is connected to pin A13. To measure battery voltage
  //    read pin A13's analog voltage and double it.
  // *) To enable I2C (and the onboard Neopixel), pull the NEOPIXEL_I2C_POWER pin HIGH.
  //    The Arduino board configuration does this automatically.
  // *) The user-readable pusbutton on the board is connected to a pin defined for Arduino
  //    as BUTTON (labeled "SW38" on the board itself)
  #define VBATPIN A13  // Used by battery management routines
#endif // End ARDUINO_ADAFRUIT_FEATHER_ESP32_V22
