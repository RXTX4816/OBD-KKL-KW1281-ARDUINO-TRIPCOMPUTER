
/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */
/* Config */
#define DEBUG 1                  // 1 = enable Serial.print
#define DEBUG_GUI 3              // 0 = ERRORS only. 1=+WARN. 2=+INFO. 3=+OBD messages=EVERYTHING
#define ECU_TIMEOUT 1300         // Most commonly is 1100ms

/* Config KLineKWP1281Lib */
#define debug_info true
#define debug_traffic true
#define is_full_duplex true // todo refactor
#define K_line Serial3
#define TX_pin 14



/* Config OBDisplay */
#define DEFAULT_UI_SIMULATION_MODE false
#define DEFAULT_UI_BAUDRATE 10400
#define DEFAULT_UI_DEBUG_MODE true
#define DEFAULT_UI_ECU_ADDRESS 17 
#define DEFAULT_UI_NIGHT_MODE true
#define DEFAULT_UI_MENU 0 // 0 Cockpit 1 Xperimental 2 debug 3 dtc 4 settings 5 graphic


/* Pins */ //Using: Serial3
const uint8_t pin_rx = 15; // Receive // Black
const uint8_t pin_tx = 14; // Transmit // White

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_ABS_BRAKES = 0x03; // UNUSED
const uint8_t ADDR_AUTO_HVAC = 0x08;  // UNUSED
const uint8_t ADDR_INSTRUMENTS = 0x17;
const uint8_t ADDR_CENTRAL_CONV = 0x46;

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */
// Constants
const uint8_t KWP_MODE_ACK = 0;         // Send ack block to keep connection alive
const uint8_t KWP_MODE_READSENSORS = 1; // Read all sensors from the connected ADDR
const uint8_t KWP_MODE_READGROUP = 2;   // Read only specified group from connected ADDR
const uint8_t supported_baud_rates_max = 4;
const uint8_t supported_baud_rates_length = supported_baud_rates_max + 1;
const uint16_t supported_baud_rates[supported_baud_rates_length] = {1200, 2400, 4800, 9600, 10400}; // Select Baud rate: 4800, 9600 or 10400 Baud depending on your ECU. This can get confusing and the ECU may switch the baud rate after connecting.
const unsigned long timeout_to_add = 1100; // Wikipedia

// EEPROM
//      Constants
const uint8_t ID_EEPROM_BAUDRATE = 0x00;
const uint8_t ID_EEPROM_ECU_ADDRESS = 0x01;
const uint8_t ID_EEPROM_IS_SIMULATION_MODE = 0x02;
const uint8_t ID_EEPROM_IS_DEBUG_MODE = 0x03;
//      Variables
bool is_first_EEPROM_setup = false;
uint8_t EEPROM_values[4] = {0xFF, 0xFF, 0xFF, 0xFF};
