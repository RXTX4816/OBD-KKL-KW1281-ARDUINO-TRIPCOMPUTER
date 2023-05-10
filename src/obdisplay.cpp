/*
OBDisplay.cpp

See readme for more info.

See https://www.blafusel.de/obd/obd2_kw1281.html for info on OBD KWP1281 protocol.

Ignore compile warnings.
*/

// Arduino/Standard Libraries
#include <Arduino.h>
// #include <EEPROM.h>
// #include <Wire.h>
// #include <time.h>
//  Third party libraries
#include "LiquidCrystal.h"
#include "NewSoftwareSerial.h"

/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */

/* Config */
// bool no_input_mode = false; // If you have no buttons connected, mainly used for fast testing
// bool auto_setup = false;
// bool compute_stats = false; // Whether statistic values should be computed (Fuel/100km etc.) Remember division is expensive on these processors.
#define DEBUG 1                  // 1 = enable Serial.print
#define ECU_TIMEOUT 1300         // Most commonly is 1100ms
#define DISPLAY_FRAME_LENGTH 333 // Length of 1 frame in ms
#define DISPLAY_MAX_X 16
#define DISPLAY_MAX_Y 2
#define PIN_RX 3 // Receive
#define PIN_TX 2 // Transmit
uint8_t simulation_mode_active = false;
#define BUTTON_RIGHT(in) (in < 60)
#define BUTTON_UP(in) (in >= 60 && in < 200)
#define BUTTON_DOWN(in) (in >= 200 && in < 400)
#define BUTTON_LEFT(in) (in >= 400 && in < 600)
#define BUTTON_SELECT(in) (in >= 600 && in < 800)

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_ABS_BRAKES = 0x03;
const uint8_t ADDR_AUTO_HVAC = 0x08;
const uint8_t ADDR_INSTRUMENTS = 0x17;
const uint8_t ADDR_CENTRAL_CONV = 0x46;

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */

// Constants
const uint8_t KWP_MODE_ACK = 0;         // Send ack block to keep connection alive
const uint8_t KWP_MODE_READSENSORS = 1; // Read all sensors from the connected ADDR
const uint8_t KWP_MODE_READGROUP = 2;   // Read only specified group from connected ADDR
const char CHAR_YES = 'Y';
const char CHAR_NO = 'N';
char STRING_ERR[] = "ERR";

// Backend
NewSoftwareSerial obd(PIN_RX, PIN_TX, false); // rx, tx, inverse logic = false
uint32_t connect_time_start = millis();
uint16_t timeout_to_add = 1100; // Wikipedia
uint16_t button_timeout = 222;
uint8_t screen_current = 0;
uint8_t menu_current = 0;
uint8_t kwp_mode = KWP_MODE_READSENSORS;
uint8_t kwp_mode_last = kwp_mode;
uint8_t kwp_group = 1; // Dont go to group 0 its not good.
// ---------------Menu Screen-----------
uint8_t menu_cockpit_screen = 0;
uint8_t menu_cockpit_screen_max = 4;
uint8_t menu_experimental_screen = 0;
uint8_t menu_experimental_screen_max = 64;
uint8_t menu_debug_screen = 0;
uint8_t menu_debug_screen_max = 4;
uint8_t menu_dtc_screen = 0;
uint8_t menu_dtc_screen_max = 9;
uint8_t menu_settings_screen = 0;
uint8_t menu_settings_screen_max = 10;
bool menu_screen_switch = false;
// -------------------------------------
uint32_t endTime = 0;
uint8_t menu_max = 4;
uint8_t menu = 0;
bool menu_switch = false;
uint16_t connection_attempts_counter = 0;
uint64_t button_read_time = 0;

// OBD Connection variables
bool connected = false;
uint16_t baud_rate = 0; // 1200, 2400, 4800, 9600, 10400
uint8_t block_counter = 0;
uint8_t group_current = 1;
bool group_current_updated = false;
uint8_t addr_selected = 0x00; // Selected ECU address to connect to, see ECU Addresses constants
bool com_error = false;

/* Temporary Measurements for if you want to find out which values show up in your groups in a desired ECU address.
Just uncomment and add the logic in readSensors(). This can also be done with VCDS or other tools.*/
bool group_side = false; // 0 = display k0 and k1. 1= display k2 and k3
bool group_side_updated = false;
void invert_group_side()
{
  group_side = !group_side;
  group_side_updated = true;
  menu_switch = true;
}
uint8_t k_temp[4] = {0, 0, 0, 0};
bool k_temp_updated = false;
float v_temp[4] = {-1, -1, -1, -1};
bool v_temp_updated = false;
String unit_temp[4] = {STRING_ERR, STRING_ERR, STRING_ERR, STRING_ERR};
bool unit_temp_updated = false;
void reset_temp_group_array()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    k_temp[i] = 0;
    v_temp[i] = 0;
    unit_temp[i] = STRING_ERR;
  }
}
// DTC error
uint16_t dtc_errors[16] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
bool dtc_errors_updated = false;
uint8_t dtc_status_bytes[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool dtc_status_bytes_updated = false;
void reset_dtc_status_errors_array()
{
  for (uint8_t i = 0; i < 8; i++)
  {
    dtc_errors[i] = 0x0000;
    dtc_status_bytes[i] = 0x00;
  }
}

/* ADDR_INSTRUMENTS measurement group entries, chronologically 0-3 in each group */

// Group 1
uint16_t vehicle_speed = 0;
bool vehicle_speed_updated = false;
uint16_t engine_rpm = 0;
bool engine_rpm_updated = false;
uint16_t oil_pressure_min = 0;
bool oil_pressure_min_updated = false;
uint32_t time_ecu = 0;
bool time_ecu_updated = false;
// Group 2
uint32_t odometer = 0;
bool odometer_updated = false;
uint32_t odometer_start = odometer;
uint8_t fuel_level = 0;
bool fuel_level_updated = false;
uint8_t fuel_level_start = fuel_level;
uint16_t fuel_sensor_resistance = 0;
bool fuel_sensor_resistance_updated = false; // Ohm
uint8_t ambient_temp = 0;
bool ambient_temp_updated = false;
// Group 3 (Only 0-2)
uint8_t coolant_temp = 0;
bool coolant_temp_updated = false;
uint8_t oil_level_ok = 0;
bool oil_level_ok_updated = false;
uint8_t oil_temp = 0;
bool oil_temp_updated = false;
// ADDR_ENGINE measurement group entries TODO
// Group 1 (0th is engine rpm)
uint8_t temperature_unknown_1 = 0; // 1
bool temperature_unknown_1_updated = false;
int8_t lambda = 0; // 2
bool lambda_updated = false;
bool exhaust_gas_recirculation_error = false; // 3, 8 bit encoding originally
bool oxygen_sensor_heating_error = false;
bool oxgen_sensor_error = false;
bool air_conditioning_error = false;
bool secondary_air_injection_error = false;
bool evaporative_emissions_error = false;
bool catalyst_heating_error = false;
bool catalytic_converter = false;
bool error_bits_updated = false;
String bits_as_string = "        ";
// Group 3 (Only 1-3 no 0th)
uint16_t pressure = 0; // mbar
bool pressure_updated = false;
float tb_angle = 0;
bool tb_angle_updated = false;
float steering_angle = 0;
bool steering_angle_updated = false;
// Group 4 (Only 1-3 no 0th)
float voltage = 0;
bool voltage_updated = false;
uint8_t temperature_unknown_2 = 0;
bool temperature_unknown_2_updated = false;
uint8_t temperature_unknown_3 = 0;
bool temperature_unknown_3_updated = false;
// Group 6 (Only 1 and 3)
uint16_t engine_load = 0; // 1
bool engine_load_updated = false;
int8_t lambda_2 = 0; // 3
bool lambda_2_updated = false;

/* Computed Stats */
uint32_t elapsed_seconds_since_start = 0;
bool elapsed_seconds_since_start_updated = false;
uint16_t elpased_km_since_start = 0;
bool elpased_km_since_start_updated = false;
uint8_t fuel_burned_since_start = 0;
bool fuel_burned_since_start_updated = false;
float fuel_per_100km = 0;
bool fuel_per_100km_updated = false;
float fuel_per_hour = 0;
bool fuel_per_hour_updated = false;

void reset_variables()
{
  block_counter = 0;
  connected = false;
  connect_time_start = 0;
  odometer_start = 0;
  fuel_level_start = 0;
  menu = 0;
  menu_switch = false;
  button_read_time = 0;
  addr_selected = 0x00;
  screen_current = 0;
  menu_current = 0;
}

// Serial debug
#if DEBUG == 1 // Compile Serial
#define debug(in) Serial.print(in)
#define debughex(in) Serial.print(in, HEX)
#define debugln(in) Serial.println(in)
#define debughexln(in) Serial.println(in, HEX)
#define debugstrnum(str, num) \
  do                          \
  {                           \
    debug(str);               \
    debug(num);               \
  } while (0)
#define debugstrnumln(str, num) \
  do                            \
  {                             \
    debug(str);                 \
    debugln(num);               \
  } while (0)
#define debugstrnumhexln(str, num) \
  do                               \
  {                                \
    debug(str);                    \
    debughexln(num);               \
  } while (0)
#else // Do not compile serial to save space
#define debug(in)
#define debughex(in)
#define debugln(in)
#define debughexln(in)
#endif

/* Display */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // 16x2 display
uint32_t display_frame_timestamp = millis();

uint8_t count_digit(int n)
{
  if (n == 0)
    return 1;
  uint8_t count = 0;
  while (n != 0)
  {
    n = n / 10;
    ++count;
  }
  return count;
}

/**
 * @brief Converts a boolean to a String
 *
 * @param value Boolean
 * @return String Y or N
 */
String convert_bool_string(bool value)
{
  if (value)
  {
    return String(CHAR_YES);
  }
  else
  {
    return String(CHAR_NO);
  }
}
char convert_bool_char(bool value)
{
  if (value)
  {
    return CHAR_YES;
  }
  else
    return CHAR_NO;
}

/**
 * Increment block counter. Min: 0, Max: 254.
 * Counts the current block number and is passed in each block.
 * A wrong block counter will result in communication errors.
 */
void increase_block_counter()
{
  if (block_counter >= 255)
  {
    block_counter = 0;
  }
  else
  {
    block_counter++;
  }
}

void increment_group_current()
{
  if (group_current > menu_experimental_screen_max)
  {
    group_current = 0;
  }
  else
  {
    group_current++;
  }
  group_current_updated = true;
}

void decrement_group_current()
{
  if (group_current == 0)
  {
    group_current = menu_experimental_screen_max;
  }
  else
  {
    group_current--;
  }
  group_current_updated = true;
}

// Display functions
void increment_menu(uint8_t &menu_current, uint8_t &menu_current_max)
{
  if (menu_current >= menu_current_max)
  {
    menu_current = 0;
  }
  else
  {
    menu_current++;
  }
  menu_switch = true;
}
void decrement_menu(uint8_t &menu_current, uint8_t &menu_current_max)
{
  if (menu_current == 0)
  {
    menu_current = menu_current_max;
  }
  else
  {
    menu_current--;
  }
  menu_switch = true;
}

// --------------------------------------------------------------------------------------------------
//                            DISPLAY CODE
// --------------------------------------------------------------------------------------------------

void lcd_print(uint8_t x, uint8_t y, String s)
{
  lcd.setCursor(x, y);
  lcd.print(s);
}

void lcd_print(uint8_t x, uint8_t y, String s, uint8_t width)
{
  while (s.length() < width)
    s += " ";
  lcd_print(x, y, s);
}

void lcd_clear(uint8_t x, uint8_t y, uint8_t width = 1)
{
  lcd.setCursor(x, y);
  for (uint8_t i = 0; i < width; i++)
  {
    lcd.print(" ");
  }
}

void lcd_print(uint8_t x, uint8_t y, int number)
{
  lcd.setCursor(x, y);
  lcd.print(number);
}
void lcd_print(uint8_t x, uint8_t y, uint8_t number)
{
  lcd_print(x, y, (int)number);
}
void lcd_print(uint8_t x, uint8_t y, uint16_t number)
{
  lcd_print(x, y, (int)number);
}
void lcd_print(uint8_t x, uint8_t y, uint32_t number)
{
  lcd_print(x, y, (int)number);
}

void lcd_print(uint8_t x, uint8_t y, float floating_number, uint8_t width = 0)
{
  lcd.setCursor(x, y);
  lcd.print(floating_number);
}

void lcd_print_bool(uint8_t x, uint8_t y, bool boolean, uint8_t width = 0)
{
  lcd_print(x, y, convert_bool_string(boolean), width);
}

void lcd_print_cockpit(uint8_t x, uint8_t y, String s, uint8_t width, bool &updated, bool force_update = false)
{
  if (updated || force_update)
  {
    lcd_clear(x, y, width);
    lcd_print(x, y, s);
    updated = false;
  }
}
void lcd_print_cockpit(uint8_t x, uint8_t y, uint16_t number, uint8_t width, bool &updated, bool force_update = false)
{
  if ((updated || force_update))
  {
    lcd_clear(x, y, width);
    uint8_t number_length = count_digit(number);
    if (number_length <= width)
    {
      lcd_print(x, y, number);
    }
    updated = false;
  }
}
void lcd_print_cockpit_float(uint8_t x, uint8_t y, float number, uint8_t width, bool &updated, bool force_update = false)
{
  if ((updated || force_update))
  {
    lcd_clear(x, y, width);
    uint8_t number_length = String(number, 2).length();
    if (number_length <= width)
    {
      lcd_print(x, y, number);
    }
    updated = false;
  }
}

void lcd_show_screen_not_supported(uint8_t screen)
{
  lcd_print(0, 0, "Screen", 7);
  lcd_print(7, 0, screen);
  lcd_print(0, 1, "not supported!");
}

void init_statusbar()
{
  lcd_print(0, 0, "C:");
  lcd_print(4, 0, "A:");
}
/**
 * Status bar: "C:Y A:9999*****"
 */
void display_statusbar()
{
  lcd_print(2, 0, connected);
  lcd_print(6, 0, obd.available());
}
void init_menu_cockpit()
{
  switch (addr_selected)
  {
  case ADDR_ENGINE:
    switch (menu_cockpit_screen)
    {
    case 0:
      lcd_print(15, 0, "V");
      lcd_print(13, 1, "TBa");
      break;
    case 1:
      lcd_print(10, 0, "load");
      lcd_print(13, 1, "STa");
      break;
    case 2:
      lcd_print(12, 0, "bits");
      lcd_print(10, 1, "lambda");
      break;
    case 3:
      lcd_print(6, 0, "kmh");
      lcd_print(8, 1, "mbar");
      break;
    case 4:
      lcd_print(6, 0, "C temp");
      lcd_print(6, 1, "C temp");
      break;
    default:
      lcd_show_screen_not_supported(menu_cockpit_screen);
      break;
    }
    break;
  case ADDR_INSTRUMENTS:
    switch (menu_cockpit_screen)
    {
    case 0:
      lcd_print(4, 0, "KMH");
      lcd_print(13, 0, "RPM");
      lcd_print(3, 1, "C");  // Coolant
      lcd_print(8, 1, "C");  // Oil
      lcd_print(13, 1, "L"); // Fuel
      break;
    case 1:
      lcd_print(2, 0, "OL");
      lcd_print(7, 0, "OP");
      lcd_print(13, 0, "AT");
      lcd_print(6, 1, "KM");
      lcd_print(13, 1, "FSR");
      break;
    case 2:
      lcd_print(6, 0, "TIME");
      lcd_print(7, 1, "L/100km");
      break;
    case 3:
      lcd_print(9, 0, "secs");
      lcd_print(6, 1, "km");
      break;
    case 4:
      lcd_print(6, 0, "km burned");
      lcd_print(7, 1, "L/h");
      break;
    default:
      lcd_show_screen_not_supported(menu_cockpit_screen);
      break;
    }
    break;
  default:
    // Addr not supported
    lcd_print(0, 0, "Addr", 5);
    lcd_print(6, 0, String(addr_selected, HEX), 2);
    lcd_print(0, 1, "not supported!");
    break;
  }
}
void init_menu_experimental()
{
  lcd_print(0, 0, "G:");
  lcd_print(0, 1, "S:");
}
void init_menu_debug()
{

  // Addr not supported
  lcd_print(0, 0, "Debug menu");
  lcd_print(0, 1, "not supported!");
}

void init_menu_dtc()
{
  switch (menu_dtc_screen)
  {
  case 0:
    lcd_print(0, 0, "DTC menu addr ");
    lcd_print(0, 1, "<");
    lcd_print(5, 1, "Read");
    lcd_print(15, 1, ">");
    break;
  case 1:
    lcd_print(0, 0, "DTC menu addr ");
    lcd_print(0, 1, "<");
    lcd_print(5, 1, "Clear");
    lcd_print(15, 1, ">");
    break;
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
    // "X/ XXXXXX St:   "
    // "/X XXXXXX St:   "
    lcd_print(1, 0, "/");
    lcd_print(10, 0, "St:");
    lcd_print(0, 1, "/8");
    lcd_print(10, 1, "St:");
    break;
  default:

    break;
  }
}
void init_menu_settings()
{
  switch (menu_settings_screen)
  {
  case 0:
    // Exit
    lcd_print(0, 0, "Exit ECU:");
    lcd_print(0, 1, "< Press select >");
    break;
  case 1:
    lcd_print(0, 0, "KWP Mode:");
    lcd_print(0, 1, "<");
    lcd_print(15, 1, ">");
    break;
  default:
    lcd_show_screen_not_supported(menu_settings_screen);
    break;
  }
}
void display_menu_cockpit(bool force_update = false)
{
  switch (addr_selected)
  {
  case ADDR_ENGINE:
    lcd_print(0, 0, voltage);
    lcd_print(0, 1, tb_angle);
    switch (menu_cockpit_screen)
    {
    case 0:
      lcd_print_cockpit_float(0, 0, voltage, 7, voltage_updated, force_update);
      lcd_print_cockpit_float(0, 1, tb_angle, 7, tb_angle_updated, force_update);
      break;
    case 1:
      lcd_print_cockpit(0, 0, engine_load, 7, engine_load_updated, force_update);
      lcd_print_cockpit_float(0, 1, steering_angle, 7, steering_angle_updated, force_update);
      break;
    case 2:
      if (error_bits_updated || force_update)
      {
        bits_as_string[0] = convert_bool_char(exhaust_gas_recirculation_error);
        bits_as_string[1] = convert_bool_char(oxygen_sensor_heating_error);
        bits_as_string[2] = convert_bool_char(oxgen_sensor_error);
        bits_as_string[3] = convert_bool_char(air_conditioning_error);
        bits_as_string[4] = convert_bool_char(secondary_air_injection_error);
        bits_as_string[5] = convert_bool_char(evaporative_emissions_error);
        bits_as_string[6] = convert_bool_char(catalyst_heating_error);
        bits_as_string[7] = convert_bool_char(catalytic_converter);
      }

      lcd_print_cockpit(0, 0, bits_as_string, 7, error_bits_updated, force_update);
      lcd_print_cockpit(0, 1, lambda_2, 7, lambda_2_updated, force_update);
      break;
    case 3:
      lcd_print_cockpit(0, 0, vehicle_speed, 7, vehicle_speed_updated, force_update);
      lcd_print_cockpit(0, 1, pressure, 7, pressure_updated, force_update);
      break;
    case 4:
      lcd_print_cockpit(0, 0, temperature_unknown_2, 4, temperature_unknown_2_updated, force_update);
      lcd_print_cockpit(0, 1, temperature_unknown_3, 4, temperature_unknown_3_updated, force_update);
      break;
    default:
      lcd_show_screen_not_supported(menu_cockpit_screen);
      break;
    }
    break;
  /*case ADDR_ABS_BRAKES:
    break;
  case ADDR_AUTO_HVAC:
    break;*/
  case ADDR_INSTRUMENTS:
    switch (menu_cockpit_screen)
    {
    case 0:
      lcd_print_cockpit(0, 0, vehicle_speed, 3, vehicle_speed_updated, force_update);
      lcd_print_cockpit(8, 0, engine_rpm, 4, engine_rpm_updated, force_update);
      lcd_print_cockpit(0, 1, coolant_temp, 3, coolant_temp_updated, force_update);
      lcd_print_cockpit(5, 1, oil_temp, 3, oil_temp_updated, force_update);
      lcd_print_cockpit(10, 1, fuel_level, 2, fuel_level_updated, force_update);
      break;
    case 1:
      lcd_print_cockpit(0, 0, oil_level_ok, 1, oil_level_ok_updated, force_update);
      lcd_print_cockpit(5, 0, oil_pressure_min, 1, oil_pressure_min_updated, force_update);
      lcd_print_cockpit(10, 0, ambient_temp, 2, ambient_temp_updated, force_update);
      lcd_print_cockpit(0, 1, odometer, 6, odometer_updated, force_update);
      lcd_print_cockpit(9, 1, fuel_sensor_resistance, 3, fuel_sensor_resistance_updated, force_update);
      break;
    case 2:
      lcd_print_cockpit(0, 0, time_ecu, 5, time_ecu_updated, force_update);
      lcd_print_cockpit(0, 1, fuel_per_100km, 6, fuel_per_100km_updated, force_update);
      break;
    case 3:
      lcd_print_cockpit(0, 0, elapsed_seconds_since_start, 8, elapsed_seconds_since_start_updated, force_update);
      lcd_print_cockpit(0, 1, elpased_km_since_start, 5, elpased_km_since_start_updated, force_update);
      break;
    case 4:
      lcd_print_cockpit(0, 0, fuel_burned_since_start, 5, fuel_burned_since_start_updated, force_update);
      lcd_print_cockpit(0, 1, fuel_per_hour, 6, fuel_per_hour_updated, force_update);
      break;
    default:
      break;
    }
    break;
  case ADDR_CENTRAL_CONV:
  default:
    // Addr not supported
    break;
  }
}
void display_menu_experimental(bool force_update = false)
{
  lcd_print_cockpit(2, 0, group_current, 2, group_current_updated, force_update);
  lcd_print_cockpit(2, 1, (uint8_t)group_side, 2, group_side_updated, force_update);
  uint8_t temp_first_pointer = 0;
  uint8_t temp_second_pointer = 1;
  if (group_side)
  {
    temp_first_pointer = 2;
    temp_second_pointer = 3;
  }

  lcd_print_cockpit_float(5, 0, v_temp[temp_first_pointer], 5, k_temp_updated, force_update);
  lcd_print_cockpit_float(5, 1, v_temp[temp_second_pointer], 5, k_temp_updated, force_update);
  lcd_print_cockpit(11, 0, unit_temp[temp_first_pointer], 5, unit_temp_updated, force_update);
  lcd_print_cockpit(11, 1, unit_temp[temp_second_pointer], 5, unit_temp_updated, force_update);
}
void display_menu_debug()
{
}
void display_menu_dtc(bool force_update = false)
{
  switch (menu_dtc_screen)
  {
  case 0:
  case 1:
    break;
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
    uint8_t dtc_pointer = menu_dtc_screen - 2;
    if (dtc_pointer < 0 || dtc_pointer > 7)
      return;
    lcd_print_cockpit(0, 0, dtc_pointer + 1, 1, dtc_errors_updated, force_update);
    lcd_print_cockpit(3, 0, dtc_errors[dtc_pointer], 6, dtc_errors_updated, force_update);
    lcd_print_cockpit(13, 0, dtc_status_bytes[dtc_pointer], 3, dtc_status_bytes_updated, force_update);
    lcd_print_cockpit(3, 1, dtc_errors[dtc_pointer + 1], 6, dtc_errors_updated, force_update);
    lcd_print_cockpit(13, 1, dtc_status_bytes[dtc_pointer + 1], 3, dtc_status_bytes_updated, force_update);
    break;
  }
}
void display_menu_settings(bool force_update = false)
{
  switch (menu_settings_screen)
  {
  case 0:
    // Exit
    break;
  case 1:
    // KWP mode
    if (kwp_mode != kwp_mode_last || force_update)
    {
      lcd_clear(4, 1, 7);
      lcd.setCursor(4, 1);
      switch (kwp_mode)
      {
      case KWP_MODE_ACK:
        lcd.print("ACK");
        break;
      case KWP_MODE_READGROUP:
        lcd.print("GROUP");
        break;
      case KWP_MODE_READSENSORS:
        lcd.print("SENSOR");
        break;
      }
      kwp_mode_last = kwp_mode;
    }
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
  case 10:
  default:
    break;
  }
}

bool engine_rpm_switch = true;
bool vehicle_speed_switch = true;
bool coolant_temp_switch = true;
bool oil_temp_switch = true;
bool oil_level_ok_switch = true;
bool fuel_level_switch = true;
void simulate_values_helper(uint8_t &val, uint8_t amount_to_change, bool &val_switch, bool &val_updated, uint8_t maximum, uint8_t minimum = 0)
{
  if (val_switch)
    val += amount_to_change;
  else
    val -= amount_to_change;

  val_updated = true;

  if (val_switch && val >= maximum)
    val_switch = false;
  else if (!val_switch && val <= minimum)
    val_switch = true;
}
void simulate_values_helper(uint16_t &val, uint8_t amount_to_change, bool &val_switch, bool &val_updated, uint16_t maximum, uint16_t minimum = 0)
{
  if (val_switch)
    val += amount_to_change;
  else
    val -= amount_to_change;

  val_updated = true;

  if (val_switch && val >= maximum)
    val_switch = false;
  else if (!val_switch && val <= minimum)
    val_switch = true;
}
void simulate_values()
{
  increase_block_counter();                                                                   // Simulate some values
  simulate_values_helper(vehicle_speed, 1, vehicle_speed_switch, vehicle_speed_updated, 200); // Vehicle speed
  simulate_values_helper(engine_rpm, 87, engine_rpm_switch, engine_rpm_updated, 7100);        // Engine RPM
  simulate_values_helper(coolant_temp, 1, coolant_temp_switch, coolant_temp_updated, 160);    // Coolant temperature
  simulate_values_helper(oil_temp, 1, oil_temp_switch, oil_temp_updated, 160);                // Oil Temperature
  simulate_values_helper(oil_level_ok, 1, oil_level_ok_switch, oil_level_ok_updated, 8);      // Oil level ok
  simulate_values_helper(fuel_level, 1, fuel_level_switch, fuel_level_updated, 57);           // Fuel
}
void compute_values()
{
  elapsed_seconds_since_start = ((millis() - connect_time_start) / 1000);
  elapsed_seconds_since_start_updated = true;
  elpased_km_since_start = odometer - odometer_start;
  elpased_km_since_start_updated = true;
  fuel_burned_since_start = abs(fuel_level_start - fuel_level);
  fuel_burned_since_start_updated = true;
  fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
  fuel_per_100km_updated = true;
  fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;
  fuel_per_hour_updated = true;
}

// --------------------------------------------------------------------------------------------------
//                            SERIAL MONITOR DEBUG CODE
// --------------------------------------------------------------------------------------------------

void serial_print_kwp_handshake_error(char response[3])
{
  debug(F("Handshake error expected ["));
  debughex(0x55);
  debug(F(" "));
  debughex(0x01);
  debug(F(" "));
  debughex(0x8A);
  debug(F("] got ["));
  debughex((uint8_t)response[0]);
  debug(F(" "));
  debughex((uint8_t)response[1]);
  debug(F(" "));
  debughex((uint8_t)response[2]);
  debugln(F("]"));
}

void serial_print_disconnected()
{
  debug(F("Disconnected. Block counter: "));
  debug(block_counter);
  debug(F(". Connected: "));
  debug(connected);
  debug(F(". Available: "));
  debugln(obd.available());
}

// --------------------------------------------------------------------------------------------------
//                            OBD CODE
// --------------------------------------------------------------------------------------------------

void disconnect()
{
  serial_print_disconnected();

  obd.end();
  reset_variables();
  delay(2222);
  //  TODO communication end procedure
}

/**
 * @brief Get number from OBD available, I still dont fully know what this means
 */
int available()
{
  return obd.available();
}

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void obdWrite(uint8_t data)
{
  debug(F("-MCU: "));
  debughexln(data);

  uint8_t to_delay = 5;
  switch (baud_rate)
  {
  case 1200:
    to_delay = 130;
    break;
  case 2400:
    to_delay = 60;
    break;
  case 4800:
    to_delay = 30;
    break;
  case 9600:
    to_delay = 10;
    break;
  }

  delay(to_delay);
  obd.write(data);
}

/**
 * @brief Read OBD input from ECU
 *
 * @return uint8_t The incoming byte or -1 if timeout
 */
uint8_t obdRead()
{
  unsigned long timeout = millis() + timeout_to_add;
  while (!obd.available())
  {
    if (millis() >= timeout)
    {
      debugln(F("ERROR: obdRead() timeout obd.available() = 0."));
      return -1;
    }
  }
  uint8_t data = obd.read();
  debug(F("ECU: "));
  debughexln(data);

  return data;
}

/**
 * @brief Perform a 5 Baud init sequence to wake up the ECU
 * 5Bd, 7O1
 * @param data Which ECU Address to wake up.
 */
void send5baud(uint8_t data)
{
  // // 1 start bit, 7 data bits, 1 parity, 1 stop bit
#define bitcount 10
  uint8_t bits[bitcount];
  uint8_t even = 1;
  uint8_t bit;
  for (uint8_t i = 0; i < bitcount; i++)
  {
    bit = 0;
    if (i == 0)
      bit = 0;
    else if (i == 8)
      bit = even; // computes parity bit
    else if (i == 9)
      bit = 1;
    else
    {
      bit = (byte)((data & (1 << (i - 1))) != 0);
      even = even ^ bit;
    }

    debugstrnum(F(" "), bit);

    bits[i] = bit;
  }
  // now send bit stream
  for (uint8_t i = 0; i < bitcount + 1; i++)
  {
    if (i != 0)
    {
      // wait 200 ms (=5 baud), adjusted by latency correction
      delay(200);
      if (i == bitcount)
        break;
    }
    if (bits[i] == 1)
    {
      // high
      digitalWrite(PIN_TX, HIGH);
    }
    else
    {
      // low
      digitalWrite(PIN_TX, LOW);
    }
  }
  obd.flush();
}

/**
 * Helper method to send the addr as 5 baud to the ECU
 */
bool KWP5BaudInit(uint8_t addr)
{
  debug(F("5 baud: (0)"));
  send5baud(addr);
  debugln(F(" (9) END"));
  return true;
}

/**
 * @brief Send a request to the ECU
 *
 * @param s Array where the data is stored
 * @param size The size of the request
 * @return true If no errors occured, will resume
 * @return false If errors occured, will disconnect
 */
bool KWPSendBlock(char *s, int size)
{
  debug(F("---KWPSend size = "));
  debug(size);
  debug(F(" block counter = "));
  debugln(block_counter);
  debug(F("To send: "));
  for (uint8_t i = 0; i < size; i++)
  {
    uint8_t data = s[i];
    debughex(data);
    debug(" ");
  }
  debugln();

  for (uint8_t i = 0; i < size; i++)
  {
    uint8_t data = s[i];
    obdWrite(data);
    /*uint8_t echo = obdRead(); ???
    if (data != echo){
      debugln(F("ERROR: invalid echo"));
      disconnect();
      errorData++;
      return false;
    }*/
    if (i < size - 1)
    {
      uint8_t complement = obdRead();
      if (complement != (data ^ 0xFF))
      {
        debugln(F("ERROR: invalid complement"));
        lcd_print(0, 1, "ERR: INV COMPL");
        delay(1333);
        lcd_print(0, 1, "MCU:", 16);
        lcd_print(5, 1, String(data, HEX));
        lcd_print(8, 1, "ECU:");
        lcd_print(13, 1, String(complement, HEX));
        delay(2000);
        return false;
      }
    }
  }
  increase_block_counter();
  return true;
}

/**
 * @brief The default way to keep the ECU awake is to send an Acknowledge Block.
 * Alternatives include Group Readings..
 *
 * @return true No errors
 * @return false Errors, disconnect
 */
bool KWPSendAckBlock()
{
  debug(F("---KWPSendAckBlock block counter = "));
  debugln(block_counter);
  char buf[32];
  sprintf(buf, "\x03%c\x09\x03", block_counter);
  if (!KWPSendBlock(buf, 4))
    return false;
  return true;
}

bool KWPSendDTCReadBlock()
{
  debug(F("---KWPSendDTCReadBlock block counter = "));
  debugln(block_counter);

  char s[32];
  sprintf(s, "\x03%c\x07\x03", block_counter);
  if (!KWPSendBlock(s, 4))
    return false;
  return true;
}

bool KWPSendDTCDeleteBlock()
{
  debug(F("---KWPSendDTCDeleteBlock block counter = "));
  debugln(block_counter);

  char s[32];
  sprintf(s, "\x03%c\x05\x03", block_counter);
  if (!KWPSendBlock(s, 4))
    return false;
  return true;
}

// count: if zero given, first received byte contains block length
// 4800, 9600 oder 10400 Baud, 8N1
// source:
// -1 = default | 1 = readsensors
/**
 * @brief Recieve a response from the ECU
 *
 * @param s Array with stored response data
 * @param maxsize Max size of response to be expected
 * @param size Size of response
 * @param source 1 if this method is called from GroupMeasurements readsensors(group) to determine com errors
 * @return true No errors
 * @return false Errors
 */
bool KWPReceiveBlock(char s[], int maxsize, int &size, int source = -1, bool initialization_phase = false)
{
  bool ackeachbyte = false;
  uint8_t data = 0;
  uint8_t recvcount = 0;
  if (size == 0)
    ackeachbyte = true;

  debugstrnum(F(" - KWPReceiveBlock. Size: "), size);
  debugstrnum(F(". Block counter: "), block_counter);

  if (size > maxsize)
  {
    debugln(F(" - KWPReceiveBlock error: Invalid maxsize"));
    lcd_print(0, 1, "ERR:size>maxsize");
    delay(2000);
    return false;
  }
  uint32_t timeout = millis() + timeout_to_add;
  uint16_t temp_iteration_counter = 0;
  while ((recvcount == 0) || (recvcount != size))
  {
    while (obd.available())
    {
      if (temp_iteration_counter == recvcount)
      {
        debug(F("      Iter: "));
        debug(temp_iteration_counter);
        debug(F(" receivecount: "));
        debugln(recvcount);
      }
      data = obdRead();
      if (data == -1)
      {
        debugln(F(" - KWPReceiveBlock error: (Available=0) or empty buffer!"));
        lcd_print(0, 1, "ERROR data = -1 ");
        delay(1700);
        return false;
      }
      s[recvcount] = data;
      recvcount++;
      if ((size == 0) && (recvcount == 1))
      {
        if (source == 1 && (data != 15 || data != 3) && obd.available())
        {
          lcd_print(0, 1, "WARN block length");
          debugln(F(" - KWPReceiveBlock warn: Communication error occured, check block length!"));
          com_error = true;
          size = 6;
        }
        else
        {
          size = data + 1;
        }
        if (size > maxsize)
        {

          debugln(F(" - KWPReceiveBlock error: Invalid maxsize"));
          lcd_print(0, 1, "ERR:size>maxsize");
          delay(2000);
          return false;
        }
      }
      if (com_error)
      {
        if (recvcount == 1)
        {
          ackeachbyte = false;
        }
        else if (recvcount == 3)
        {
          ackeachbyte = true;
        }
        else if (recvcount == 4)
        {
          ackeachbyte = false;
        }
        else if (recvcount == 6)
        {
          ackeachbyte = true;
        }
        continue;
      }
      if ((ackeachbyte) && (recvcount == 2))
      {
        if (data != block_counter)
        {
          if (initialization_phase)
          {
            lcd_print(0, 1, "ERR:size>maxsize");
            delay(1000);
            lcd_print(0, 1, "Exp:" + String(data) + " Is:" + String(block_counter) + "         ");
            delay(3333);
          }
          debug(F(" - KWPReceiveBlock error: Invalid block counter. Expected: "));
          Serial.print((uint8_t)data);
          debug(F(". Is: "));
          debugln((uint8_t)block_counter);
          return false;
        }
      }
      if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
      {
        obdWrite(data ^ 0xFF); // send complement ack
        // delay(25);
        // uint8_t echo = obdRead();
        // if (echo != (data ^ 0xFF))
        //{
        //   if (debug_mode_enabled)
        //   {
        //     debug(F("ERROR: invalid echo "));
        //     debughexln(echo);
        //   }
        //  errorData++;
        //  If ECHO is wrong just keep going
        //  return false;
        //}
      }
      timeout = millis() + timeout_to_add;

      debug(F(" - KWPReceiveBlock: Added timeout. ReceiveCount: "));
      debug((uint8_t)recvcount);
      debug(F(". Processed data: "));
      debughex((uint8_t)data);
      debug(F(". ACK compl: "));
      debugln(((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)));
    }

    if (millis() >= timeout)
    {
      debug(F(" - KWPReceiveBlock: Timeout overstepped on iteration "));
      debug(temp_iteration_counter);
      debug(F(" with receivecount "));
      debugln(recvcount);

      if (recvcount == 0)
      {
        debugln(F("No connection to ECU! Check wiring."));
      }

      if (recvcount == 0)
      {
        lcd_print(0, 1, "Nothing received!   ");
        delay(1222);
        lcd_print(0, 1, "Check wiring!    ");
      }
      else
      {
        lcd_print(0, 1, "Timeout");
      }
      delay(1222);
      // errorTimeout++;
      return false;
    }
    temp_iteration_counter++;
  }
  // show data
  debug(F("IN: size = "));
  debug(size);
  debug(F(" data = "));
  for (uint8_t i = 0; i < size; i++)
  {
    uint8_t data = s[i];
    debughex(data);
    debug(F(" "));
  }
  debugln();

  increase_block_counter();
  return true;
}

bool KWPErrorBlock()
{
  // Kommunikationsfehler
  char s[64];
  s[0] = 0x03;
  s[1] = block_counter;
  s[2] = 0x00;
  s[3] = 0x03;
  if (!KWPSendBlock(s, 4))
  {
    com_error = false;
    return false;
  }
  block_counter = 0;
  com_error = false;
  int size2 = 0;
  if (!KWPReceiveBlock(s, 64, size2))
  {
    return false;
  }
  return true;
}

bool KWPReceiveAckBlock()
{
  // --------- Expect response acknowledge ----------------
  char buf[32];
  int size2 = 0;
  if (!KWPReceiveBlock(buf, 4, size2))
  {
    return false;
  }
  if (buf[0] != 0x03 || buf[2] != 0x09 || buf[3] != 0x03)
  {
    debug(F(" - Error receiving ACK procedure got s[0]-s[3]: "));
    debughex(buf[0]);
    debug(F(" "));
    debughex(buf[1]);
    debug(F(" "));
    debughex(buf[2]);
    debug(F(" "));
    debughex(buf[3]);
    debugln(F(" should be 03 BC 09 03"));
    return false;
  }
  if (com_error)
  {
    KWPErrorBlock();
    return false;
  }
  return true;
}

/**
 * @brief Last step in initial ECU startup sequence.
 *
 * @return true no errors
 * @return false errors
 */
bool readConnectBlocks(bool initialization_phase = false)
{
  debugln(F(" - Readconnectblocks"));

  String info;
  while (true)
  {
    int size = 0;
    char s[64];
    if (!(KWPReceiveBlock(s, 64, size)))
      return false;
    if (size == 0)
      return false;
    if (s[2] == '\x09')
      break;
    if (s[2] != '\xF6')
    {
      debugln(F(" - Readconnectblocks ERROR: unexpected answer"));

      lcd_print(0, 1, "ERR: s[2]!=xF6");
      delay(2000);
      return false;
    }
    String text = String(s);
    info += text.substring(3, size - 2);
    if (!KWPSendAckBlock())
      return false;
  }
  debugstrnum(F("label = "), info);
  return true;
}

/**
 * @brief Perform a measurement group reading and safe the value to the corresponding variable.
 * Each group contains 4 values. Refer to your Label File for further information and order.
 * @param group The group to read
 * @return true no errors
 * @return false errors
 */
bool readSensors(int group)
{
  debug(F(" - ReadSensors group "));
  debugln(group);

  for (int i = 0; i <= 3; i++)
  {
    k_temp[i] = 0;
    v_temp[i] = -1;
    unit_temp[i] = "ERR";
  }

  char s[64];
  sprintf(s, "\x04%c\x29%c\x03", block_counter, group);
  if (!KWPSendBlock(s, 5))
    return false;
  int size = 0;
  if (!KWPReceiveBlock(s, 64, size, 1))
  {
    return false;
  }
  if (com_error)
  {
    // Kommunikationsfehler
    char s[64];
    s[0] = 0x03;
    s[1] = block_counter;
    s[2] = 0x00;
    s[3] = 0x03;
    if (!KWPSendBlock(s, 4))
    {
      com_error = false;
      return false;
    }
    block_counter = 0;
    com_error = false;
    int size2 = 0;
    if (!KWPReceiveBlock(s, 64, size2))
    {
      return false;
    }
  }
  if (s[2] != '\xe7')
  {
    debugln(F("ERROR: invalid answer"));
    lcd_print(0, 1, "ERR: s[2]!=xE7");
    delay(2000);
    // errorData++;
    return false;
  }
  int count = (size - 4) / 3;
  debug(F("count="));
  debugln(count);
  for (int idx = 0; idx < count; idx++)
  {
    byte k = s[3 + idx * 3];
    byte a = s[3 + idx * 3 + 1];
    byte b = s[3 + idx * 3 + 2];
    String n;
    float v = 0;

    debug(F("type="));
    debug(k);
    debug(F("  a="));
    debug(a);
    debug(F("  b="));
    debug(b);
    debug(F("  text="));

    String t = "";
    String units = "";
    char buf[32];
    switch (k)
    {
    case 1:
      v = 0.2 * a * b;
      units = F("rpm");
      break;
    case 2:
      v = a * 0.002 * b;
      units = F("%%");
      break;
    case 3:
      v = 0.002 * a * b;
      units = F("Deg");
      break;
    case 4:
      v = abs(b - 127) * 0.01 * a;
      units = F("ATDC");
      break;
    case 5:
      v = a * (b - 100) * 0.1;
      units = F("°C");
      break;
    case 6:
      v = 0.001 * a * b;
      units = F("V");
      break;
    case 7:
      v = 0.01 * a * b;
      units = F("km/h");
      break;
    case 8:
      v = 0.1 * a * b;
      units = F(" ");
      break;
    case 9:
      v = (b - 127) * 0.02 * a;
      units = F("Deg");
      break;
    case 10:
      if (b == 0)
        t = F("COLD");
      else
        t = F("WARM");
      break;
    case 11:
      v = 0.0001 * a * (b - 128) + 1;
      units = F(" ");
      break;
    case 12:
      v = 0.001 * a * b;
      units = F("Ohm");
      break;
    case 13:
      v = (b - 127) * 0.001 * a;
      units = F("mm");
      break;
    case 14:
      v = 0.005 * a * b;
      units = F("bar");
      break;
    case 15:
      v = 0.01 * a * b;
      units = F("ms");
      break;
    case 18:
      v = 0.04 * a * b;
      units = F("mbar");
      break;
    case 19:
      v = a * b * 0.01;
      units = F("l");
      break;
    case 20:
      v = a * (b - 128) / 128;
      units = F("%%");
      break;
    case 21:
      v = 0.001 * a * b;
      units = F("V");
      break;
    case 22:
      v = 0.001 * a * b;
      units = F("ms");
      break;
    case 23:
      v = b / 256 * a;
      units = F("%%");
      break;
    case 24:
      v = 0.001 * a * b;
      units = F("A");
      break;
    case 25:
      v = (b * 1.421) + (a / 182);
      units = F("g/s");
      break;
    case 26:
      v = float(b - a);
      units = F("C");
      break;
    case 27:
      v = abs(b - 128) * 0.01 * a;
      units = F("°");
      break;
    case 28:
      v = float(b - a);
      units = F(" ");
      break;
    case 30:
      v = b / 12 * a;
      units = F("Deg k/w");
      break;
    case 31:
      v = b / 2560 * a;
      units = F("°C");
      break;
    case 33:
      v = 100 * b / a;
      units = F("%%");
      break;
    case 34:
      v = (b - 128) * 0.01 * a;
      units = F("kW");
      break;
    case 35:
      v = 0.01 * a * b;
      units = F("l/h");
      break;
    case 36:
      v = ((unsigned long)a) * 2560 + ((unsigned long)b) * 10;
      units = F("km");
      break;
    case 37:
      v = b;
      break; // oil pressure ?!
    // ADP: FIXME!
    /*case 37: switch(b){
             case 0: sprintf(buf, F("ADP OK (%d,%d)"), a,b); t=String(buf); break;
             case 1: sprintf(buf, F("ADP RUN (%d,%d)"), a,b); t=String(buf); break;
             case 0x10: sprintf(buf, F("ADP ERR (%d,%d)"), a,b); t=String(buf); break;
             default: sprintf(buf, F("ADP (%d,%d)"), a,b); t=String(buf); break;
          }*/
    case 38:
      v = (b - 128) * 0.001 * a;
      units = F("Deg k/w");
      break;
    case 39:
      v = b / 256 * a;
      units = F("mg/h");
      break;
    case 40:
      v = b * 0.1 + (25.5 * a) - 400;
      units = F("A");
      break;
    case 41:
      v = b + a * 255;
      units = F("Ah");
      break;
    case 42:
      v = b * 0.1 + (25.5 * a) - 400;
      units = F("Kw");
      break;
    case 43:
      v = b * 0.1 + (25.5 * a);
      units = F("V");
      break;
    case 44:
      sprintf(buf, "%2d:%2d", a, b);
      t = String(buf);
      break;
    case 45:
      v = 0.1 * a * b / 100;
      units = F(" ");
      break;
    case 46:
      v = (a * b - 3200) * 0.0027;
      units = F("Deg k/w");
      break;
    case 47:
      v = (b - 128) * a;
      units = F("ms");
      break;
    case 48:
      v = b + a * 255;
      units = F(" ");
      break;
    case 49:
      v = (b / 4) * a * 0.1;
      units = F("mg/h");
      break;
    case 50:
      v = (b - 128) / (0.01 * a);
      units = F("mbar");
      break;
    case 51:
      v = ((b - 128) / 255) * a;
      units = F("mg/h");
      break;
    case 52:
      v = b * 0.02 * a - a;
      units = F("Nm");
      break;
    case 53:
      v = (b - 128) * 1.4222 + 0.006 * a;
      units = F("g/s");
      break;
    case 54:
      v = a * 256 + b;
      units = F("count");
      break;
    case 55:
      v = a * b / 200;
      units = F("s");
      break;
    case 56:
      v = a * 256 + b;
      units = F("WSC");
      break;
    case 57:
      v = a * 256 + b + 65536;
      units = F("WSC");
      break;
    case 59:
      v = (a * 256 + b) / 32768;
      units = F("g/s");
      break;
    case 60:
      v = (a * 256 + b) * 0.01;
      units = F("sec");
      break;
    case 62:
      v = 0.256 * a * b;
      units = F("S");
      break;
    case 64:
      v = float(a + b);
      units = F("Ohm");
      break;
    case 65:
      v = 0.01 * a * (b - 127);
      units = F("mm");
      break;
    case 66:
      v = (a * b) / 511.12;
      units = F("V");
      break;
    case 67:
      v = (640 * a) + b * 2.5;
      units = F("Deg");
      break;
    case 68:
      v = (256 * a + b) / 7.365;
      units = F("deg/s");
      break;
    case 69:
      v = (256 * a + b) * 0.3254;
      units = F("Bar");
      break;
    case 70:
      v = (256 * a + b) * 0.192;
      units = F("m/s^2");
      break;
    default:
      sprintf(buf, "%2x, %2x      ", a, b);
      break;
    }

    /*
     * Update k_temp and v_temp and unit_temp
     */
    if (k_temp[idx] != k)
    {
      k_temp[idx] = k;
      k_temp_updated = true;
    }
    if (v_temp[idx] != v)
    {
      v_temp[idx] = v;
      v_temp_updated = true;
    }
    if (unit_temp[idx] != units)
    {
      unit_temp[idx] = units;
      unit_temp_updated = true;
    }

    /*
     *  Add here the values from your label file for each address.
     */
    switch (addr_selected)
    {
    case ADDR_INSTRUMENTS:
      switch (group)
      {
      case 1:
        switch (idx)
        {
        case 0:
          // 0.0 km/h Speed
          if (vehicle_speed != (uint16_t)v)
          {
            vehicle_speed = (uint16_t)v;
            vehicle_speed_updated = true;
          }
          break;
        case 1:
          // 0 /min Engine Speed
          if (engine_rpm != (uint16_t)v)
          {
            engine_rpm = (uint16_t)v;
            engine_rpm_updated = true;
          }
          break;
        case 2:
          // Oil Pr. 2 < min (Oil pressure 0.9 bar)
          if (oil_pressure_min != (uint16_t)v)
          {
            oil_pressure_min = (uint16_t)v;
            oil_pressure_min_updated = true;
          }
          break;
        case 3:
          // 21:50 Time
          if (time_ecu != (uint32_t)v)
          {
            time_ecu = (uint32_t)v;
            time_ecu_updated = true;
          }
          break;
        }
        break;
      case 2:
        switch (idx)
        {
        case 0:
          // 101010 Odometer
          if (odometer != (uint32_t)v)
          {
            odometer = (uint32_t)v;
            odometer_updated = true;
          }
          if (millis() - connect_time_start < 10000)
          {
            odometer_start = odometer;
          }
          break;
        case 1:
          // 9 l Fuel level
          if (fuel_level != (uint16_t)v)
          {
            fuel_level = (uint16_t)v;
            fuel_level_updated = true;
          }
          if (millis() - connect_time_start < 10000)
          {
            fuel_level_start = fuel_level;
          }
          break;
        case 2:
          // 93 ohms Fuel Sender Resistance
          if (fuel_sensor_resistance != (uint16_t)v)
          {
            fuel_sensor_resistance = (uint16_t)v;
            fuel_sensor_resistance_updated = true;
          }
          break;
        case 3:
          // 0°C Ambient Temperature
          if (ambient_temp != (uint8_t)v)
          {
            ambient_temp = (uint8_t)v;
            ambient_temp_updated = true;
          }
          break;
        }
        break;
      case 3:
        switch (idx)
        {
        case 0:
          // 12.0°C Coolant temp.
          if (coolant_temp != (uint8_t)v)
          {
            coolant_temp = (uint8_t)v;
            coolant_temp_updated = true;
          }
          break;
        case 1:
          // OK Oil Level (OK/n.OK)
          if (oil_level_ok != (uint8_t)v)
          {
            oil_level_ok = (uint8_t)v;
            oil_level_ok_updated = true;
          }
          break;
        case 2:
          // 11.0°C Oil temp
          if (oil_temp != (uint8_t)v)
          {
            oil_temp = (uint8_t)v;
            oil_temp_updated = true;
          }
          break;
        case 3:
          // N/A
          break;
        }
        break;
      }
      break;
    case ADDR_ENGINE:
      switch (group)
      {
      case 1:
        switch (idx)
        {
        case 0:
          // /min RPM
          if (engine_rpm != (uint16_t)v)
          {
            engine_rpm = (uint16_t)v;
            engine_rpm_updated = true;
          }
          break;
        case 1:
          // 0 /min Engine Speed
          if (temperature_unknown_1 != (uint8_t)v)
          {
            temperature_unknown_1 = (uint8_t)v;
            temperature_unknown_1_updated = true;
          }
          break;
        case 2:
          // Oil Pr. 2 < min (Oil pressure 0.9 bar)

          if (lambda != (int8_t)v)
          {
            lambda = (int8_t)v;
            lambda_updated = true;
          }
          break;
        case 3:
          // 21:50 Time
          String binary_bits_string = String(v);
          for (int i = 0; i < 8; i++)
          {
            if (binary_bits_string.charAt(i) == '1')
            {
              switch (i)
              {
              case 0:
                exhaust_gas_recirculation_error = true;
                break;
              case 1:
                oxygen_sensor_heating_error = true;
                break;
              case 2:
                oxgen_sensor_error = true;
                break;
              case 3:
                air_conditioning_error = true;
                break;
              case 4:
                secondary_air_injection_error = true;
                break;
              case 5:
                evaporative_emissions_error = true;
                break;
              case 6:
                catalyst_heating_error = true;
                break;
              case 7:
                catalytic_converter = true;
                break;
              }
            }
          }
          break;
        }
        break;
      case 3:
        switch (idx)
        {
        case 0:
          break;
        case 1:
          // 9.0 l Fuel level

          if (pressure != (uint16_t)v)
          {
            pressure = (uint16_t)v;
            pressure_updated = true;
          }
          break;
        case 2:
          // 93 ohms Fuel Sender Resistance
          if (tb_angle != (float)v)
          {
            tb_angle = (float)v;
            tb_angle_updated = true;
          }
          break;
        case 3:
          // 0.0°C Ambient Temperature
          if (steering_angle != (float)v)
          {
            steering_angle = (float)v;
            steering_angle_updated = true;
          }
          break;
        }
        break;
      case 4:
        switch (idx)
        {
        case 0:
          break;
        case 1:
          if (voltage != (float)v)
          {
            voltage = (float)v;
            voltage_updated = true;
          }
          break;
        case 2:
          if (temperature_unknown_2 != (uint8_t)v)
          {
            temperature_unknown_2 = (uint8_t)v;
            temperature_unknown_2_updated = true;
          }
          break;
        case 3:
          if (temperature_unknown_3 != (uint8_t)v)
          {
            temperature_unknown_3 = (uint8_t)v;
            temperature_unknown_3_updated = true;
          }
          break;
        }
        break;
      case 6:
        switch (idx)
        {
        case 0:
          break;
        case 1:
          if (engine_load != (uint16_t)v)
          {
            engine_load = (uint16_t)v;
            engine_load_updated = true;
          }
          break;
        case 2:
          break;
        case 3:
          if (lambda_2 != (int8_t)v)
          {
            lambda_2 = (int8_t)v;
            lambda_2_updated = true;
          }
          break;
        }
        break;
      }
      break;
    }
  }
  /*if (units.length() != 0) {
      dtostrf(v, 4, 2, buf);
      t = String(buf) + " " + units;
    }
//    debugln(t);

    //lcd.setCursor(0, idx);
    //while (t.length() < 20) t += " ";
    //lcd.print(t);
  }
  sensorCounter++;*/
  return true;
}

/**
 * KW1281 procedure to send a simple acknowledge block to keep the connection alive
 */
bool send_ack(bool expect_response_ack = true)
{
  if (!KWPSendAckBlock())
    return false;

  if (!expect_response_ack)
    return true;

  if (!KWPReceiveAckBlock())
    return false;

  return true;
}

/**
 * KW1281 procedure to read DTC error codes
 *
 * @return 0=false, 1=true, 2=true_no_errors_found
 */
uint8_t read_DTC_codes()
{
  debug(F("Read DTC on ADDR 0x"));
  debughexln(addr_selected);

  if (!KWPSendDTCReadBlock())
    return false;

  reset_dtc_status_errors_array();
  bool all_dtc_errors_received = false;
  uint8_t dtc_errors_received_counter = 0;
  char s[64];
  int size = 0;
  while (!all_dtc_errors_received)
  {
    if (!KWPReceiveBlock(s, 64, size, 1))
    {
      return false;
    }
    if (com_error)
    {
      // Kommunikationsfehler
      KWPErrorBlock();
      return false;
    }

    if (s[2] == 0xFC)
    {
      // Read next DTC error block
      if (s[3] == 0xFF && s[4] == 0xFF && s[5] == 0x88)
      {
        // No DTC errors found
        debugln(F("No DTC codes found"));
        return 2;
      }
      else
      {
        // Extract DTC errors
        uint8_t block_length = s[0];
        uint8_t dtc_error_amount = (uint8_t)((block_length - 3) / 3);
        if (dtc_error_amount < 1 || dtc_error_amount > 4)
        {
          debugln(F("DTC wrong amount of DTC errors"));
          return false;
        }
        for (int i = 0; i < dtc_error_amount; i++)
        {
          uint8_t byte_high = s[3 + 3 * i];
          uint8_t byte_low = s[3 + 3 * i + 1];
          uint8_t byte_status = s[3 + 3 * i + 2];
          dtc_errors[i + dtc_errors_received_counter * 4] = byte_low | (byte_high << 8);
          dtc_status_bytes[i + dtc_errors_received_counter * 4] = byte_status;
        }
        debug(F("DTC errors: "));
        for (int i = 0; i < dtc_error_amount; i++)
        {
          debug(i);
          debug(F(" = "));
          debughex(dtc_errors[i]);
          debug(F(" | "));
        }
        debugln(F(""));
        debug(F("DTC Status bytes: "));
        for (int i = 0; i < dtc_error_amount; i++)
        {
          debug(i);
          debug(F(" = "));
          debughex(dtc_status_bytes[i]);
          debug(F(" | "));
        }
        debugln(F(""));
        dtc_errors_received_counter++;
        if (dtc_errors_received_counter > 3)
        {
          debugln(F("Too much errors to receive. INCREASE ARRAY SIZE!"));
          all_dtc_errors_received = true;
          continue;
        }
      }
    }
    else if (s[2] == 0x09)
    {
      // No more DTC error blocks
      all_dtc_errors_received = true;
      continue;
    }
    else
    {
      debugln(F("DTC wrong block title"));
    }
  }

  if (!KWPSendAckBlock())
    return false;

  return true;
}

/**
 * KW1281 procedure to delete DTC error codes
 */
bool delete_DTC_codes()
{
  if (!KWPSendDTCDeleteBlock())
    return false;

  return true;
}

bool kwp_exit()
{
  lcd.clear();
  lcd_print(0, 0, "Exiting...");
  debugln(F("Manual KWP exit.."));
  // Perform KWP end output block
  delay(15);
  char s[64];
  sprintf(s, "\x03%c\x06\x03", block_counter);
  if (!KWPSendBlock(s, 4))
  {
    debugln(F("KWP exit failed"));
    lcd_print(0, 1, "error!");
    return false;
  }
  else
  {
    debugln(F("KWP exit succesful"));
    lcd_print(0, 1, "success!");
  }
  return true;
}

bool obd_connect()
{
  debugln(F("Connecting to ECU"));

  block_counter = 0;

  lcd.clear();
  init_statusbar();
  lcd_print(0, 1, "->PRESS ENTER<-");
  while (true)
  {
    display_statusbar();
    int user_input = analogRead(0);
    if (user_input >= 600 && user_input < 800)
      break;
  }

  obd.begin(baud_rate); // 9600 for 0x01, 10400 for other addresses, 1200 for very old ECU < 1996
  display_statusbar();
  debugln(F("Init "));
  lcd_print(0, 1, F("Init"), 16);
  KWP5BaudInit(addr_selected);
  display_statusbar();

  char response[3] = {0, 0, 0}; // Response should be (0x55, 0x01, 0x8A)base=16 = (85 1 138)base=2
  int response_size = 3;
  if (!KWPReceiveBlock(response, 3, response_size, -1, true) & ((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A)))
  {
    serial_print_kwp_handshake_error(response);
    lcd_print(0, 1, "ECU: " + String((uint8_t)response[0], HEX) + " " + String((uint8_t)response[1], HEX) + " " + String((uint8_t)response[2], HEX), 16);
    delay(ECU_TIMEOUT);
    return false;
  }
  display_statusbar();

  debugln(F("KWP5BaudInit DONE"));
  lcd_print(0, 1, "Read ECU data...");
  debugln(F("ReadConnectBlocks"));
  if (!readConnectBlocks(true))
  {
    display_statusbar();
    lcd_print(0, 1, "Read ECU data..N");
    return false;
  }
  debugln(F("ECU connected"));

  connected = true;
  display_statusbar();
  lcd_print(1, 1, F("ECU connected"), 16);
  display_statusbar();
  return true;
}

void connect_helper_bool_display(String first_line, String left_string, String right_string)
{
  lcd.clear();
  lcd_print(0, 0, first_line);
  lcd_print(0, 1, left_string);
  lcd_print(10, 1, right_string);
}
void connect_helper_bool(int8_t &userinput_value, uint8_t &config_value, uint8_t left_value, uint8_t right_value)
{
  while (userinput_value == -1)
  {
    int user_input = analogRead(0);
    if (BUTTON_RIGHT(user_input))
    {
      // Right button
      userinput_value = right_value;
    }
    else if (BUTTON_LEFT(user_input))
    {
      // Left button
      userinput_value = left_value;
    }
  }

  if (userinput_value == left_value)
  {
    lcd_clear(8, 1, 8);
    config_value = left_value;
  }
  else if (userinput_value == right_value)
  {
    lcd_clear(0, 1, 8);
    config_value = right_value;
  }
  delay(350);
}
/**
 * Perform user setup before connecting to ECU
 */
bool connect()
{
  debugstrnumln(F("Connect attempt: "), connection_attempts_counter);

  // Startup configuration // 0 = false, 1 = true, -1 = undefined for booleans as int8_t
  int8_t userinput_simulation_mode = -1;
  uint16_t userinput_baudrate = 9600;
  uint8_t userinput_baudrate_pointer = 3; // for default 9600
  uint16_t supported_baud_rates_size = 5;
  uint16_t supported_baud_rates[supported_baud_rates_size] = {1200, 2400, 4800, 9600, 10400};
  int8_t userinput_ecu_address = -1; // 1 or 17

  if (connection_attempts_counter > 0)
  {
    userinput_simulation_mode = simulation_mode_active;
  }

  connect_helper_bool_display("Connect mode", "<- ECU", "SIM ->");
  connect_helper_bool(userinput_simulation_mode, simulation_mode_active, 0, 1);

  lcd.clear();
  lcd_print(0, 0, "<--   Baud:  -->");
  lcd_print(2, 1, "-> " + String(userinput_baudrate), 10);
  bool pressed_enter = false;
  while (!pressed_enter)
  {
    int user_input = analogRead(0);
    if (BUTTON_RIGHT(user_input))
    {
      // Right button
      if (userinput_baudrate_pointer >= 4)
      {
        userinput_baudrate_pointer = 0;
      }
      else
        userinput_baudrate_pointer++;
      // userinput_baudrate_last = userinput_baudrate;
      userinput_baudrate = supported_baud_rates[userinput_baudrate_pointer];
      lcd_print(2, 1, "-> " + String(userinput_baudrate), 10);
      delay(333);
    }
    else if (BUTTON_LEFT(user_input))
    {
      // Left button
      if (userinput_baudrate_pointer <= 0)
      {
        userinput_baudrate_pointer = 4;
      }
      else
        userinput_baudrate_pointer--;
      // userinput_baudrate_last = userinput_baudrate;
      userinput_baudrate = supported_baud_rates[userinput_baudrate_pointer];
      lcd_print(2, 1, "-> " + String(userinput_baudrate), 10);
      delay(333);
    }
    else if (BUTTON_SELECT(user_input))
    {
      // Enter button
      pressed_enter = true;
    }
    delay(10);
  }
  baud_rate = userinput_baudrate;
  delay(555);

  connect_helper_bool_display("ECU address:", "<-- 01", "17 -->");
  connect_helper_bool(userinput_ecu_address, addr_selected, ADDR_ENGINE, ADDR_INSTRUMENTS);

  debugln(F("Saved configuration: "));
  debugstrnumln(F("--- DEBUG "), DEBUG);
  debugstrnumln(F("--- SIMULATION "), simulation_mode_active);
  debugstrnumln(F("--- baud "), baud_rate);
  debugstrnumhexln(F("--- addr "), addr_selected);

  // Connect to ECU
  connection_attempts_counter++;
  if (!simulation_mode_active && !obd_connect())
  {
    disconnect();
    return false;
  }
  connect_time_start = millis();
  menu_switch = true;
  return true;
}

// --------------------------------------------------------------------------------------------------
//                            ARDUINO CODE
// --------------------------------------------------------------------------------------------------

void setup()
{

// For debug USB Serial monitor
#if DEBUG == 1
  Serial.begin(9600);
#endif

  // Display
  lcd.begin(DISPLAY_MAX_X, DISPLAY_MAX_Y);

  // Pins
  pinMode(PIN_TX, OUTPUT); // TX
  digitalWrite(PIN_TX, HIGH);

  // Startup animation
  lcd.clear();
  lcd_print(0, 0, "O B D");
  lcd_print(1, 1, "D I S P L A Y");

  delay(777);
}

void loop()
{

  // Check connection
  if (!connected && !connect())
    return;

  // Update values
  if (!simulation_mode_active)
  {

    switch (kwp_mode)
    {
    case KWP_MODE_ACK:
      if (!send_ack())
      {
        disconnect();
        return;
      }
      break;
    case KWP_MODE_READGROUP:
      if (!readSensors(kwp_group))
      {
        disconnect();
        return;
      }
      break;
    case KWP_MODE_READSENSORS:
      // Read the sensor groups
      for (int i = 1; i <= 3; i++)
      {
        if (!readSensors(i))
        {
          disconnect();
          return;
        }
      }
      break;
    default:
      debugln(F("Kwp_mode undefined EXIT"));

      break;
    }
  }
  else
  {
    simulate_values();
    delay(222);
  }

  compute_values();

  // Button input
  bool button_pressed = false;
  if (millis() > endTime)
  {
    // User input, menu selection
    int user_input = analogRead(0);
    if (user_input < 60)
    {
      // Right button
      button_pressed = true;
      increment_menu(menu, menu_max);
    }
    else if (BUTTON_LEFT(user_input))
    {
      // Left button
      button_pressed = true;
      decrement_menu(menu, menu_max);
    }
    else
    {
      switch (menu)
      {
      case 0:
        if (BUTTON_UP(user_input))
        {
          button_pressed = true;
          increment_menu(menu_cockpit_screen, menu_cockpit_screen_max);
        }
        else if (BUTTON_DOWN(user_input))
        {
          button_pressed = true;
          decrement_menu(menu_cockpit_screen, menu_cockpit_screen_max);
        }
        break;
      case 1:
        if (BUTTON_UP(user_input))
        {
          button_pressed = true;
          increment_menu(menu_experimental_screen, menu_experimental_screen_max);
          increment_group_current();
        }
        else if (BUTTON_DOWN(user_input))
        {
          button_pressed = true;
          decrement_menu(menu_experimental_screen, menu_experimental_screen_max);
          decrement_group_current();
        }
        else if (BUTTON_SELECT(user_input))
        {
          button_pressed = true;
          invert_group_side();
        }
        break;
      case 2:
        if (BUTTON_UP(user_input))
        {
          button_pressed = true;
          increment_menu(menu_debug_screen, menu_debug_screen_max);
        }
        else if (BUTTON_DOWN(user_input))
        {
          button_pressed = true;
          decrement_menu(menu_debug_screen, menu_debug_screen_max);
        }
        break;
      case 3:
        if (BUTTON_UP(user_input))
        {
          button_pressed = true;
          increment_menu(menu_dtc_screen, menu_dtc_screen_max);
        }
        else if (BUTTON_DOWN(user_input))
        {
          // Down button
          button_pressed = true;
          decrement_menu(menu_dtc_screen, menu_dtc_screen_max);
        }
        else
        {
          switch (menu_dtc_screen)
          {
          case 0:
            if (BUTTON_SELECT(user_input))
            {
              // Select button = Exit/Reconnect
              delay(5);
              if (!read_DTC_codes())
              {
                lcd.clear();
                lcd_print(0, 0, "DTC read error");
                lcd_print(0, 1, "Disconnecting...");
                disconnect();
                return;
              }
            }
            break;
          case 1:
            if (BUTTON_SELECT(user_input))
            {
              delay(5);
              if (!delete_DTC_codes())
              {
                lcd.clear();
                lcd_print(0, 0, "DTC delete error");
                lcd_print(0, 1, "Disconnecting...");
                disconnect();
                return;
              }
            }
            break;
          default:

            break;
          }
        }
        break;
      case 4:
        if (BUTTON_UP(user_input))
        {
          // Up button
          button_pressed = true;
          increment_menu(menu_settings_screen, menu_settings_screen_max);
        }
        else if (BUTTON_DOWN(user_input))
        {
          // Down button
          button_pressed = true;
          decrement_menu(menu_settings_screen, menu_settings_screen_max);
        }
        else
        {
          switch (menu_settings_screen)
          {
          case 0:
            if (BUTTON_SELECT(user_input))
            {
              // Exit/Reconnect
              kwp_exit();
              disconnect();
              return;
            }
            break;
          case 1:
            if (BUTTON_SELECT(user_input))
            {
              switch (kwp_mode)
              {
              case KWP_MODE_ACK:
                kwp_mode = KWP_MODE_READGROUP;
                break;
              case KWP_MODE_READGROUP:
                kwp_mode = KWP_MODE_READSENSORS;
                break;
              case KWP_MODE_READSENSORS:
                kwp_mode = KWP_MODE_ACK;
                break;
              }

              button_pressed = true;
            }
            break;
          default:
            break;
          }
        }
        break;
      }
    }

    if (button_pressed)
    {
      endTime = millis() + button_timeout;
    }
  }

  // Perform menu switch or update values on current menu
  if (menu_switch || menu_screen_switch)
  {
    lcd.clear();
    switch (menu)
    {
    case 0:
      init_menu_cockpit();
      display_menu_cockpit(true);
      break;
    case 1:
      init_menu_experimental();
      display_menu_experimental(true);
      break;
    case 2:
      init_menu_debug();
      break;
    case 3:
      init_menu_dtc();
      display_menu_dtc(true);
      break;
    case 4:
      init_menu_settings();
      display_menu_settings(true);
      break;
    }
    menu_switch = false;
    menu_screen_switch = false;
  }

  // Display current menu and screen
  if ((millis() >= display_frame_timestamp))
  {
    switch (menu)
    {
    case 0:
      display_menu_cockpit();
      break;
    case 1:
      display_menu_experimental();
      break;
    case 2:
      display_menu_debug();
      break;
    case 3:
      display_menu_dtc(); // Fix later
      break;
    case 4:
      display_menu_settings();
      break;
    }
    display_frame_timestamp = millis() + DISPLAY_FRAME_LENGTH;
  }
}
