/*
OBDisplay.cpp

See readme for more info.

See https://www.blafusel.de/obd/obd2_kw1281.html for info on OBD KWP1281 protocol.

Ignore compile warnings.
*/

// Arduino/Standard Libraries
#include <Arduino.h>
// #include <EEPROM.h>
#include <Wire.h>
#include <time.h>
//  Third party libraries
#include "LiquidCrystal.h"
#include "NewSoftwareSerial.h"
#include "bigInt.h"

/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */

/* Config */
bool no_input_mode = false; // If you have no buttons connected, mainly used for fast testing
bool auto_setup = false;
bool simulation_mode_active = false; // If simulation mode is active the device will display imaginary values
bool debug_mode_enabled = false;
bool compute_stats = false; // Whether statistic values should be computed (Fuel/100km etc.) Remember division is expensive on these processors.
uint8_t ecu_addr = 17;
int setup_max_duration = 500;
unsigned long last_random_number_time = 0;
int random_number_update_rate = 1500;

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_ABS_BRAKES = 0x03; // UNUSED
const uint8_t ADDR_AUTO_HVAC = 0x08;  // UNUSED
const uint8_t ADDR_INSTRUMENTS = 0x17;

/* Pins */
uint8_t pin_rx = 3; // Receive
uint8_t pin_tx = 2; // Transmit

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */

// Backend
NewSoftwareSerial obd(pin_rx, pin_tx, false); // rx, tx, inverse logic = false
unsigned long connect_time_start = millis();
unsigned long timeout_to_add = 1000;
int screen_current = 0;
int menu_current = 0;
String last_first_line = "";
String last_second_line = "";
unsigned long endTime = 0;
byte menu_max = 4;
byte menu = 0;
byte menu_last = menu;
bool menu_switch = false;
int connection_attempts_counter = 0;
unsigned long button_read_time = 0;

// OBD Connection variables
bool connected = false;
bool connected_last = connected; // Connection with ECU active
int available_last = 0;
int baud_rate = 0; // 4800, 9600, 10400
int baud_rate_error_count = 0;
int baud_rate_total_error_count = 0;
unsigned int block_counter = 0;
unsigned int block_counter_last = block_counter; // Could be byte maybe?
int group_current = 1;
uint8_t addr_current = -1;   // Current address of connection
uint8_t addr_default = 0x17; // Default address = ENGINE
uint8_t addr_to_reconnect = -1;
uint8_t addr_selected = 0x00; // Selected ECU address to connect to, see ECU Addresses constants
bool com_error = false;
bool com_warning = false;
bool com_warning_last = com_warning; // Whether a communication warning occured // Block length warning. Expected 15 got " + String(data)

/* Temporary Measurements for if you want to find out which values show up in your groups in a desired ECU address.
Just uncomment and add the logic in readSensors(). This can also be done with VCDS or other tools.*/
byte k[4] = {0, 0, 0, 0};
float v[4] = {-1, -1, -1, -1};
byte first_k = 0;
float first_v = -1;
byte second_k = 0;
float second_v = -1;
byte third_k = 0;
float third_v = -1;
byte fourth_k = 0;
float fourth_v = -1;

/* ADDR_INSTRUMENTS measurement group entries, chronologically 0-3 in each group */
// Group 1
int vehicle_speed = 0;
int vehicle_speed_last = vehicle_speed;
int engine_rpm = 0;
int engine_rpm_last = engine_rpm; // Also in ADDR_Engine Group 1 0th
int oil_pressure_min = 0;
int oil_pressure_min_last = oil_pressure_min;
int time_ecu = 0;
int time_ecu_last = time_ecu;
// Group 2
unsigned long odometer = 0;
unsigned long odometer_last = odometer;
unsigned long odometer_start = odometer;
int fuel_level = 0;
int fuel_level_last = fuel_level;
int fuel_level_start = fuel_level;
int fuel_sensor_resistance = 0;
int fuel_sensor_resistance_last = fuel_sensor_resistance; // Ohm
float ambient_temp = 0;
float ambient_temp_last = ambient_temp;
// Group 3 (Only 0-2)
int coolant_temp = 0;
int coolant_temp_last = coolant_temp;
int oil_level_ok = 0;
int oil_level_ok_last = oil_level_ok;
int oil_temp = 0;
int oil_temp_last = oil_temp;
// ADDR_ENGINE measurement group entries TODO
// Group 1 (0th is engine rpm)
int temperature_unknown_1 = 0;                // 1
float lambda = 0;                             // 2
bool exhaust_gas_recirculation_error = false; // 3, 8 bit encoding originally
bool oxygen_sensor_heating_error = false;
bool oxgen_sensor_error = false;
bool air_conditioning_error = false;
bool secondary_air_injection_error = false;
bool evaporative_emissions_error = false;
bool catalyst_heating_error = false;
bool catalytic_converter = false;
// Group 3 (Only 1-3 no 0th)
int pressure = 0; // mbar
float tb_angle = 0;
float steering_angle = 0;
// Group 4 (Only 1-3 no 0th)
float voltage = 0;
int temperature_unknown_2 = 0;
int temperature_unknown_3 = 0;
// Group 6 (Only 1 and 3)
float engine_load = 0; // 1
float lambda_2 = 0;    // 3

/* Computed Stats */
float elapsed_seconds_since_start = 0;
float elapsed_seconds_since_start_last = elapsed_seconds_since_start;
int elpased_km_since_start = 0;
int elpased_km_since_start_last = elpased_km_since_start;
int fuel_burned_since_start = 0;
int fuel_burned_since_start_last = fuel_burned_since_start;
float fuel_per_100km = 0;
float fuel_per_100km_last = fuel_per_100km;
float fuel_per_hour = 0;
float fuel_per_hour_last = fuel_per_hour;

/* Display */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // 16x2 display

// DEBUG infos
String error_msg1 = "";
String error_msg2 = "";

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
    return "Y";
  }
  else
  {
    return "N";
  }
}
char convert_bool_char(bool value)
{
  if (value)
  {
    return 'Y';
  }
  else
    return 'N';
}
String convert_int_to_string(int value)
{
  char result[15];
  sprintf(result, "%d", value);
  return result;
}
String convert_int_to_string(uint16_t value)
{
  char result[15];
  sprintf(result, "%d", value);
  return result;
}

String floatToString(float v)
{
  String res;
  char buf[16];
  dtostrf(v, 4, 2, buf);
  res = String(buf);
  return res;
}
bool check_msg_length(String msg)
{
  return msg.length() <= 16; // Display size 16 characters
}

// Display functions
void increment_menu()
{
  if (menu >= menu_max)
  {
    menu_last = menu;
    menu = 0;
  }
  else
  {
    menu++;
  }
  menu_switch = true;
}
void decrement_menu()
{
  if (menu == 0)
  {
    menu_last = menu;
    menu = menu_max;
  }
  else
  {
    menu--;
  }
  menu_switch = true;
}
void init_statusbar()
{
  lcd.setCursor(0, 0);
  lcd.print("C:");
  lcd.setCursor(4, 0);
  lcd.print("A:");
}
/**
 * Status bar: "C:Y A:9999*****"
 */
void display_statusbar()
{
  lcd.setCursor(2, 0);
  lcd.print(convert_bool_char(connected));
  String temp_obd_available = String(obd.available());
  uint8_t temp_obd_available_x_coordinate = 6;
  uint8_t temp_obd_available_length = temp_obd_available.length();
  if (temp_obd_available_length < 5 && temp_obd_available_length != 0)
  {
    temp_obd_available_x_coordinate += 4 - temp_obd_available_length;
  }
  else
  {
    temp_obd_available = "9999";
    temp_obd_available_x_coordinate += temp_obd_available.length();
  }
  for (int i = 6; i < 9; i++)
  {
    // Clear chars at position 6, 7 and 8 if current number is small
    if (i < temp_obd_available_x_coordinate)
    {
      lcd.setCursor(i, 0);
      lcd.print(" ");
    }
  }
  lcd.setCursor(temp_obd_available_x_coordinate, 0);
  lcd.print(temp_obd_available);
}
void init_menu_cockpit()
{
  lcd.setCursor(4, 0);
  lcd.print("KMH");
  lcd.setCursor(13, 0);
  lcd.print("RPM");
  lcd.setCursor(3, 1);
  lcd.print("C"); // Coolant
  lcd.setCursor(8, 1);
  lcd.print("C"); // Oil
  lcd.setCursor(13, 1);
  lcd.print("L"); // Fuel

}
void init_menu_experimental()
{
  
}
void init_menu_debug()
{
}
void init_menu_dtc()
{
}
void init_menu_settings()
{
}
void display_menu_cockpit()
{
}
void display_menu_experimental()
{
}
void display_menu_debug()
{
}
void display_menu_dtc()
{
}
void display_menu_settings()
{
}

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

bool engine_rpm_switch = true;
bool kmh_switch = true;
bool coolant_switch = true;
bool oil_switch = true;
bool fuellevel_switch = true;
bool fuelconsumption_switch = true;
void simulate_values()
{
  // Simulate some values
  increase_block_counter();
  if (random(0, 4) == 1)
    com_warning = !com_warning;

  // Vehicle speed
  if (kmh_switch)
    vehicle_speed += 1;
  else
    vehicle_speed -= 1;
  if (kmh_switch && vehicle_speed >= 200)
    kmh_switch = false;
  else if (!kmh_switch && vehicle_speed <= 0)
    kmh_switch = true;

  // Engine RPM
  if (engine_rpm_switch)
    engine_rpm += 77;
  else
    engine_rpm -= 77;
  if (engine_rpm_switch && engine_rpm >= 7100)
    engine_rpm_switch = false;
  else if (!engine_rpm_switch && engine_rpm <= 0)
    engine_rpm_switch = true;

  // Coolant temperature
  if (coolant_switch)
    coolant_temp += 1;
  else
    coolant_temp -= 1;
  if (coolant_switch && coolant_temp >= 160)
    coolant_switch = false;
  else if (!coolant_switch && coolant_temp <= 0)
    coolant_switch = true;

  // Oil Temperature
  if (oil_switch)
    oil_temp += 1;
  else
    oil_temp -= 1;
  if (oil_switch && oil_temp >= 160)
    oil_switch = false;
  else if (!oil_switch && oil_temp <= 0)
    oil_switch = true;

  // Oil level ok
  oil_level_ok = 1;

  // Fuel
  if (fuellevel_switch)
    fuel_level += 1;
  else
    fuel_level -= 1;
  if (fuellevel_switch && fuel_level >= 57)
    fuellevel_switch = false;
  else if (!fuellevel_switch && fuel_level <= 0)
    fuellevel_switch = true;

  // Fuel consumption
  if (fuelconsumption_switch)
    fuel_per_100km += 1;
  else
    fuel_per_100km -= 1;
  if (fuelconsumption_switch && fuel_per_100km >= 25)
    fuelconsumption_switch = false;
  else if (!fuelconsumption_switch && fuel_per_100km <= 0)
    fuelconsumption_switch = true;
}

void compute_values()
{
  elapsed_seconds_since_start = ((millis() - connect_time_start) / 1000);
  elpased_km_since_start = odometer - odometer_start;
  fuel_burned_since_start = abs(fuel_level_start - fuel_level);
  fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
  fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;
}

void disconnect()
{
  delay(3500);
  block_counter = 0;
  connected = false;
  connect_time_start = 0;
  odometer_start = 0;
  fuel_level_start = 0;
  menu = 0;
  menu_last = menu;
  menu_switch = false;
  button_read_time = 0;
  addr_current = -1;
  screen_current = 0;
  menu_current = 0;
  if (debug_mode_enabled)
  {
    Serial.println(F("Disconnected"));
  }
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
 * @brief Checks if the connection with the ECU is active. Function below converts 1 and 0 to "Y" and "N"
 */
bool is_connected()
{
  return connected;
}
String is_connected_as_string()
{
  return convert_bool_string(is_connected());
}

void obdWrite(uint8_t data)
{
  if (debug_mode_enabled)
  {
    Serial.print(F("MCU: "));
    Serial.println(data, HEX);
  }
  delay(5);
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
      if (debug_mode_enabled)
      {
        Serial.println(F("ERROR: obdRead() timeout"));
      }
      // errorTimeout++;
      return -1;
    }
  }
  uint8_t data = obd.read();
  if (debug_mode_enabled)
  {
    Serial.print("ECU: ");
    Serial.println(data, HEX);
  }
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
  byte bits[bitcount];
  byte even = 1;
  byte bit;
  for (int i = 0; i < bitcount; i++)
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
    if (debug_mode_enabled)
    {
      Serial.print(F("bit"));
      Serial.print(i);
      Serial.print(F("="));
      Serial.print(bit);
      if (i == 0)
        Serial.print(F(" startbit"));
      else if (i == 8)
        Serial.print(F(" parity"));
      else if (i == 9)
        Serial.print(F(" stopbit"));
      Serial.println();
    }

    bits[i] = bit;
  }
  // now send bit stream
  for (int i = 0; i < bitcount + 1; i++)
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
      digitalWrite(pin_tx, HIGH);
    }
    else
    {
      // low
      digitalWrite(pin_tx, LOW);
    }
  }
  obd.flush();
}

/**
 * Helper method to send the addr as 5 baud to the ECU
 */
bool KWP5BaudInit(uint8_t addr)
{
  if (debug_mode_enabled)
  {
    Serial.println(F("---KWP 5 baud init"));
  }
  send5baud(addr);
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
  if (debug_mode_enabled)
  {
    Serial.print(F("---KWPSend sz="));
    Serial.print(size);
    Serial.print(F(" block_counter="));
    Serial.println(block_counter);
    // show data
    Serial.print(F("OUT:"));
    for (int i = 0; i < size; i++)
    {
      uint8_t data = s[i];
      Serial.print(data, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  for (int i = 0; i < size; i++)
  {
    uint8_t data = s[i];
    obdWrite(data);
    /*uint8_t echo = obdRead(); ???
    if (data != echo){
      Serial.println(F("ERROR: invalid echo"));
      disconnect();
      errorData++;
      return false;
    }*/
    if (i < size - 1)
    {
      uint8_t complement = obdRead();
      if (complement != (data ^ 0xFF))
      {
        if (debug_mode_enabled)
        {
          Serial.println(F("ERROR: invalid complement"));
        }
        lcd.setCursor(0, 0);
        error_msg1 = "Sent: " + String(char(data ^ 0xFF)) + " Resp: " + String(char(complement));
        error_msg2 = "ERR: INV COMPL";
        lcd.print("Sent: " + String(char(data)) + " Resp: " + String(char(complement)));
        lcd.setCursor(0, 1);
        lcd.print("ERR: INV COMPL");
        // errorData++;
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
  if (debug_mode_enabled)
  {
    Serial.print(F("---KWPSendAckBlock block_counter="));
    Serial.println(block_counter);
  }
  char buf[32];
  sprintf(buf, "\x03%c\x09\x03", block_counter);
  return (KWPSendBlock(buf, 4));
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
  int recvcount = 0;
  if (size == 0)
    ackeachbyte = true;

  if (debug_mode_enabled)
  {
    Serial.print(F("---KWPReceive sz="));
    Serial.print(size);
    Serial.print(F(" block_counter="));
    Serial.println(block_counter);
  }

  if (size > maxsize)
  {
    if (debug_mode_enabled)
    {
      Serial.println("ERROR: invalid maxsize");
    }
    lcd.setCursor(0, 1);
    lcd.print("ERR:size>maxsize");
    delay(2000);
    return false;
  }
  unsigned long timeout = millis() + timeout_to_add;
  unsigned long timeout_last = timeout;
  while ((recvcount == 0) || (recvcount != size))
  {
    while (obd.available())
    {
      data = obdRead();
      if (data == -1)
      {
        lcd.setCursor(0, 1);
        lcd.print("Timeout obdRead");
        delay(1000);
        return false;
      }
      s[recvcount] = data;
      recvcount++;
      if ((size == 0) && (recvcount == 1))
      {
        if (source == 1 && (data != 15 || data != 3) && obd.available())
        {
          lcd.setCursor(0, 1);
          lcd.print("WARN block length");
          // lcd.setCursor(0, 2);
          // lcd.print("Exp 15 Is " + String(data));
          com_error = true;
          size = 6;
        }
        else
        {
          size = data + 1;
        }
        if (size > maxsize)
        {
          //          Serial.println("ERROR: invalid maxsize");
          lcd.setCursor(0, 1);
          lcd.print("ERR:size>maxsize");
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
            lcd.setCursor(0, 1);
            lcd.print("ERR: BLOCK COUNT");
            delay(1000);
            lcd.print("Exp:" + String(data) + " Is:" + String(block_counter) + "         ");
            delay(3333);
            return false;
          }
        }
      }
      if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
      {
        obdWrite(data ^ 0xFF); // send complement ack
        //delay(25);
        //uint8_t echo = obdRead();
        //if (echo != (data ^ 0xFF))
        //{
        //  if (debug_mode_enabled)
        //  {
        //    Serial.print(F("ERROR: invalid echo "));
        //    Serial.println(echo, HEX);
        //  }
          // errorData++;
          // If ECHO is wrong just keep going
          // return false;
        //}
      }
      timeout_last = timeout;
      timeout = millis() + timeout_to_add;
    }

    if (millis() >= timeout)
    {
      unsigned long timeout_difference = abs(millis() - timeout);
      if (debug_mode_enabled)
      {
        Serial.print(F("ERROR: timeout overstepped by "));
        Serial.print(timeout_difference);
        Serial.println(F(" ms"));
      }
      lcd.setCursor(0, 1);
      lcd.print("Timeout " + String(timeout_difference) + " ms");
      delay(2000);
      // errorTimeout++;
      return false;
    }
  }
  if (debug_mode_enabled)
  {
    // show data
    Serial.print(F("IN: sz="));
    Serial.print(size);
    Serial.print(F(" data="));
    for (int i = 0; i < size; i++)
    {
      uint8_t data = s[i];
      Serial.print(data, HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
  increase_block_counter();
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
  if (debug_mode_enabled)
  {
    Serial.println(F("------readconnectblocks"));
  }
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
      if (debug_mode_enabled)
      {
        Serial.println(F("ERROR: unexpected answer"));
      }
      lcd.setCursor(0, 1);
      lcd.print("ERR: s[2]!=xF6    ");
      delay(2000);
      // errorData++;
      return false;
    }
    String text = String(s);
    info += text.substring(3, size - 2);
    if (!KWPSendAckBlock())
      return false;
  }
  if (debug_mode_enabled)
  {
    Serial.print("label=");
    Serial.println(info);
  }
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
  if (debug_mode_enabled)
  {
    Serial.print(F("------readSensors "));
    Serial.println(group);
  }

  for (int i = 0; i <= 3; i++)
  {
    k[i] = 0;
    v[i] = -1;
  }

  // first_k = -1;
  // first_v = -1;
  // second_k = -1;
  // second_v = -1;
  // third_k = -1;
  // third_v = -1;
  // fourth_k = -1;
  // fourth_v = -1;

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
    sprintf(s, "\x03%c\x00\x03", block_counter);
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
    if (debug_mode_enabled)
    {
      Serial.println(F("ERROR: invalid answer"));
    }
    lcd.setCursor(0, 1);
    lcd.print("ERR: s[2]!=xe7  ");
    delay(2000);
    // errorData++;
    return false;
  }
  int count = (size - 4) / 3;
  if (debug_mode_enabled)
  {
    Serial.print(F("count="));
    Serial.println(count);
  }
  for (int idx = 0; idx < count; idx++)
  {
    byte k = s[3 + idx * 3];
    byte a = s[3 + idx * 3 + 1];
    byte b = s[3 + idx * 3 + 2];
    String n;
    float v = 0;

    if (debug_mode_enabled)
    {
      Serial.print(F("type="));
      Serial.print(k);
      Serial.print(F("  a="));
      Serial.print(a);
      Serial.print(F("  b="));
      Serial.print(b);
      Serial.print(F("  text="));
    }
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

    switch (addr_current)
    {
    case ADDR_INSTRUMENTS:
      switch (group)
      {
      case 1:
        switch (idx)
        {
        case 0:
          // 0.0 km/h Speed
          vehicle_speed = (int)v;
          break;
        case 1:
          // 0 /min Engine Speed
          engine_rpm = (int)v;
          break;
        case 2:
          // Oil Pr. 2 < min (Oil pressure 0.9 bar)
          oil_pressure_min = (int)v;
          break;
        case 3:
          // 21:50 Time
          time_ecu = (int)v;
          break;
        }
        break;
      case 2:
        switch (idx)
        {
        case 0:
          // 101010 Odometer
          odometer = (int)v;
          if (millis() - connect_time_start < 3500)
          {
            odometer_start = odometer;
          }
          break;
        case 1:
          // 9.0 l Fuel level
          fuel_level = v;
          fuel_level = v;
          if (millis() - connect_time_start < 3500)
          {
            fuel_level_start = fuel_level;
          }
          break;
        case 2:
          // 93 ohms Fuel Sender Resistance
          fuel_sensor_resistance = (int)v;
          break;
        case 3:
          // 0.0°C Ambient Temperature
          ambient_temp = v;
          break;
        }
        break;
      case 3:
        switch (idx)
        {
        case 0:
          // 12.0°C Coolant temp.
          coolant_temp = (int)v;
          break;
        case 1:
          // OK Oil Level (OK/n.OK)
          break;
        case 2:
          // 11.0°C Oil temp
          oil_temp = (int)v;
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
          engine_rpm = (int)v;
          break;
        case 1:
          // 0 /min Engine Speed
          temperature_unknown_1 = (int)v;
          break;
        case 2:
          // Oil Pr. 2 < min (Oil pressure 0.9 bar)
          lambda = v;
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
          pressure = (int)v;
          break;
        case 2:
          // 93 ohms Fuel Sender Resistance
          tb_angle = v;
          break;
        case 3:
          // 0.0°C Ambient Temperature
          steering_angle = v;
          break;
        }
        break;
      case 4:
        switch (idx)
        {
        case 0:
          break;
        case 1:
          voltage = v;
          break;
        case 2:
          temperature_unknown_2 = (int)v;
          break;
        case 3:
          temperature_unknown_3 = (int)v;
          break;
        }
        break;
      case 6:
        switch (idx)
        {
        case 0:
          break;
        case 1:
          engine_load = v;
          break;
        case 2:
          break;
        case 3:
          lambda_2 = v;
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
//    Serial.println(t);

    //lcd.setCursor(0, idx);
    //while (t.length() < 20) t += " ";
    //lcd.print(t);
  }
  sensorCounter++;*/
  return true;
}

int random_integer(int min, int max)
{
  return random(min, max);
}
float random_float()
{
  return 0.00;
}

bool obd_connect()
{
  block_counter = 0;

  lcd.clear();
  init_statusbar();
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("->PRESS ENTER<-");
  bool select = false;
  while (!select)
  {
    display_statusbar();
    int user_input = analogRead(0);
    if (user_input >= 600 && user_input < 800)
    {
      select = true;
    }
    delay(10);
  }

  lcd.setCursor(0, 1);
  lcd.print("OBD.begin      ");
  if (debug_mode_enabled)
  {
    Serial.println(F("OBD.begin()"));
  }
  obd.begin(baud_rate); // Baud rate 9600 for Golf 4/Bora or 10400 in weird cases
  lcd.print("OBD.begin  DONE");
  display_statusbar();
  delay(144);
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("5BaudInit      ");
  if (debug_mode_enabled)
  {
    Serial.print(F("KWP5BaudInit on addr "));
    Serial.println(addr_selected, HEX);
  }
  if (!simulation_mode_active && !KWP5BaudInit(addr_selected))
  {
    lcd.setCursor(0, 1);
    lcd.print("5BaudInit  ERROR");
    if (debug_mode_enabled)
    {
      Serial.println(F("KWP5BaudInit ERROR"));
    }
    return false;
  }
  display_statusbar();

  // printDebug("Init ADDR " + String(addr_selected) + " with " + baud_rate + " baud");
  char response[3]; // Response should be 0x55, 0x01, 0x8A
  int response_size = 3;
  lcd.setCursor(0, 1);
  lcd.print("Handshake       ");
  if (!simulation_mode_active && !KWPReceiveBlock(response, 3, response_size, true))
  {

    lcd.setCursor(0, 1);
    lcd.print("Handshake  ERROR");
    display_statusbar();
    if (debug_mode_enabled)
    {
      Serial.println(F("KWP5BaudInit Handshake ERROR"));
      Serial.println(F("THE FOLLOWING MAY HAVE NO CORRELATION TO THE ERROR"));
      Serial.print(F("Expected ["));
      Serial.print(0x55, HEX);
      Serial.print(F(" "));
      Serial.print(0x01, HEX);
      Serial.print(F(" "));
      Serial.print(0x8A, HEX);
      Serial.print(F("] got ["));
      Serial.print((uint8_t)response[0], HEX);
      Serial.print(F(" "));
      Serial.print((uint8_t)response[1], HEX);
      Serial.print(F(" "));
      Serial.print((uint8_t)response[2], HEX);
      Serial.println(F("]"));
    }
    delay(1444);
    display_statusbar();
    lcd.setCursor(0, 1);
    lcd.print("ECU: " + String((uint8_t)response[0], HEX) + " " + String((uint8_t)response[1], HEX) + " " + String((uint8_t)response[2], HEX) + "      ");
    delay(3800);
    // printError("connect() KWPReceiveBlock error");
    return false;
  }
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("Handshake    ...");
  if (!simulation_mode_active && ((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A))) // 85 1 138
  {
    display_statusbar();
    lcd.setCursor(0, 1);
    lcd.print("Handshake  WRONG");
    if (debug_mode_enabled)
    {
      Serial.println(F("Handshake Error!"));
      Serial.print(F("Expected ["));
      Serial.print(0x55, HEX);
      Serial.print(F(" "));
      Serial.print(0x01, HEX);
      Serial.print(F(" "));
      Serial.print(0x8A, HEX);
      Serial.print(F("] got ["));
      Serial.print((uint8_t)response[0], HEX);
      Serial.print(F(" "));
      Serial.print((uint8_t)response[1], HEX);
      Serial.print(F(" "));
      Serial.print((uint8_t)response[2], HEX);
      Serial.println(F("]"));
    }
    // printError("Expected [" + String(0x55) + " " + String(0x01) + " " + String(0x8A) + "] got [" + String((uint8_t)response[0]) + " " + String((uint8_t)response[1]) + " " + String((uint8_t)response[2]) + "]");
    return false;
  }
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("Handshake  RIGHT");
  if (debug_mode_enabled)
  {
    Serial.println(F("KWP5BaudInit Handshake DONE"));
  }
  delay(88);
  if (debug_mode_enabled)
  {
    Serial.println(F("KWP5BaudInit DONE"));
  }
  lcd.setCursor(0, 1);
  lcd.print("5BaudInit   DONE");
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("Read ECU data...");

  if (debug_mode_enabled)
  {
    Serial.println(F("ReadConnectBlocks"));
  }
  if (!simulation_mode_active && !readConnectBlocks(true))
  {
    display_statusbar();
    lcd.setCursor(0, 1);
    lcd.print("Read ECU data..N");
    // printError("readConnectBlocks() error");
    return false;
  }
  if (debug_mode_enabled)
  {
    Serial.println(F("ReadConnectBlocks DONE"));
    Serial.println(F("Connected to ECU!"));
  }
  connected = true;
  display_statusbar();
  lcd.setCursor(0, 1);
  lcd.print("Read ECU data..Y");
  delay(44);
  // printDebug("Connection to ECU established!");
  lcd.setCursor(0, 1);
  lcd.print(" ECU connected! ");
  display_statusbar();
  return true;
}

bool connect()
{
  //  Get ECU Addr to connect to from user
  if (connection_attempts_counter > 0)
  {

    if (debug_mode_enabled)
    {
      Serial.print(F("WARN: connection_attempts_counter > 0. Is: "));
      Serial.println(connection_attempts_counter);
    }
    // If you are here this means this is not the first time your MCU is trying to connect
  }

  // Connect to ECU
  connection_attempts_counter++;
  if (!obd_connect())
  {
    disconnect();
    return false;
  }
  connect_time_start = millis();
  menu_switch = true;
  return true;
}

void setup()
{

  Serial.begin(9600);
  // char lcd_buff[17];
  lcd.begin(16, 2); // col, rows
  lcd.clear();

  // Pins
  pinMode(pin_tx, OUTPUT); // TX
  digitalWrite(pin_tx, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    O B D");
  lcd.setCursor(0, 1);
  lcd.print(" D I S P L A Y ");

  delay(1444);

  // Startup configuration // 0 = false, 1 = true, -1 = undefined for booleans as int8_t
  int8_t userinput_debug_mode = -1; // Whether to print Serial messages
  int8_t userinput_simulation_mode = -1;
  uint16_t userinput_baudrate = 9600;
  uint16_t userinput_baudrate_last = userinput_baudrate;
  uint8_t userinput_baudrate_pointer = 3; // for default 9600
  uint16_t supported_baud_rates_size = 5;
  uint16_t supported_baud_rates[supported_baud_rates_size] = {1200, 2400, 4800, 9600, 10400};
  uint8_t userinput_ecu_address = 0; // 1 or 17

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Debug mode on?");
  lcd.setCursor(0, 1);
  lcd.print("<-- Y      N -->");
  while (userinput_debug_mode == -1)
  {
    int user_input = analogRead(0);
    if (user_input < 60)
    {
      // Right button
      userinput_debug_mode = 0;
    }
    else if (400 <= user_input && user_input < 600)
    {
      // Left button
      userinput_debug_mode = 1;
    }
    delay(10);
  }
  if (userinput_debug_mode == 0)
  {
    lcd.print("           N -->");
    debug_mode_enabled = false;
  }
  else if (userinput_debug_mode == 1)
  {
    lcd.print("<-- Y           ");
    debug_mode_enabled = true;
  }
  else
  {
    setup();
  }
  delay(555);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select con mode:");
  lcd.setCursor(0, 1);
  lcd.print("<- ECU    SIM ->");
  while (userinput_simulation_mode == -1)
  {
    int user_input = analogRead(0);
    if (user_input < 60)
    {
      // Right button
      userinput_simulation_mode = 1;
    }
    else if (400 <= user_input && user_input < 600)
    {
      // Left button
      userinput_simulation_mode = 0;
    }
    delay(10);
  }
  if (userinput_simulation_mode == 0)
  {
    lcd.print("<- ECU          ");
    simulation_mode_active = false;
  }
  else if (userinput_simulation_mode == 1)
  {
    lcd.print("          SIM ->");
    simulation_mode_active = true;
  }
  else
  {
    setup();
  }
  delay(555);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("<--   Baud:  -->");
  lcd.setCursor(0, 1);
  lcd.print("  -> " + convert_int_to_string(userinput_baudrate) + "     ");
  bool pressed_enter = false;
  while (!pressed_enter)
  {
    int user_input = analogRead(0);
    if (user_input < 60)
    {
      // Right button
      if (userinput_baudrate_pointer >= 4)
      {
        userinput_baudrate_pointer = 0;
      }
      else
        userinput_baudrate_pointer++;
      userinput_baudrate_last = userinput_baudrate;
      userinput_baudrate = supported_baud_rates[userinput_baudrate_pointer];
      lcd.setCursor(0, 1);
      lcd.print("  -> " + convert_int_to_string(userinput_baudrate) + "     ");
      delay(333);
    }
    else if (400 <= user_input && user_input < 600)
    {
      // Left button
      if (userinput_baudrate_pointer <= 0)
      {
        userinput_baudrate_pointer = 4;
      }
      else
        userinput_baudrate_pointer--;
      userinput_baudrate_last = userinput_baudrate;
      userinput_baudrate = supported_baud_rates[userinput_baudrate_pointer];
      lcd.setCursor(0, 1);
      lcd.print("  -> " + convert_int_to_string(userinput_baudrate) + "     ");
      delay(333);
    }
    else if (user_input >= 600 && user_input < 800)
    {
      // Enter button
      pressed_enter = true;
    }
    delay(10);
  }

  bool save_check = false;
  for (int i = 0; i < supported_baud_rates_size; i++)
  {
    if (userinput_baudrate == supported_baud_rates[i])
      save_check = true;
  }
  if (!save_check)
    setup();

  baud_rate = userinput_baudrate;
  if (baud_rate < 4800)
    timeout_to_add = 2000; // Extend timeout for 1200 and 2400 baud due to concerns about its low communication speed
  delay(555);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select ECU addr:");
  lcd.setCursor(0, 1);
  lcd.print("<-- 01    17 -->");
  while (userinput_ecu_address == 0)
  {
    int user_input = analogRead(0);
    if (user_input < 60)
    {
      // Right button
      userinput_ecu_address = 17;
    }
    else if (400 <= user_input && user_input < 600)
    {
      // Left button
      userinput_ecu_address = 1;
    }
    delay(10);
  }
  if (userinput_ecu_address == 1)
  {
    lcd.setCursor(0, 1);
    lcd.print("<-- 01          ");
    addr_selected = ADDR_ENGINE;
  }
  else if (userinput_ecu_address == 17)
  {
    lcd.setCursor(0, 1);
    lcd.print("          17 -->");
    addr_selected = ADDR_INSTRUMENTS;
  }
  else
  {
    setup();
  }
  delay(555);

  if (debug_mode_enabled)
  {
    Serial.println(F("Saved configuration: "));
    Serial.println(F("--- DEBUG on"));
    if (simulation_mode_active)
      Serial.println(F("--- SIMULATION on"));
    else
      Serial.println(F("--- SIMULATION off"));
    Serial.print(F("--- "));
    Serial.print(baud_rate);
    Serial.println(F(" baud"));
    Serial.print(F("--- "));
    Serial.print(addr_selected, HEX);
    Serial.println(F(" HEX"));
  }
}

void loop()
{

  // Check connection
  if (!connected && !connect())
  {
    return;
  }

  // Update values
  if (!simulation_mode_active)
  {
    // Read the sensor groups
    for (int i = 1; i <= 3; i++)
    {
      if (!readSensors(i))
      {
        disconnect();
        return;
      }
    }
  }
  else
  {
    simulate_values();
    delay(333);
  }

  if (engine_rpm > 4000)
  {
    // TODO Turn on LED
  }
  if (oil_temp < 80)
  {
    // TODO
  }
  if (coolant_temp < 80)
  {
    // TODO
  }

  String first_line = "                ";
  String second_line = "                ";

  String vehicle_speed_string = String(vehicle_speed);
  String engine_rpm_string = String(engine_rpm);
  String coolant_temp_string = String(coolant_temp);
  String oil_temp_string = String(oil_temp);
  String fuel_level_string = String(fuel_level);
  String block_counter_string = String(block_counter);

  int vehicle_speed_string_length = strlen(vehicle_speed_string.c_str());
  int engine_rpm_string_length = strlen(engine_rpm_string.c_str());
  int coolant_temp_string_length = strlen(coolant_temp_string.c_str());
  int oil_temp_string_length = strlen(oil_temp_string.c_str());
  int fuel_level_string_length = strlen(fuel_level_string.c_str());
  int block_counter_string_length = strlen(block_counter_string.c_str());

  float elapsed_seconds_since_start = ((millis() - connect_time_start) / 1000);
  int elpased_km_since_start = odometer - odometer_start;
  int fuel_burned_since_start = fuel_level_start - fuel_level;

  float fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
  float fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;

  switch (menu_current)
  {
  case 0:
    // Default menu, tachometer etc
    switch (screen_current)
    {
    case 0:
      for (int i = 0; i < vehicle_speed_string_length; i++)
      {
        if (i + 1 >= vehicle_speed_string_length)
        {
          first_line.setCharAt(2, vehicle_speed_string.charAt(i));
        }
        else if (i + 2 >= vehicle_speed_string_length)
        {
          first_line.setCharAt(1, vehicle_speed_string.charAt(i));
        }
        else
        {
          first_line.setCharAt(0, vehicle_speed_string.charAt(i));
        }
      }
      first_line.setCharAt(4, 'K');
      first_line.setCharAt(5, 'M');
      first_line.setCharAt(6, 'H');
      for (int i = 0; i < engine_rpm_string_length; i++)
      {
        if (i + 1 >= engine_rpm_string_length)
        {
          first_line.setCharAt(11, engine_rpm_string.charAt(i));
        }
        else if (i + 2 >= engine_rpm_string_length)
        {
          first_line.setCharAt(10, engine_rpm_string.charAt(i));
        }
        else if (i + 3 >= engine_rpm_string_length)
        {
          first_line.setCharAt(9, engine_rpm_string.charAt(i));
        }
        else
        {
          first_line.setCharAt(8, engine_rpm_string.charAt(i));
        }
      }
      if (engine_rpm > 4000)
      {
        first_line.setCharAt(12, '-');
      }
      first_line.setCharAt(13, 'R');
      first_line.setCharAt(14, 'P');
      first_line.setCharAt(15, 'M');

      for (int i = 0; i < coolant_temp_string_length; i++)
      {
        if (i + 1 >= coolant_temp_string_length)
        {
          second_line.setCharAt(2, coolant_temp_string.charAt(i));
        }
        else if (i + 2 >= coolant_temp_string_length)
        {
          second_line.setCharAt(1, coolant_temp_string.charAt(i));
        }
        else
        {
          second_line.setCharAt(0, coolant_temp_string.charAt(i));
        }
      }
      second_line.setCharAt(3, 'C');
      for (int i = 0; i < oil_temp_string_length; i++)
      {
        if (i + 1 >= oil_temp_string_length)
        {
          second_line.setCharAt(7, oil_temp_string.charAt(i));
        }
        else if (i + 2 >= oil_temp_string_length)
        {
          second_line.setCharAt(6, oil_temp_string.charAt(i));
        }
        else
        {
          second_line.setCharAt(5, oil_temp_string.charAt(i));
        }
      }
      second_line.setCharAt(8, 'C');
      for (int i = 0; i < fuel_level_string_length; i++)
      {
        if (i + 1 >= fuel_level_string_length)
        {
          second_line.setCharAt(12, fuel_level_string.charAt(i));
        }
        else
        {
          second_line.setCharAt(11, fuel_level_string.charAt(i));
        }
      }
      second_line.setCharAt(13, 'L');
      // second_line = String(coolant_temp) + "C " + String(oil_temp) + "C " + floatToString(fuel_level) + "L";
      break;
    case 1:
      for (int i = 0; i < vehicle_speed_string_length; i++)
      {
        if (i + 1 >= vehicle_speed_string_length)
        {
          first_line.setCharAt(2, vehicle_speed_string.charAt(i));
        }
        else if (i + 2 >= vehicle_speed_string_length)
        {
          first_line.setCharAt(1, vehicle_speed_string.charAt(i));
        }
        else
        {
          first_line.setCharAt(0, vehicle_speed_string.charAt(i));
        }
      }
      first_line.setCharAt(4, 'K');
      first_line.setCharAt(5, 'M');
      first_line.setCharAt(6, 'H');
      for (int i = 0; i < engine_rpm_string_length; i++)
      {
        if (i + 1 >= engine_rpm_string_length)
        {
          first_line.setCharAt(11, engine_rpm_string.charAt(i));
        }
        else if (i + 2 >= engine_rpm_string_length)
        {
          first_line.setCharAt(10, engine_rpm_string.charAt(i));
        }
        else if (i + 3 >= engine_rpm_string_length)
        {
          first_line.setCharAt(9, engine_rpm_string.charAt(i));
        }
        else
        {
          first_line.setCharAt(8, engine_rpm_string.charAt(i));
        }
      }
      if (engine_rpm > 4000)
      {
        first_line.setCharAt(12, '-');
      }
      first_line.setCharAt(13, 'R');
      first_line.setCharAt(14, 'P');
      first_line.setCharAt(15, 'M');

      second_line = String(fuel_per_100km) + " | " + String(fuel_per_hour) + "           ";
      for (int i = 0; i < block_counter_string_length; i++)
      {
        if (i + 1 >= block_counter_string_length)
        {
          second_line.setCharAt(15, block_counter_string.charAt(i));
        }
        else if (i + 2 >= block_counter_string_length)
        {
          second_line.setCharAt(14, block_counter_string.charAt(i));
        }
        else
        {
          second_line.setCharAt(13, block_counter_string.charAt(i));
        }
      }
      break;
    default:
      screen_current = 0;
      return;
      break;
    }
    break;
  case 1:
    // Group select menu
    switch (screen_current)
    {
    case 0:
      first_line = "Group: " + String(group_current) + "            ";
      second_line = "-> results";
      break;
    case 1:
      first_line = "1 " + String(first_k) + " " + first_v;
      second_line = "2 " + String(second_k) + " " + second_v;
      break;
    case 2:
      first_line = "3 " + String(third_k) + " " + third_v;
      second_line = "4 " + String(fourth_k) + " " + fourth_v;
      break;
    default:
      screen_current = 0;
      return;
      break;
    }
    break;
  case 2:
    // MSG select menu
    first_line = "2/NA";
    second_line = "";
    return;
    break;
  case 3:
    // DTC Error menu
    first_line = "3/NA";
    second_line = "";
    return;
    break;
  default:
    menu_current = 0;
    screen_current = 0;
    return;
    break;
  }

  // Draw text
  if (!first_line.equals(last_first_line))
  {
    lcd.setCursor(0, 0);
    lcd.print(first_line + "                ");
    last_first_line = first_line;
  }
  if (!second_line.equals(last_second_line))
  {
    lcd.setCursor(0, 1);
    lcd.print(second_line + "                ");
    last_second_line = second_line;
  }

  bool button_pressed = false;
  // Button input
  if (millis() > endTime)
  {
    // User input, menu selection
    int user_input = analogRead(0);
    // User wants to change things a little bit
    if (user_input < 60)
    {
      // Right button
      // next screen
      // menu.next_screen();
      // delay(500);
      button_pressed = true;
      screen_current++;
    }
    else if (user_input < 200)
    {
      // Up button
      //
      button_pressed = true;
      // group_current++;
      menu_current++;
      screen_current = 0;
    }
    else if (user_input < 400)
    {
      // Down button
      //
      /*if (group_current > 1)
      {
        group_current--;

        delay(500);
      }*/
      button_pressed = true;
      menu_current--;
      screen_current = 0;
    }
    else if (user_input < 600)
    {
      // Left button
      // previous screen
      // menu.previous_screen();
      // delay(500);
      button_pressed = true;
      screen_current--;
    }
    else if (user_input < 800)
    {
      // Select button
      //
      button_pressed = true;
      /*if (addr_current == 0x01)
      {

        lcd.clear();
        if (readSensors(group_current))
        {
          if (vehicle_speed > v_max)
          {
            v_max = vehicle_speed;
            EEPROM.write(v_max_addr, v_max); // 3.3ms
          }
          /*lcd.setCursor(0, 0);
            lcd.print(floatToString(supply_voltage) + "V                      ");
            lcd.setCursor(0, 1);
            lcd.print(coolant_temp + "°C                     ");
          lcd.setCursor(0, 1);
          lcd.print("Sensors read");
        }
        else
        {
          lcd.setCursor(0, 1);
          lcd.print("ERR SENSORS");
          delay(1000);
          lcd.clear();
          return;
        }
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("             ");
        lcd.setCursor(0, 1);
        lcd.print("ERR: ECU NO SUPP");
        delay(2000);
        lcd.clear();
        return;
      }*/
    }
  }

  if (button_pressed)
  {
    endTime = millis() + 500;
  }

  // if (millis() - lastMillis > period) {
  // char buf[16];
  // time_passed_text = (char*) ltoa(lastMillis, buf, 10);
  // lastMillis = millis();
  // menu.update();
  // }

  // lcd.setCursor(0, 0);
  // lcd.print("Pressed Key:");

  // lcd.setCursor(0, 1);
  // lcd.print(selected_user_input);
}
