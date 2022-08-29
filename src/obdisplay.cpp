#include <Arduino.h>
#include <LiquidCrystal.h>
#include <LiquidMenu.h>
#include <Wire.h>
#include <time.h>
#include <EEPROM.h>
#include "NewSoftwareSerial.h"
#include "bigInt.h"

// VW BORA KWP1281, 036906034AM MARELLI ECU
const uint8_t ADDR_ENGINE = 0x01;
// Label File: None
// Measurement Blocks - 0x08:
// Group 00: 1-10 with 3 digit numbers
// Group 01: 0 /min RPM, 17.0°C Temperature, 0.0% Lambda, 10110010 Binary Bits
// Group 02: 0 /min RPM, 0.0% Load, 0.0ms Time Correction, 1012.0 mbar Absolute Pressure
// Group 03: 0 /min RPM, 1012.0 mbar Absolute Pressure, 5.4° T.B. Angle, 0.0° Steering Angle
// Group 04: 0 /min RPM, 11.70 V Voltage, 17.0°C Temperature, 14°C Temperature
// Group 05: 0 /min RPM, 0.0% Load, 0.0 km/h Speed, Part Throttle NA
// Group 06: 0 /min RPM, 0.0% Load, 14°C Temperature, -1.0% Lambda
// Group 14: 0 /min RPM, 0.0% Load, 0.0 no units, Enabled NA
// Readiness - 0x15:
// Readiness Status: 10100101
// Exhaust Gas Recirculation: Failed or Incomplete
// Oxygen Sensor Heating: Passed
// Oxygen Sensor(s): Failed or Incomplete
// Air Conditioning: Passed
// Secondary Air Injection: Passed
// Evaporative Emissions: Failed or Incomplete
// Catalyst Heating: Passed
// Catalytic Converter(s): Failed or Incomplete
const uint8_t ADDR_ABS_BRAKES = 0x03; // UNUSED
// Label File: None
// Measurement Blocks - 0x08:
// Group 4: 0.00° Steering Angle, 0.31 m/s^2 Lateral Acceleration, -0.36°/s Turn Rate, N/A
// Group 5: -1.27 bar Pressure, 0.42 bar Pressure, N/A, N/A
const uint8_t ADDR_AUTO_HVAC = 0x08; // UNUSED
// Label File: 381-907-044.LBL
// Measurement Blocks - 0x08:
// Group 1: 8.0 A/C clutch sw-off cond. 1-12, 0.0 Eng. speed recog (0=no, 1=yes), 0.0 km/h Road Speed, 121.0 Standing time (0-240min.)
// Group 2: 42.0 Measured Value, 42.0 Specified Value, 219.0 Position: air supply cooled, 42.0 Position: air supply heated
// Group 3: 221.0 Measured Value, 221.0 Specified Value, 221.0 Position: air flow to panel, 40.0 Position: air flow to footwell
// Group 4: 223.0 Measured Value, 223.0 Specified Value, 223.0 Position: air flow to footwell, 39.0 Position: air flow to defroster
// Group 5: 237.0 Measured Value, 234.0 Specified Value, 234.0 Position: fresh air, 30.0 Position: recirculating
// Group 6: 0.0°C Temp displayed on panel, 7.0°C Temp. air intake duct, 0.0°C Outside air temperature, 0.0% Sun photo sensor (0-120%)
// Group 7: 0.0 Outlet temp. panel (G191), 5.0°C Outlet temp. floor (G192), 3.0°C Panel temp. near LCD (G56), 0.0 n/a
// Group 8: 0.00V Specif. Voltage air blower, 0.28V Measured Voltage air blower, 12.18V Measured Voltage A/C clutch, 0.0 no units
const uint8_t ADDR_INSTRUMENTS = 0x17;
// Label File: 1J0-920-xx0.LBL
// Measurement Blocks - 0x08:
// Group 1: 0.0 km/h Speed, 0 /min Engine Speed, Oil Pr. 2 < min (Oil pressure 0.9 bar), 21:50 Time
// Group 2: 121960 Odometer, 9.0 l Fuel level, 93 ohms Fuel Sender Resistance, 0.0°C Ambient Temperature
// Group 3: 12.0°C Coolant temp., OK Oil Level (OK/n.OK), 11.0°C Oil temp, N/A




unsigned long last_random_number_time = 0;
int random_number_update_rate = 1500;

// Backend
uint8_t addr_current = -1;   // Current address of connection
uint8_t addr_default = 0x17; // Default address = ENGINE
int block_counter = 0;
int group_current = 1;
bool connected = false;
int baud_rate = 9600;
int baud_rate_error_count = 0;
int baud_rate_total_error_count = 0;
bool com_error = false;

// Menu
int screen_current = 0;
int menu_current = 0;
String last_first_line = "";
String last_second_line = "";
unsigned long endTime = 0;

// Pins
uint8_t pin_rx = 3;
uint8_t pin_tx = 2;
//int pin_led = -1;

// Temp Measurements
byte first_k = 0;
float first_v = -1;
byte second_k = 0;
float second_v = -1;
byte third_k = 0;
float third_v = -1;
byte fourth_k = 0;
float fourth_v = -1;

// Measurements addr 17
int vehicle_speed = 999;
int engine_rpm = 9999;
int oil_pressure_min = 9; // Oil Pr. 2 < min // NOT USED
int time_ecu = 9999;
unsigned long connect_time_start = millis();

int odometer = 999999;
int odometer_start = 999998;
int fuel_level = 99;
int fuel_level_start = 98;
int fuel_sensor_resistance; // Ohm
float ambient_temp = 99.9;

int coolant_temp = 999;
int oil_level_ok = 9;
int oil_temp = 999;
//int intake_air_temp = -1;
//int engine_load = -1;
//float throttle_valve = -1;
//float supply_voltage = -1;
//float fuel_consumption = -1;

// EEPROM local variables
//int v_max = -1;
//int fault_codes_count = -1;
//int errors_count = -1;

// EEPROM Storage Addresses
//int v_max_addr = 0;             // Store the maximum speed
//int fault_codes_count_addr = 1; // Amount of fault codes
//int errors_count_addr = 2;      // Amount of obdisplay errors (calculations, serial communication, display)

// Init libraries
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);          //16x2 display
LiquidMenu menu(lcd);                         // Menu for lcd
NewSoftwareSerial obd(pin_rx, pin_tx, false); // rx, tx, inverse logic = false

// Menu
//LiquidLine line_1_start(0, 0, "CON:N");
//LiquidLine line_2_start(0, 1, "PLEASE START");
//LiquidScreen screen_start(line_1_start, line_2_start);

// Config
byte simulation_mode_active = -1;
int setup_max_duration = 500;

// DEBUG infos
String error_msg1 = "";
String error_msg2 = "";

String convert_int_to_string(int value) {
  char result[15];
  sprintf(result, "%d", value);
  return result;
}

/**
 * @brief Converts a boolean to a String
 * 
 * @param value Boolean
 * @return String Y or N
 */
String convert_bool_string(bool value) {
  if (value) {
    return "Y";
  } else {
    return "N";
  }
}

void baud_error() {
  if (baud_rate_total_error_count > 6) {
    addr_default = 0x01;
  } 
  if (baud_rate_error_count > 1) {
      if (baud_rate == 9600) {
        baud_rate = 10400;
      } else {
        baud_rate = 9600;
      }
      baud_rate_error_count = 0;
  } else {
    baud_rate_error_count++;
  }
  
  baud_rate_total_error_count++;
}

// Check if connection with ECU is active
bool is_connected() {
  if (!obd.isListening()) {
    return false;
  } else {
    return true;
  }
}

void disconnect() {
  connected = false;
  addr_current = -1;
  odometer_start
 = 999998;
 fuel_level_start = 98;
  connect_time_start = millis();
  // TODO Kommunikationsende prozedur
}

void increase_block_counter() {
  if (block_counter == 255) {
    block_counter = 0;
  } else {
    block_counter++;
  }
}

void obdWrite(uint8_t data) {
  // #ifdef DEBUG
  // Serial.print("uC:");
  // Serial.println(data, HEX);*/
  // #endif
  delay(5);
  obd.write(data);
}

/**
 * @brief Read OBD input from ECU
 * 
 * @return uint8_t The incoming byte or -1 if timeout
 */
uint8_t obdRead() {
  unsigned long timeout = millis() + 1000;
  while (!obd.available()) {
    if (millis() >= timeout) {
      // Serial.println(F("ERROR: obdRead timeout"));
      // errorTimeout++;
      disconnect();
      return -1;
    }
  }
  uint8_t data = obd.read();
  //  #ifdef DEBUG
  //  Serial.print("ECU:");
  //  Serial.println(data, HEX);
  //  #endif
  return data;
}

String floatToString(float v)
{
  String res;
  char buf[16];
  dtostrf(v, 4, 2, buf);
  res = String(buf);
  return res;
}

// 5Bd, 7O1
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
    /*    Serial.print(F("bit"));
    Serial.print(i);
    Serial.print(F("="));
    Serial.print(bit);
    if (i == 0) Serial.print(F(" startbit"));
    else if (i == 8) Serial.print(F(" parity"));
    else if (i == 9) Serial.print(F(" stopbit"));
    Serial.println();*/
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

bool KWP5BaudInit(uint8_t addr)
{
  //  Serial.println(F("---KWP 5 baud init"));
  //delay(3000);
  send5baud(addr);
  return true;
}

bool KWPSendBlock(char *s, int size)
{
  /*  Serial.print(F("---KWPSend sz="));
  Serial.print(size);
  Serial.print(F(" block_counter="));
  Serial.println(block_counter);
  // show data
  Serial.print(F("OUT:"));*/
  //for (int i = 0; i < size; i++)
  //{
  //  uint8_t data = s[i];
    //    Serial.print(data, HEX);
    //    Serial.print(" ");
  //}
  //  Serial.println();
  for (int i = 0; i < size; i++)
  {
    uint8_t data = s[i];
    obdWrite(data);
    /*uint8_t echo = obdRead();
    if (data != echo){
      Serial.println(F("ERROR: invalid echo"));
      disconnect();
      errorData++;
      return false;
    }*/
    if (i < size - 1) {
      uint8_t complement = obdRead();
      if (complement != (data ^ 0xFF)) {
        //        Serial.println(F("ERROR: invalid complement"));
        lcd.setCursor(0, 0);
        error_msg1 = "Sent: " + String(char(data ^ 0xFF)) + " Resp: " + String(char(complement));
        error_msg2 = "ERR: INV COMPL";
        lcd.print("Sent: " + String(char(data)) + " Resp: " + String(char(complement)));
        lcd.setCursor(0, 1);
        lcd.print("ERR: INV COMPL");
        delay(3000);
        disconnect();
        //errorData++;
        return false;
      }
    }
  }
  increase_block_counter();
  return true;
}

bool KWPSendAckBlock()
{
  /*  Serial.print(F("---KWPSendAckBlock block_counter="));
  Serial.println(block_counter);*/
  char buf[32];
  sprintf(buf, "\x03%c\x09\x03", block_counter);
  return (KWPSendBlock(buf, 4));
}

uint8_t data_temp = 0;
int block_count_errors = 0;
// count: if zero given, first received byte contains block length
// 4800, 9600 oder 10400 Baud, 8N1
// source: 
// -1 = default | 1 = readsensors
bool KWPReceiveBlock(char s[], int maxsize, int &size, int source=-1)
{
  bool ackeachbyte = false;
  uint8_t data = 0;
  int recvcount = 0;
  if (size == 0)
    ackeachbyte = true;
  /*  Serial.print(F("---KWPReceive sz="));
  Serial.print(size);
  Serial.print(F(" block_counter="));
  Serial.println(block_counter); */
  if (size > maxsize)
  {
    //    Serial.println("ERROR: invalid maxsize");
    lcd.setCursor(0, 1);
    lcd.print("ERR:size>maxsize                 ");
    delay(2000);
    return false;
  }
  unsigned long timeout = millis() + 1000;
  while ((recvcount == 0) || (recvcount != size)) {
    while (obd.available()) {
      data = obdRead();
      if (data == -1) {
        lcd.setCursor(0, 1);
        lcd.print("Timeout                ");
        delay(2000);
        return false;
      }
      s[recvcount] = data;
      recvcount++;
      if ((size == 0) && (recvcount == 1)) {
        if (source == 1 && (data != 15 || data != 3) && obd.available()) {
          lcd.setCursor(0, 1);
          lcd.print("WARN block length");
          lcd.setCursor(0, 2);
          lcd.print("Exp 15 Is " + String(data));
          com_error = true;
          size = 6;
        } else {
          size = data + 1;
        }
        if (size > maxsize) {
          //          Serial.println("ERROR: invalid maxsize");
          lcd.setCursor(0, 1);
          lcd.print("ERR:size>maxsize                 ");
          delay(2000);
          return false;
        }
      }
      if (com_error) {
        if (recvcount == 1) {
          ackeachbyte = false;
        } else if (recvcount == 3) {
          ackeachbyte = true;
        } else if (recvcount == 4) {
          ackeachbyte = false;
        } else if (recvcount == 6) {
          ackeachbyte = true;
        }
        continue;
      }
      if ((ackeachbyte) && (recvcount == 2)) {
        if (data != block_counter) {
          lcd.setCursor(0, 0);
          lcd.print("ERR: BLOCK COUNT                 ");
          lcd.setCursor(0, 1);
          lcd.print("Exp:" + String(data) + " Is:" + String(block_counter) + "         ");
          delay(2000);
          disconnect();
          return false;
        }
      }
      if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
      {
        obdWrite(data ^ 0xFF); // send complement ack
        /*uint8_t echo = obdRead();
        if (echo != (data ^ 0xFF)){
          Serial.print(F("ERROR: invalid echo "));
          Serial.println(echo, HEX);
          disconnect();
          errorData++;
          return false;
        }*/
      }
      timeout = millis() + 1000;
    }

    if (millis() >= timeout)
    {
      //      Serial.println(F("ERROR: timeout"));
      lcd.setCursor(0, 1);
      lcd.print("ERR: TIMEOUT                ");
      delay(2000);
      disconnect();
      //errorTimeout++;
      return false;
    }
  }
  // show data
  /*  Serial.print(F("IN: sz="));
  Serial.print(size);
  Serial.print(F(" data=")); */
  //for (int i = 0; i < size; i++)
  //{
    //uint8_t data = s[i];
    /*    Serial.print(data, HEX);
    Serial.print(F(" "));*/
  //}
  //  Serial.println();
  increase_block_counter();
  return true;
}

bool readConnectBlocks()
{
  // read connect blocks
  //  Serial.println(F("------readconnectblocks"));
  /*  lcd.setCursor(0, 0);
  lcd.print("KW1281 label");*/
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
      //      Serial.println(F("ERROR: unexpected answer"));
      lcd.setCursor(0, 1);
      lcd.print("ERR: s[2]!=xF6    ");
      delay(2000);
      disconnect();
      //errorData++;
      return false;
    }
    String text = String(s);
    info += text.substring(3, size - 2);
    if (!KWPSendAckBlock())
      return false;
  }
  //  Serial.print("label=");
  //  Serial.println(info);
  //lcd.setCursor(0, 1);
  //lcd.print(info);
  return true;
}

bool connect(uint8_t addr, int baudrate)
{
  /*  Serial.print(F("------connect addr="));
  Serial.print(addr);
  Serial.print(F(" baud="));
  Serial.println(baudrate);*/
  //tone(pinBuzzer, 1200);
  //delay(100);
  //noTone(pinBuzzer);
  /*  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("KW1281 wakeup");
  //  lcdPrint(0, 1, "", 20);
  //  lcdPrint(0, 2, "", 20);*/
  block_counter = 0;
  addr_current = -1;
  obd.begin(baudrate);
  KWP5BaudInit(addr);
  // answer: 0x55, 0x01, 0x8A
  char s[3];
  /*  lcd.setCursor(0, 0);
  lcd.print("KW1281 recv"); */
  int size = 3;
  if (!KWPReceiveBlock(s, 3, size))
    return false;
  if ((((uint8_t)s[0]) != 0x55) || (((uint8_t)s[1]) != 0x01) || (((uint8_t)s[2]) != 0x8A))
  {
    //    Serial.println(F("ERROR: invalid magic"));
    disconnect();
    //errorData++;
    return false;
  }
  //connected = true;
  if (!readConnectBlocks())
    return false;
  addr_current = addr;
  return true;
}

bool readSensors(int group)
{
  //  Serial.print(F("------readSensors "));
  //  Serial.println(group);
  //lcd.setCursor(0, 0);
  // lcd.print("KW1281 sensor");

  //first_k = -1;
  //first_v = -1;
  //second_k = -1;
  //second_v = -1;
  //third_k = -1;
  //third_v = -1;
  //fourth_k = -1;
  //fourth_v = -1;

  char s[64];
  sprintf(s, "\x04%c\x29%c\x03", block_counter, group);
  if (!KWPSendBlock(s, 5))
    return false;
  int size = 0;
  if (!KWPReceiveBlock(s, 64, size, 1)) {
    return false;
  }
  if (com_error) {
    // Kommunikationsfehler
    char s[64];
    sprintf(s, "\x03%c\x00\x03", block_counter);
    if (!KWPSendBlock(s, 4)) {
      com_error = false;
      return false;
    }
    block_counter = 0;
    com_error = false;
    int size2 = 0;
    if (!KWPReceiveBlock(s, 64, size2)) {
      return false;
    }
  }
  if (s[2] != '\xe7')
  {
    //    Serial.println(F("ERROR: invalid answer"));
    disconnect();
    lcd.setCursor(0, 1);
    lcd.print("ERR: s[2]!=xe7  ");
    delay(2000);
    //errorData++;
    return false;
  }
  int count = (size - 4) / 3;
  //  Serial.print(F("count="));
  //  Serial.println(count);
  for (int idx = 0; idx < count; idx++)
  {
    byte k = s[3 + idx * 3];
    byte a = s[3 + idx * 3 + 1];
    byte b = s[3 + idx * 3 + 2];
    String n;
    float v = 0;
    /*    Serial.print(F("type="));
    Serial.print(k);
    Serial.print(F("  a="));
    Serial.print(a);
    Serial.print(F("  b="));
    Serial.print(b);
    Serial.print(F("  text="));*/
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

    /*switch (idx)
    {
    case 0:
      first_k = k;
      first_v = v;
      break;
    case 1:
      second_k = k;
      second_v = v;
      break;
    case 2:
      third_k = k;
      third_v = v;
      break;
    case 3:
      fourth_k = k;
      fourth_v = v;
      break;
    }*/

    switch (addr_current) {
    case ADDR_INSTRUMENTS:
      switch (group) {
      case 1:
        switch (idx) {
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
          // 121960 Odometer
          odometer = (int)v;
          break;
        case 1:
          // 9.0 l Fuel level
          fuel_level = v;
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
        switch (idx) {
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
    }

    /*if (addr_current == 0x01) {
      if (group == 4) {
        if (idx == 1) {
          supply_voltage = v;
        } else {
          coolant_temp = (int)v;
        }
      }
    }*/
    /*switch (addr_current) {
      case 0x01:
        switch (group) {
          case 4:
            switch (idx) {
              case 1: supply_voltage = v;  break;        //OK
              case 2: coolant_temp = (int)v;    break;        //OK
              break;
            }
            break;
          case 5:
            switch (idx) {
              case 1: engine_load = (int)v;     break;      
              case 2: vehicle_speed = (int)v;   break;        //OK
              break;
            }
            break;
          case 134:
            switch (idx) {
              case 0: oil_temp = (int)v;     break;      //or 134-3
              break;
            }
            break;
        }
        break;*/
    /*     case ADR_Dashboard:
               switch (group) {
                 case 1:
                   switch (idx) {
                     case 0: vehicleSpeed = v; break;
                     case 1: engineSpeed = v; break;
                     case 2: oilPressure = v; break;
                   }
                   break;
                 case 2:
                   switch (idx) {
                     case 0: odometer = v; break;
                     case 1: fuelLevel = v; break;
                   }
                   break;
                 case 50:
                   switch (idx) {
                     case 1: engineSpeed = v; break;
                     case 2: oilTemp = v; break;
                     case 3: coolantTemp = v; break;
                   }
                   break;
               }*/
    //break;
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

void alarm()
{
  //  if (alarmCounter > 10) return;
  //  tone(pinBuzzer, 1200);
  //  delay(100);
  //  noTone(pinBuzzer);
  //  alarmCounter++;
}

int random_integer(int min, int max) {
  return random(min, max);
}
float random_float() {
  return 0.00;
}

void setup()
{
  //char lcd_buff[17];
  lcd.begin(16, 2); // col, rows
  lcd.clear();

  //lcd.setCursor(0, 0);
  //lcd.print("OBDISPLAY");

  //lcd.print("EEPROM init..");
  //delay(800);
  //v_max = (int)EEPROM.read(v_max_addr);
  //fault_codes_count = (int)EEPROM.read(fault_codes_count_addr);
  //errors_count = (int)EEPROM.read(errors_count_addr);
  //lcd.clear();
  //lcd.setCursor(0, 0);
  //lcd.print("Setup obdisplay.");
  //lcd.setCursor(0, 1);
  //lcd.print("EEPROM rdy!");
  //delay(800);

  // Pins
  pinMode(pin_tx, OUTPUT);
  digitalWrite(pin_tx, HIGH);
  //pinMode(pin_led, OUTPUT);
  //analogWrite(pin_led, 50);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    O B D");
  lcd.setCursor(0, 1);
  lcd.print(" D I S P L A Y ");

  delay(444);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select mode:");
  lcd.setCursor(0, 1);
  lcd.print("ECU or SIM");
  unsigned long setup_start_time = millis();
  while(simulation_mode_active != 0 && simulation_mode_active != 1) {

    if (millis() - setup_start_time > setup_max_duration) {
      simulation_mode_active = 0;
      break;
    }

    int user_input = analogRead(0);
    if (user_input < 60){
      // Right button
      simulation_mode_active = 1;
    } else if (400 <= user_input && user_input < 600) {
      // Left button
      simulation_mode_active = 0;
    }
    delay(10);
  }
  lcd.clear();
  if (simulation_mode_active == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Selected mode:");
    lcd.setCursor(0, 1);
    lcd.print("ECU connect");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Selected mode:");
    lcd.setCursor(0, 1);
    lcd.print("SIMULATION");
  }

  delay(444);

  // menu.add_screen(welcome_screen);
  // menu.add_screen(screen2);
  // menu.add_screen(pwm_screen);
  // menu.update();
}

void loop()
{

  if (!connected){
    block_counter = 0;
    addr_current = -1;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CON:" + convert_bool_string(connected) + " AVA:" + String(obd.available()));
    lcd.setCursor(0, 1);
    lcd.print("Init " + String(baud_rate) + " -> " + String(addr_default));
    
    //bool select = false;
    bool select = true;
    while (!select) {
      int user_input = analogRead(0);
      if (user_input >= 600 && user_input < 800) {
        select = true;
      }
      delay(10);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CON:" + convert_bool_string(connected) + " AVA:" + String(obd.available()));
    lcd.setCursor(0, 1);
    lcd.print("Connecting");

    if (!simulation_mode_active) {
      obd.begin(baud_rate); // Baud rate 9600 for Golf 4/Bora or 10400 in weird cases
    //lcd.setCursor(0, 1);
    //lcd.print("BaudInit..");
    KWP5BaudInit(addr_default);
    //lcd.setCursor(0, 1);
    //lcd.print("Init..       ");
    char response[3]; // Response = 0x55, 0x01, 0x8A
    int response_size = 3;
    if (!KWPReceiveBlock(response, 3, response_size))
    {
      lcd.setCursor(0, 1);
      lcd.print("INIT RESP ERROR            ");
      //baud_error(); // Test
      delay(1000);
      lcd.clear();
      return;
    }
    lcd.setCursor(6, 1);
    lcd.print(" 1" + convert_bool_string(((uint8_t)response[0]) == 0x55) +
              " 2" + convert_bool_string(((uint8_t)response[1]) == 0x01) +
              " 3" + convert_bool_string(((uint8_t)response[2]) == 0x8A));
    if ((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A))
    {
      lcd.setCursor(0, 0);
      lcd.print("DEF " + String(0x55) + " " + String(0x01) + " " + String(0x8A) + "               ");
      lcd.setCursor(0, 1);
      lcd.print("ERR " + String((uint8_t)response[0]) + " " + String((uint8_t)response[1]) + " " + String((uint8_t)response[2]) + "            ");
      baud_error();
      disconnect();
      delay(2000);
      return;
    }
    if (!readConnectBlocks())
    {
      lcd.setCursor(0, 1);
      lcd.print("ERR ConnectBlock             ");
      delay(2000);
      return;
    }
    addr_current = addr_default;
    connected = true;
    lcd.setCursor(0, 0);
    lcd.print("CON:" + convert_bool_string(connected) + " AVA:" + obd.available() + " ");
    lcd.setCursor(12, 0);
    lcd.print("               ");
    lcd.setCursor(0, 1);
    lcd.print("SUCC CONNECTION!             ");
    last_first_line = "SUCCESSFUL             ";
    last_second_line = "CONNECTION             ";
    return;
    } else {
      lcd.setCursor(0, 0);
    lcd.print("CON:" + convert_bool_string(connected) + " AVA:" + obd.available() + " ");
    lcd.setCursor(12, 0);
    lcd.print("               ");
    lcd.setCursor(0, 1);
    lcd.print("SUCC SIM CONNECTION!         ");
    last_first_line = "SUCCESSFUL SIM        ";
    last_second_line = "CONNECTION             ";
    connect_time_start = millis();
    connected = true;
    return;
    }

    
  }

  if (!simulation_mode_active) {
    for (int i = 1; i <= 3; i++)
    if (!readSensors(i)) {
      disconnect();
      delay(4000);
      return;
    } 
    if (odometer_start == 999998) {
      odometer_start = odometer;
    }
    if (fuel_level_start == 98) {
      fuel_level_start = fuel_level;
    }
  } else {
    unsigned long current_time = millis();
    if (current_time - last_random_number_time > random_number_update_rate) {
      vehicle_speed = random_integer(0, 195);
      engine_rpm = random_integer(700, 6000);
      fuel_level = random_integer(42, 42);
      fuel_level_start = 42;
      coolant_temp = random_integer(25, 130);
      oil_temp = random_integer(25, 130);
      odometer_start = 111111;
      odometer = 120000;
      odometer++;
      last_random_number_time = current_time;
      increase_block_counter();
    }
    
  }

  // TODO:
  // Engine RPM > 4000 (ORANGE LED)
  // Oil temp or Cooler temp over 115 (RED LED / SOUND)


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

  float elapsed_seconds_since_start = ((millis() - connect_time_start)/1000);
  int elpased_km_since_start = odometer - odometer_start;
  int fuel_burned_since_start = fuel_level_start - fuel_level;

//  10km     160sec   1L  
//  
//
//
//
  float fuel_per_100km = (100/elpased_km_since_start)*fuel_burned_since_start;
  float fuel_per_hour = (3600/elapsed_seconds_since_start)*fuel_burned_since_start;

  switch (menu_current) {
  case 0:
    // Default menu, tachometer etc
    switch (screen_current) {
    case 0:
      for (int i = 0; i < vehicle_speed_string_length; i++) {
        if (i+1 >= vehicle_speed_string_length) {
          first_line.setCharAt(2, vehicle_speed_string.charAt(i));
        } else if (i+2 >= vehicle_speed_string_length) {
          first_line.setCharAt(1, vehicle_speed_string.charAt(i));
        } else {
          first_line.setCharAt(0, vehicle_speed_string.charAt(i));
        }
      }
      first_line.setCharAt(4, 'K');
      first_line.setCharAt(5, 'M');
      first_line.setCharAt(6, 'H');
      for (int i = 0; i < engine_rpm_string_length; i++) {
        if (i+1 >= engine_rpm_string_length) {
          first_line.setCharAt(11, engine_rpm_string.charAt(i));
        } else if (i+2 >= engine_rpm_string_length) {
          first_line.setCharAt(10, engine_rpm_string.charAt(i));
        } else if (i+3 >= engine_rpm_string_length) {
          first_line.setCharAt(9, engine_rpm_string.charAt(i));
        } else {
          first_line.setCharAt(8, engine_rpm_string.charAt(i));
        }
      }
      if (engine_rpm > 4000) {
        first_line.setCharAt(12, '-');
      }
      first_line.setCharAt(13, 'R');
      first_line.setCharAt(14, 'P');
      first_line.setCharAt(15, 'M');

      for (int i = 0; i < coolant_temp_string_length; i++) {
        if (i+1 >= coolant_temp_string_length) {
          second_line.setCharAt(2, coolant_temp_string.charAt(i));
        } else if (i+2 >= coolant_temp_string_length) {
          second_line.setCharAt(1, coolant_temp_string.charAt(i));
        } else {
          second_line.setCharAt(0, coolant_temp_string.charAt(i));
        }
      }
      second_line.setCharAt(3, 'C');
      for (int i = 0; i < oil_temp_string_length; i++) {
        if (i+1 >= oil_temp_string_length) {
          second_line.setCharAt(7, oil_temp_string.charAt(i));
        } else if (i+2 >= oil_temp_string_length) {
          second_line.setCharAt(6, oil_temp_string.charAt(i));
        } else {
          second_line.setCharAt(5, oil_temp_string.charAt(i));
        }
      }
      second_line.setCharAt(8, 'C');
      for (int i = 0; i < fuel_level_string_length; i++) {
        if (i+1 >= fuel_level_string_length) {
          second_line.setCharAt(12, fuel_level_string.charAt(i));
        } else {
          second_line.setCharAt(11, fuel_level_string.charAt(i));
        }
      }
      second_line.setCharAt(13, 'L');
      //second_line = String(coolant_temp) + "C " + String(oil_temp) + "C " + floatToString(fuel_level) + "L";
      break;
    case 1:
      for (int i = 0; i < vehicle_speed_string_length; i++) {
        if (i+1 >= vehicle_speed_string_length) {
          first_line.setCharAt(2, vehicle_speed_string.charAt(i));
        } else if (i+2 >= vehicle_speed_string_length) {
          first_line.setCharAt(1, vehicle_speed_string.charAt(i));
        } else {
          first_line.setCharAt(0, vehicle_speed_string.charAt(i));
        }
      }
      first_line.setCharAt(4, 'K');
      first_line.setCharAt(5, 'M');
      first_line.setCharAt(6, 'H');
      for (int i = 0; i < engine_rpm_string_length; i++) {
        if (i+1 >= engine_rpm_string_length) {
          first_line.setCharAt(11, engine_rpm_string.charAt(i));
        } else if (i+2 >= engine_rpm_string_length) {
          first_line.setCharAt(10, engine_rpm_string.charAt(i));
        } else if (i+3 >= engine_rpm_string_length) {
          first_line.setCharAt(9, engine_rpm_string.charAt(i));
        } else {
          first_line.setCharAt(8, engine_rpm_string.charAt(i));
        }
      }
      if (engine_rpm > 4000) {
        first_line.setCharAt(12, '-');
      }
      first_line.setCharAt(13, 'R');
      first_line.setCharAt(14, 'P');
      first_line.setCharAt(15, 'M');

      second_line = String(fuel_per_100km) + " | " + String(fuel_per_hour) + "           ";
      for (int i = 0; i < block_counter_string_length; i++) {
        if (i+1 >= block_counter_string_length) {
          second_line.setCharAt(15, block_counter_string.charAt(i));
        } else if (i+2 >= block_counter_string_length) {
          second_line.setCharAt(14, block_counter_string.charAt(i));
        } else {
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
    switch (screen_current) {
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
      //menu.next_screen();
      // delay(500);
      button_pressed = true;
      screen_current++;
    }
    else if (user_input < 200)
    {
      // Up button
      //
      button_pressed = true;
      //group_current++;
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

  //if (millis() - lastMillis > period) {
  //char buf[16];
  //time_passed_text = (char*) ltoa(lastMillis, buf, 10);
  //lastMillis = millis();
  //menu.update();
  //}

  // lcd.setCursor(0, 0);
  // lcd.print("Pressed Key:");

  // lcd.setCursor(0, 1);
  // lcd.print(selected_user_input);
}
