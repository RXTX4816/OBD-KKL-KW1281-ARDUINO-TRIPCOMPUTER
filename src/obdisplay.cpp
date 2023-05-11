/*
OBDisplay.cpp

See readme for more info.

See https://www.blafusel.de/obd/obd2_kw1281.html for info on OBD KWP1281 protocol.

Ignore compile warnings.
*/

// Arduino/Standard Libraries
#include <Arduino.h>
//  Third party libraries
#include "LiquidCrystal.h"
#include "NewSoftwareSerial.h"
/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// https://github.com/RXTX4816/OBD-KKL-KW1281-ARDUINO-TRIPCOMPUTER/issues/2
// ------------------------------------------------------------------------
#define DEBUG 1               // 1 = enable Serial.print, 0 = dont include serial code to save flash memory
uint8_t obd_pre_delay = 25;   // 25 is default value. Please try it the following: {0, 5, 10, 15, 35, 50, 80}
const bool AUTO_SETUP = true; // Debug on, ECU connect mode, 1200 baud, ADDR 0x01
// ----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_INSTRUMENTS = 0x17;

/* Pins */
uint8_t pin_rx = 3; // Receive
uint8_t pin_tx = 2; // Transmit

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */

/* Config */
#define ECU_TIMEOUT 1300         // Most commonly is 1100ms
#define DISPLAY_FRAME_LENGTH 333 // Length of 1 frame in ms
#define DISPLAY_MAX_X 16
#define DISPLAY_MAX_Y 2
const char CHAR_YES = 'Y';
const char CHAR_NO = 'N';
char STRING_ERR[] = "ERR";
#define BUTTON_RIGHT(in) (in < 60)
#define BUTTON_UP(in) (in >= 60 && in < 200)
#define BUTTON_DOWN(in) (in >= 200 && in < 400)
#define BUTTON_LEFT(in) (in >= 400 && in < 600)
#define BUTTON_SELECT(in) (in >= 600 && in < 800)
uint32_t endTime = 0;

// Backend
NewSoftwareSerial obd(pin_rx, pin_tx, false); // rx, tx, inverse logic = false
uint32_t connect_time_start = millis();
uint16_t timeout_to_add = 1100; // Wikipedia
uint16_t button_timeout = 222;
uint16_t connection_attempts_counter = 0;
uint64_t button_read_time = 0;

// OBD Connection variables
bool connected = false;
uint16_t baud_rate = 0; // 1200, 2400, 4800, 9600, 10400
uint8_t block_counter = 0;
uint8_t addr_selected = 0x00; // Selected ECU address to connect to, see ECU Addresses constants
bool com_error = false;

/* Display */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // 16x2 display
uint32_t display_frame_timestamp = millis();

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
#define debugstrnum(str, num)
#define debugstrnumln(str, num)
#define debugstrnumhexln(str, num)
#endif

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

void disconnect()
{
  obd.end();
  debug(F("Disconnected. Block counter: "));
  debug(block_counter);
  debug(F(". Connected: "));
  debug(connected);
  debug(F(". Available: "));
  debugln(obd.available());

  block_counter = 0;
  connected = false;
  connect_time_start = 0;
  button_read_time = 0;
  addr_selected = 0x00;

  delay(2222);
  lcd_print(0, 1, "Disconnected", 16);
  delay(1222);
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

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void obdWrite(uint8_t data)
{

  debug(F("->MCU: "));
  debughexln(data);

  uint8_t to_delay = 5;
  switch (baud_rate)
  {
  case 1200:
    to_delay = obd_pre_delay;
    break;
  case 2400:
    to_delay = 20;
    break;
  case 4800:
    to_delay = 15;
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

    debugstrnum(F(" "), bit);

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
  debugstrnumln(F("---KWPSendAckBlock block counter = "), block_counter);
  char buf[32];
  buf[0] = 0x03;
  buf[1] = block_counter;
  buf[2] = 0x09;
  buf[3] = 0x03;
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

  debugstrnum(F(" - KWPReceiveBlock. Size: "), size);
  debugstrnum(F(". Block counter: "), block_counter);

  if (size > maxsize)
  {
    debugln(F(" - KWPReceiveBlock error: Invalid maxsize"));
    lcd_print(0, 1, "ERR:size>maxsize");
    delay(2000);
    return false;
  }
  unsigned long timeout = millis() + timeout_to_add;
  uint16_t temp_iteration_counter = 0;
  uint8_t temp_0x0F_counter = 0;
  while ((recvcount == 0) || (recvcount < size))
  {
    while (obd.available())
    {

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

      if (initialization_phase && recvcount > maxsize)
      {
        Serial.print(F(" - - - Init com error. Recievecount:"));
        Serial.print(recvcount);
        Serial.print(F(" data:0x"));
        Serial.println(data, HEX);

        if (data == 0x55)
        {
          temp_0x0F_counter = 0;
          // Maybe ECU is trying to send sync bytes again?
          Serial.println(F(" - - - Maybe sync bytes coming in"));
          // Reset to beginning of init phase
          s[0] = 0x00;
          s[1] = 0x00;
          s[2] = 0x00;
          size = 3;
          recvcount = 1;
          s[0] = 0x55;
          Serial.println(F(" - - - Reset KWPReceiveBlock: s={0x55,0,0}, size=3, recvcount=1"));

          timeout = millis() + timeout_to_add;
          Serial.print(F(" - KWPReceiveBlock info: Added timeout. ReceiveCount: "));
          Serial.print((uint8_t)recvcount);
          Serial.print(F(". Processed data: "));
          Serial.print((uint8_t)data, HEX);
          Serial.print(F(". ACK compl: "));
          Serial.print(((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)));
          Serial.print(F(". Iteration: "));
          Serial.print(temp_iteration_counter);
        }
        else if (data == 0xFF)
        {
          temp_0x0F_counter = 0;
          Serial.println(F(" - - - Expected, skipping. Maybe beginning of COM ERROR"));
        }
        else if (data == 0x0F)
        {
          if (temp_0x0F_counter >= 1)
          {
            Serial.println(F(" - - - Expected, acknowledge"));
            obdWrite(data ^ 0xFF);
            timeout = millis() + timeout_to_add;
            Serial.print(F(" - KWPReceiveBlock info: Added timeout. ReceiveCount: "));
            Serial.print((uint8_t)recvcount);
            Serial.print(F(". Processed data: "));
            Serial.print((uint8_t)data, HEX);
            Serial.print(F(". ACK compl: "));
            Serial.print(((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)));
            Serial.print(F(". Iteration: "));
            Serial.print(temp_iteration_counter);
            temp_0x0F_counter = 0;
          }
          else
          {
            temp_0x0F_counter++;
          }
        }
        else
        {
          Serial.println(F(" - - - Unknown data sent by ECU! Trying to ignore and NOT send an ACK"));
          temp_0x0F_counter = 0;
        }
        continue;
      }
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
            lcd.setCursor(0, 1);
            lcd.print("ERR: BLOCK COUNT");
            delay(1000);
            lcd.print("Exp:" + String(data) + " Is:" + String(block_counter) + "         ");
            delay(1111);
          }
          debugstrnum(F(" - KWPReceiveBlock error: Invalid block counter. Expected: "), (uint8_t)data);
          debugstrnumln(F(" Is: "), (uint8_t)block_counter);
          return false;
        }
      }
      if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
      {
        obdWrite(data ^ 0xFF); // send complement ack
      }
      timeout = millis() + timeout_to_add;
      // debugstrnum(F(" - KWPReceiveBlock: Added timeout. ReceiveCount: "), (uint8_t)recvcount);
      // debug(F(". Processed data: "));
      // debughex(data);
      // debugstrnumln(F(". ACK compl: "), ((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)));
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
      return false;
    }
    temp_iteration_counter++;
  }
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
    if (!(KWPReceiveBlock(s, 64, size, -1, initialization_phase)))
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
 * KW1281 exit procedure
 */
bool kwp_exit()
{
  lcd.clear();
  lcd_print(0, 0, "Exiting...");
  debugln(F("Manual KWP exit.."));
  // Perform KWP end output block
  delay(15);
  char s[64];
  s[0] = 0x03;
  s[1] = block_counter;
  s[2] = 0x06;
  s[3] = 0x03;
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
    if (BUTTON_SELECT(analogRead(0)))
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
  if (!KWPReceiveBlock(response, 3, response_size, -1, true))
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
    lcd_print(0, 1, "ECU: " + String((uint8_t)response[0], HEX) + " " + String((uint8_t)response[1], HEX) + " " + String((uint8_t)response[2], HEX), 16);
    delay(ECU_TIMEOUT);
    return false;
  }
  if (((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A)))
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
    lcd_print(0, 1, "ECU: " + String((uint8_t)response[0], HEX) + " " + String((uint8_t)response[1], HEX) + " " + String((uint8_t)response[2], HEX), 16);
    delay(ECU_TIMEOUT);
    return false;
  }

  // display_statusbar();

  // debugln(F("KWP5BaudInit DONE"));
  lcd_print(0, 1, "Read ECU data...");
  // debugln(F("ReadConnectBlocks"));
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

bool connect()
{

  debugstrnumln(F("Connect attempt: "), connection_attempts_counter);

  // Startup configuration // 0 = false, 1 = true, -1 = undefined for booleans as int8_t
  uint16_t userinput_baudrate = 1200;
  uint8_t userinput_baudrate_pointer = 0; // for default 9600
  uint16_t supported_baud_rates_size = 5;
  uint16_t supported_baud_rates[supported_baud_rates_size] = {1200, 2400, 4800, 9600, 10400};
  int8_t userinput_ecu_address = -1; // 1 or 17
  uint8_t userinput_pre_delay = obd_pre_delay;
  uint8_t userinput_pre_delay_pointer = 0;
  uint8_t userinput_pre_delays_size = 7;
  uint8_t userinput_pre_delays[userinput_pre_delays_size] = {0, 5, 10, 15, 35, 50, 80};

  lcd.clear();
  lcd_print(0, 0, "<--   Baud:  -->");
  lcd_print(2, 1, "-> " + String(userinput_baudrate), 10);
  bool pressed_enter = false;
  if (AUTO_SETUP) {
    userinput_baudrate = 1200;
    pressed_enter = true;
  }
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

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select ECU addr:");
  lcd.setCursor(0, 1);
  lcd.print("<-- 01    17 -->");
  if (AUTO_SETUP)
  {
    userinput_ecu_address = 0x01;
  }
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
    return false;
  }
  delay(111);

  lcd.clear();
  lcd_print(0, 0, "<--Pre delay-->");
  lcd_print(3, 1, "-> " + String(userinput_pre_delays[userinput_pre_delay_pointer]), 10);
  pressed_enter = false;
  while (!pressed_enter)
  {
    int user_input = analogRead(0);
    if (BUTTON_RIGHT(user_input))
    {
      // Right button
      if (userinput_pre_delay_pointer >= 6)
      {
        userinput_pre_delay_pointer = 0;
      }
      else
        userinput_pre_delay_pointer++;
      // userinput_baudrate_last = userinput_baudrate;
      userinput_pre_delay = userinput_pre_delays[userinput_pre_delay_pointer];
      lcd_print(2, 1, "-> " + String(userinput_pre_delay), 10);
      delay(333);
    }
    else if (BUTTON_LEFT(user_input))
    {
      // Left button
      if (userinput_pre_delay_pointer <= 0)
      {
        userinput_pre_delay_pointer = 6;
      }
      else
        userinput_pre_delay_pointer--;
      // userinput_baudrate_last = userinput_baudrate;
      userinput_pre_delay = userinput_pre_delays[userinput_pre_delay_pointer];
      lcd_print(2, 1, "-> " + String(userinput_pre_delay), 10);
      delay(333);
    }
    else if (BUTTON_SELECT(user_input))
    {
      // Enter button
      pressed_enter = true;
    }
    delay(10);
  }
  obd_pre_delay = userinput_pre_delay;
  delay(555);

  debugln(F("Saved configuration: "));
  debugstrnumln(F("--- baud "), baud_rate);
  debugstrnumln(F("--- pre delay "), obd_pre_delay);
  debugstrnumhexln(F("--- addr "), addr_selected);

  // Connect to ECU
  connection_attempts_counter++;
  if (!obd_connect())
  {
    disconnect();
    return false;
  }
  connect_time_start = millis();
  return true;
}

void setup()
{

// For debug USB Serial monitor
#if DEBUG == 1
  Serial.begin(9600);
#endif

  // Display
  lcd.begin(DISPLAY_MAX_X, DISPLAY_MAX_Y);

  // Pins
  pinMode(pin_tx, OUTPUT); // TX white
  digitalWrite(pin_tx, HIGH);

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

  // Send ack to keep connection alive
  if (!send_ack())
  {
    disconnect();
    return;
  }

  if (millis() - connect_time_start < 1000)
  {
    lcd.clear();
    lcd_print(0, 0, "BLCK: ");
    lcd_print(0, 1, "Secs: ");
  }

  // Button input
  bool button_pressed = false;
  if (millis() > endTime)
  {
    // User input, menu selection
    int user_input = analogRead(0);

    if (BUTTON_SELECT(user_input))
    {
      // Exit/Reconnect
      kwp_exit();
      disconnect();
      return;
    }
  }

  // Display current menu and screen
  if ((millis() >= display_frame_timestamp))
  {
    lcd_print(6, 0, block_counter);
    lcd_print(6, 1, (uint16_t) ((millis()-connect_time_start)/1000));
    display_frame_timestamp = millis() + DISPLAY_FRAME_LENGTH;
  }
}
