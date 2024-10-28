#if DEBUG == 1 // Compile Serial
#define debug(in) Serial.print(in)
#define debughex(in) Serial.print(in, HEX)
#define debugln(in) Serial.println(in)
#define debughexln(in) Serial.println(in, HEX)
#define debugstrnum(str, num) \
    do                        \
    {                         \
        debug(str);           \
        debug(num);           \
    } while (0)
#define debugstrnumln(str, num) \
    do                          \
    {                           \
        debug(str);             \
        debugln(num);           \
    } while (0)
#define debugstrnumhex(str, num) \
    do                           \
    {                            \
        debug(str);              \
        debughex(num);           \
    } while (0)
#define debugstrnumhexln(str, num) \
    do                             \
    {                              \
        debug(str);                \
        debughexln(num);           \
    } while (0)
#else // Do not compile serial to save space
#define debug(in)
#define debughex(in)
#define debugln(in)
#define debughexln(in)
#define debugstrnum(str, num)
#define debugstrnumln(str, num)
#define debugstrnumhex(str, num)
#define debugstrnumhexln(str, num)
#endif


// Font
word back_color = TFT_BLACK;
word font_color = TFT_CYAN;
word font_color_alt = TFT_CYAN;
word font_color_warn = TFT_YELLOW;
word font_color_correct = TFT_GREEN;
word font_color_wrong = TFT_RED;
word font_color_dark = TFT_DARKGREY;
word font_color_alt_other = TFT_ORANGE;
void init_colors(bool night_mode_active = true) {
    // Set colors
    if (night_mode_active)
    {
        back_color = TFT_BLACK;
        font_color = TFT_CYAN;
        font_color_alt = TFT_CYAN;
        font_color_warn = TFT_YELLOW;
        font_color_correct = TFT_GREEN;
        font_color_wrong = TFT_RED;
        font_color_dark = TFT_LIGHTGREY;
        font_color_alt_other = TFT_ORANGE;
    }
    else
    {
        back_color = TFT_WHITE;
        font_color = TFT_BLACK;
        font_color_alt = TFT_BLUE;
        font_color_warn = TFT_YELLOW;
        font_color_correct = TFT_GREEN;
        font_color_wrong = TFT_RED;
        font_color_dark = TFT_DARKGREY;
        font_color_alt_other = TFT_ORANGE;
    }
}

// Timers in ms
// 0-7: User timers
#define TIMER_CODE_DISPLAY 8
#define TIMER_CODE_START_FRAME 9
#define TIMER_CODE_BUTTON_PRESSED 10
#define TIMER_CODE_START_MENU_SWITCH 11
#define TIMER_CODE_START_DRAWSCREEN 12
#define TIMER_CODE_STARTUP_ANIMATION 13
#define TIMER_CODE_INIT_STATUSBAR 14
#define TIMER_CODE_DRAW_STATUSBAR 15
#define TIMER_CODE_OBD_MODE 16
#define TIMER_CODE_OBD_CONNECT 17
#define TIMER_CODE_OBD_CONNECTIONSTART 18
#define TIMER_CODE_OBD_UPDATE 19
// Timer Buffers
#define TIMER_BUFFER_SIZE 32
uint32_t timer_start[TIMER_BUFFER_SIZE] = {0};
uint32_t timer_end[TIMER_BUFFER_SIZE] = {0};
uint32_t timer_duration[TIMER_BUFFER_SIZE] = {0};
// Timer Records
uint32_t timer_max_drawscreen = 0;
// Timer functions
bool timer_useable(uint8_t timer_code) {
    return timer_duration[timer_code] != 0;
}
void timer_reset(uint8_t timer_code) {
    timer_start[timer_code] = 0;
    timer_end[timer_code] = 0;
    timer_duration[timer_code] = 0;
}
void timer_begin(uint8_t timer_code) {
    timer_reset(timer_code);
    timer_start[timer_code] = millis();
}
void timer_done(uint8_t timer_code) {
    if (timer_start[timer_code] == 0) {
        timer_reset(timer_code);
        return;
    }
    timer_end[timer_code] = millis();
    timer_duration[timer_code] = timer_end[timer_code] - timer_start[timer_code];

    switch(timer_code) {
        case TIMER_CODE_START_DRAWSCREEN:
            if (timer_duration[timer_code] > timer_max_drawscreen)
                timer_max_drawscreen = timer_duration[timer_code];
            break;
    }
}


// Debug
const int debug_messages_as_code_length = 64;
unsigned int debug_messages_overflow_counter = 0;
unsigned int debug_messages_current = 0;
uint8_t debug_messages_as_code[debug_messages_as_code_length];
void debug_clear()
{
    for (uint8_t i = 0; i < debug_messages_as_code_length; i++)
    {
        debug_messages_as_code[i] = 0;
    }
    debug_messages_overflow_counter = 0;
    debug_messages_current = 0;
}
word debug_decode_string_color(uint8_t code) {
    switch(code) {
        case DEBUG_CODE_EMPTY:
        case DEBUG_CODE_NEWLINE:
        case DEBUG_CODE_INIT_5BAUD:
        case DEBUG_CODE_INIT_HANDSHAKE_CORRECT:
        case DEBUG_CODE_CONNECTBLOCKS_CORRECT:
        case DEBUG_CODE_INIT_CONNECTED:
        case DEBUG_CODE_INIT_SIMULATION_ACTIVE:
        case DEBUG_CODE_INIT_MULTIPLECONNECTATTEMPTS:
        case DEBUG_CODE_DTC_DTCCODESEMTPY:
        case DEBUG_CODE_DTC_CORRECT:
        case DEBUG_CODE_KWPEXIT_STARTING:
        case DEBUG_CODE_KWPEXIT_CORRECT:
            return font_color;
            break;
        case DEBUG_CODE_OVERFLOW:
        case DEBUG_CODE_KWPRECEIVEBLOCK_COMERROR_WARN:
        case DEBUG_CODE_DISCONNECTED_WARN:
        case DEBUG_CODE_SENSORS_COMERROR_WARN:
            return font_color_alt;
            break;
        case DEBUG_CODE_INIT_HANDSHAKE_ERROR:
        case DEBUG_CODE_INIT_HANDSHAKE_WRONG:
        case DEBUG_CODE_KWPRECEIVEBLOCK_SIZE_ERROR:
        case DEBUG_CODE_KWPRECEIVEBLOCK_EMPTYREAD_ERROR:
        case DEBUG_CODE_KWPRECEIVEBLOCK_BLOCKCOUNTER_ERROR:
        case DEBUG_CODE_KWPRECEIVEBLOCK_TIMEOUT_ERROR:
        case DEBUG_CODE_KWPRECEIVEBLOCK_TIMEOUT_NOCONNECTION_ERROR:
        case DEBUG_CODE_CONNECTBLOCKS_ERROR:
        case DEBUG_CODE_CONNECTBLOCKS_RECEIVE_ERROR:
        case DEBUG_CODE_CONNECTBLOCKS_SIZEZERO_ERROR:
        case DEBUG_CODE_CONNECTBLOCKS_WRONGANSWER_ERROR:
        case DEBUG_CODE_CONNECTBLOCKS_SENDACK_ERROR:
        case DEBUG_CODE_SENDBLOCK_INVALIDCOMPLEMENT_ERROR:
        case DEBUG_CODE_DISCONNECTED_ERROR:
        case DEBUG_CODE_RECEIVEACK_ERROR:
        case DEBUG_CODE_RECEIVEACK_WRONGVALUE_ERROR:
        case DEBUG_CODE_KEEPALIVE_ERROR:
        case DEBUG_CODE_DTC_INITBLOCKSEND_ERROR:
        case DEBUG_CODE_DTC_RECEIVEBLOCK_ERROR:
        case DEBUG_CODE_DTC_WRONGVALUE_ERROR:
        case DEBUG_CODE_DTC_SENDACK_ERROR:
        case DEBUG_CODE_DTCDELETE_SENDBLOCK_ERROR:
        case DEBUG_CODE_DTCDELETE_RECEIVEBLOCK_ERROR:
        case DEBUG_CODE_DTCDELETE_WRONGVALUE_ERROR:
        case DEBUG_CODE_KWPEXIT_SENDBLOCK_ERROR:
        case DEBUG_CODE_SENSORS_SENDBLOCK_ERROR:
        case DEBUG_CODE_SENSORS_RECEIVEBLOCK_ERROR:
        case DEBUG_CODE_SENSORS_COMERROR_SENDERRORPROCEDURE_ERROR:
        case DEBUG_CODE_SENSORS_COMERROR_RECEIVEERRORPROCEDURE_ERROR:
        case DEBUG_CODE_SENSORS_WRONGVALUE_ERROR:
            return font_color_wrong;
            break;
        default:
            font_color;
            break;

    }
    return font_color;
}
const char* debug_decode_string(uint8_t code) {
    switch(code) {
        case DEBUG_CODE_EMPTY:
            return "";
            break;
        case DEBUG_CODE_NEWLINE:
            return " ";
            break;
        case DEBUG_CODE_OVERFLOW:
            return " - DEBUG OVERFLOW - ";
            break;
        case DEBUG_CODE_INIT_5BAUD:
            return "Initiating 5 baud";
            break;
        case DEBUG_CODE_INIT_HANDSHAKE_ERROR:
            return "Handshake: Nothing received";
            break;
        case DEBUG_CODE_INIT_HANDSHAKE_WRONG:
            return "Handshake: Wrong values";
            break;
        case DEBUG_CODE_INIT_HANDSHAKE_CORRECT:
            return "Handshake correct";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_SIZE_ERROR:
            return "ReceiveBlock invalid maxsize";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_EMPTYREAD_ERROR:
            return "ReceiveBlock AVA=0 or BUF=0";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_COMERROR_WARN:
            return "ReceiveBlock COM error";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_BLOCKCOUNTER_ERROR:
            return "ReceiveBlock wrong blockctr";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_TIMEOUT_ERROR:
            return "ReceiveBlock: Timeout";
            break;
        case DEBUG_CODE_KWPRECEIVEBLOCK_TIMEOUT_NOCONNECTION_ERROR:
            return "Check wires Received 0 data";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_ERROR:
            return "Connectblocks error";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_CORRECT:
            return "Connectblocks correct";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_RECEIVE_ERROR:
            return "Connectblocks receive error";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_SIZEZERO_ERROR:
            return "Connectblocks size = 0";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_WRONGANSWER_ERROR:
            return "Connectblocks Expected 0xF6";
            break;
        case DEBUG_CODE_CONNECTBLOCKS_SENDACK_ERROR:
            return "Connectblocks Send ACK fail";
            break;
        case DEBUG_CODE_SENDBLOCK_INVALIDCOMPLEMENT_ERROR:
            return "Sendblock wrong complement";
            break;
        case DEBUG_CODE_INIT_CONNECTED:
            return "Connected to ECU :)";
            break;
        case DEBUG_CODE_INIT_SIMULATION_ACTIVE:
            return "Simulation mode is active";
            break;
        case DEBUG_CODE_INIT_MULTIPLECONNECTATTEMPTS:
            return "Not first conection attempt";
            break;
        case DEBUG_CODE_DISCONNECTED_WARN:
            return "Disconnected";
            break;
        case DEBUG_CODE_DISCONNECTED_ERROR:
            return "Disconnected due to error";
            break;
        case DEBUG_CODE_RECEIVEACK_ERROR:
            return "Receiveblock ACK fail";
            break;
        case DEBUG_CODE_RECEIVEACK_WRONGVALUE_ERROR:
            return "Receiveblock buf[2] != 0x09";
            break;
        case DEBUG_CODE_KEEPALIVE_ERROR:
            return "Mode Keepalive failed";
            break;
        case DEBUG_CODE_DTC_INITBLOCKSEND_ERROR:
            return "Read DTC init send error";
            break;
        case DEBUG_CODE_DTC_RECEIVEBLOCK_ERROR:
            return "Read DTC receive failed";
            break;
        case DEBUG_CODE_DTC_WRONGVALUE_ERROR:
            return "Read DTC wrong value";
            break;
        case DEBUG_CODE_DTC_DTCCODESEMTPY:
            return "Read DTC no error codes :)";
            break;
        case DEBUG_CODE_DTC_SENDACK_ERROR:
            return "Read DTC send ACK failed";
            break;
        case DEBUG_CODE_DTC_CORRECT:
            return "Read DTC sucessful";
            break; 
        case DEBUG_CODE_DTCDELETE_SENDBLOCK_ERROR:
            return "Delete DTC send block fail";
            break;
        case DEBUG_CODE_DTCDELETE_RECEIVEBLOCK_ERROR:
            return "Delete DTC receive block";
            break;
        case DEBUG_CODE_DTCDELETE_WRONGVALUE_ERROR:
            return "Delete DTC s[2] != 0x09";
            break;
        case DEBUG_CODE_KWPEXIT_STARTING:
            return "Manual exit started";
            break;
        case DEBUG_CODE_KWPEXIT_SENDBLOCK_ERROR:
            return "Exit send block error";
            break;
        case DEBUG_CODE_KWPEXIT_CORRECT:
            return "Exit done: disconnected";
            break;
        case DEBUG_CODE_SENSORS_SENDBLOCK_ERROR:
            return "Sensors send block failed";
            break;
        case DEBUG_CODE_SENSORS_RECEIVEBLOCK_ERROR:
            return "Sensors receive block failed";
            break;
        case DEBUG_CODE_SENSORS_COMERROR_WARN:
            return "Sensors comerror warn";
            break;
        case DEBUG_CODE_SENSORS_COMERROR_SENDERRORPROCEDURE_ERROR:
            return "Sensors comerror in send";
            break;
        case DEBUG_CODE_SENSORS_COMERROR_RECEIVEERRORPROCEDURE_ERROR:
            return "Sensors comerror in receive";
            break;
        case DEBUG_CODE_SENSORS_WRONGVALUE_ERROR:
            return "Sensors s[2] != 0xE7";
            break;
        default:
            break;

    }
    return "";
}
void debug_add(uint8_t msg_code) {
    debug_messages_as_code[debug_messages_current] = msg_code;
    debug_messages_current++;
    if (debug_messages_current >= debug_messages_as_code_length) {
        debug_messages_current = 0; // Reset to 0 when max length reached.
        debugln(F("DEBUG OVERFLOW, OVERWRITING!"));
        debug_messages_overflow_counter++;
        debug_add(1); // DEBUG RESET, 0 was overwritten!! = 1
    } 
}
const char* debug_get_string(unsigned int pos) {
    if (pos >= debug_messages_as_code_length)
        return "";
    uint8_t code = debug_messages_as_code[pos];
    return debug_decode_string(code);
}
void debug_print_serial() {
    for (uint8_t i = 0; i < debug_messages_as_code_length; i++)
    {
        Serial.println(debug_get_string(i));
    }

}

// Maybe not needed?
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
bool check_msg_length(String msg)
{
    return msg.length() <= 30;
}
byte get_engine_rpm_lines(int engine_rpm_temp)
{
    if (engine_rpm_temp <= 0)
        return 0;
    else if (engine_rpm_temp < 1000)
        return 1;
    else if (engine_rpm_temp < 2000)
        return 2;
    else if (engine_rpm_temp < 3000)
        return 3;
    else if (engine_rpm_temp < 4000)
        return 4;
    else if (engine_rpm_temp < 5000)
        return 5;
    else if (engine_rpm_temp < 6000)
        return 6;
    else
        return 7;
}


