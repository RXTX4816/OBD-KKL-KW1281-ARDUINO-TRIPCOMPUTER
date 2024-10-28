#include <KLineKWP1281Lib.h>
#include <EEPROM.h>
#include "UTFT.h"
#include "config.h"
#include "communication.h"

#include "joystick.h"


#include "debug_codes.h"
#include "obdisplay.h"
#include "display.h"

#if debug_info
  KLineKWP1281Lib diag(beginFunction, endFunction, sendFunction, receiveFunction, TX_pin, is_full_duplex, &Serial);
#else
  KLineKWP1281Lib diag(beginFunction, endFunction, sendFunction, receiveFunction, TX_pin, is_full_duplex);
#endif

void reset() {
    
    block_counter = 0;
    connected = false;
    odometer_start = 0;
    fuel_level_start = 0;
    menu = DEFAULT_UI_MENU;
    menu_last = menu;
    menu_switch = false;

    delay(865);
    g.fillScr(back_color);
    g.setColor(font_color);

    debug_add(DEBUG_CODE_NEWLINE);

}

// Lib
/**
 * @brief Disconnect from the ECU. Reset all necessary values.
 *
 * TODO implement correct disconnect procedure!
 *
 */
void disconnect()
{

    diag.disconnect();
    //serial_end();
    debug(F("Disconnected. Block counter: "));
    debug(block_counter);
    debug(F(". Connected: "));
    debug(connected);
    debug(F(". Available: "));
    debugln(Serial3.available());

    // TODO differentiate between WARN and ERROR
    debug_add(DEBUG_CODE_DISCONNECTED_WARN);
    debug_add(DEBUG_CODE_NEWLINE);

    reset();
}


void init_eeprom() {
    // EEPROM setup
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t temp_EEPROM_read = EEPROM.read(i);
        switch (temp_EEPROM_read)
        {
        case 0xFF:
            // Never initialized
            is_first_EEPROM_setup = true;
            debugstrnumln("EEPROM write 0x00 to ", i);
            EEPROM.write(i, 0x00);
            break;
        default:
            break;
        }
    }
    // EEPROM read and store in array
    for (uint8_t i = 0; i < 4; i++) {
        EEPROM_values[i] = EEPROM.read(i);
        debugstrnum("EEPROM ", i);
        debugstrnumln(": ", EEPROM_values[i]);
    }


    if (is_first_EEPROM_setup) {
        g.print("EEPROM not initialized", CENTER, rows[5]);
        g.print("--------", CENTER, rows[6]);
        g.print("EEPROM 1-4 reset to 0x00", CENTER, rows[7]);
        clearRow(5);
        clearRow(6);
        clearRow(7);
    }
}



bool read_DTC_codes() {

  Serial.println("DTC read not supported");
  return true;
}
bool delete_DTC_codes() {
  Serial.println("DTC delete not supported");
  return true;
}

uint8_t measurements[29];
/**
 * @brief Perform a measurement group reading and safe the value to the corresponding variable.
 * Each group contains 4 values. Refer to your Label File for further information and order.
 * @param group The group to read
 * @return true no errors
 * @return false errors
 */
bool read_sensors(uint8_t block)
{
    // debugstrnum(F(" - ReadSensors group "), group);
    if (simulation_mode_active) {
        simulate_values();
        return true;
    }

    // This will contain the amount of measurements in the current block, after calling the readGroup() function.
    uint8_t amount_of_measurements = 0;
    
    /*
        The readGroup() function can return:
        *KLineKWP1281Lib::SUCCESS - received measurements
        *KLineKWP1281Lib::FAIL    - the requested block does not exist
        *KLineKWP1281Lib::ERROR   - communication error
    */
    
    // Read the requested group and store the return value.
    KLineKWP1281Lib::executionStatus readGroup_status = diag.readGroup(amount_of_measurements, block, measurements, sizeof(measurements));
    
    block_counter+=2;
    
    // Check the return value.
    switch (readGroup_status)
    {
        case KLineKWP1281Lib::ERROR:
        Serial.println("Error reading measurements!");
        return false;
        
        case KLineKWP1281Lib::FAIL:
        Serial.print("Block ");
        Serial.print(block);
        Serial.println(" does not exist!");
        return false; // TODO
        
        // Execute the code after the switch().
        case KLineKWP1281Lib::SUCCESS:
        break;
    }
    
    // If the block was read successfully, display its measurements.
    //Serial.print("Block ");
    //Serial.print(block);
    //Serial.println(':');
        
    // Display each measurement.
    for (uint8_t i = 0; i < 4; i++)
    {
        // Format the values with a leading tab.
        //Serial.print('\t');
        
        /*
        The getMeasurementType() function can return:
            *KLineKWP1281Lib::UNKNOWN - index out of range (measurement doesn't exist in block)
            *KLineKWP1281Lib::VALUE   - regular measurement, with a value and units
            *KLineKWP1281Lib::TEXT    - text measurement
        */
        
        //Get the current measurement's type and check the return value.
        switch (KLineKWP1281Lib::getMeasurementType(i, amount_of_measurements, measurements, sizeof(measurements)))
        {
        // "Value and units" type
        case KLineKWP1281Lib::VALUE:
        {
            // This will hold the measurement's units.
            char units_string[16];
            
            // Display the calculated value, with the recommended amount of decimals.
            //Serial.print(KLineKWP1281Lib::getMeasurementValue(i, amount_of_measurements, measurements, sizeof(measurements)),
            //            KLineKWP1281Lib::getMeasurementDecimals(i, amount_of_measurements, measurements, sizeof(measurements)));
            
            // The function getMeasurementUnits() returns the same string that it's given. It's the same as units_string.
            //Serial.print(' ');


            double v = KLineKWP1281Lib::getMeasurementValue(i, amount_of_measurements, measurements, sizeof(measurements));
            //Serial.print(v);
            //Serial.print(" ");
            switch (addr_selected)
            {
            case ADDR_INSTRUMENTS:
                switch (block) {
                    case 1:
                        switch (i) {
                            case 0: // 0.0 km/h Speed
                                if (vehicle_speed != (uint16_t)v)
                                {
                                    vehicle_speed = (uint16_t)v;
                                    vehicle_speed_updated = true;
                                }
                                break;
                            case 1: // 0 /min Engine Speed
                            if (engine_rpm != (uint16_t)v)
                                    {
                                        engine_rpm = (uint16_t)v;
                                        engine_rpm_updated = true;
                                    }                                
                                    
                                break;
                            case 2: // Oil Pr. 2 < min (Oil pressure 0.9 bar)
                                if (oil_pressure_min != (uint16_t)v)
                                {
                                    oil_pressure_min = (uint16_t)v;
                                    oil_pressure_min_updated = true;
                                }
                                break;
                            case 3: // 21:50 Time
                                
                                if (time_ecu != (uint32_t)v)
                                {
                                    time_ecu = (uint32_t)v;
                                    time_ecu_updated = true;
                                }
                                break;
                            default:
                            break;
                        }
                        break;
                    case 2:
                        switch (i) {
                            case 0: // Odometer
                                if (odometer != (uint32_t)v)
                                {
                                    odometer = (uint32_t)v;
                                    odometer_updated = true;
                                }
                                if (millis() - timer_start[TIMER_CODE_OBD_CONNECTIONSTART] < 10000)
                                {
                                    odometer_start = odometer;
                                }
                                break;
                            case 1: // 9.0 l Fuel level
                                if (fuel_level != (uint16_t)v)
                                {
                                    fuel_level = (uint16_t)v;
                                    fuel_level_updated = true;
                                }
                                if (millis() - timer_start[TIMER_CODE_OBD_CONNECTIONSTART] < 10000)
                                {
                                    fuel_level_start = fuel_level;
                                }
                                break;
                            case 2: // 93 ohms Fuel Sender Resistance
                                if (fuel_sensor_resistance != (uint16_t)v)
                                {
                                    fuel_sensor_resistance = (uint16_t)v;
                                    fuel_sensor_resistance_updated = true;
                                }
                                break;
                            case 3: // 0.0°C Ambient Temperature
                                if (ambient_temp != (uint8_t)v)
                                {
                                    ambient_temp = (uint8_t)v;
                                    ambient_temp_updated = true;
                                }
                                break;
                            default:
                            break;
                        }
                        break;
                    case 3:
                        switch (i) {
                            case 0: // 12.0°C Coolant temp
                                if (coolant_temp != (uint8_t)v)
                                {
                                    coolant_temp = (uint8_t)v;
                                    coolant_temp_updated = true;
                                }
                                break;
                            case 1: // OK Oil Level (OK/n.OK)
                                if (oil_level_ok != (uint8_t)v)
                                {
                                    oil_level_ok = (uint8_t)v;
                                    oil_level_ok_updated = true;
                                }
                                break;
                            case 2: // 11.0°C Oil temp
                                if (oil_temp != (uint8_t)v)
                                {
                                    oil_temp = (uint8_t)v;
                                    oil_temp_updated = true;
                                }
                                break;
                            case 3: // N/A
                            default:
                            break;
                        }
                        break;
                    default:
                        Serial.println("group not supported"); // Implement your own solution
                        break;

                }
                break;
            
            default:
                Serial.println("Please implement the desired address!"); // Implement your own solution
                break;
            }
        }
        break;
        
        // "Text" type
        case KLineKWP1281Lib::TEXT:
        {
            // This will hold the measurement's text.
            //char text_string[16];
            
            // The function getMeasurementText() returns the same string that it's given. It's the same as text_string.
            //Serial.println(KLineKWP1281Lib::getMeasurementText(i, amount_of_measurements, measurements, sizeof(measurements), text_string, sizeof(text_string)));
        }
        break;
        
        // Invalid measurement index
        case KLineKWP1281Lib::UNKNOWN:
            //Serial.println("N/A");
            break;
        }
    }

    // Leave an empty line.
    //Serial.println();

    return true;

}


/**
 * KW1281 procedure to send a simple acknowledge block to keep the connection alive
 */
bool keep_alive(bool expect_response_ack = true)
{
    if (simulation_mode_active) {
        delay(100);
        return true;
    }

    timer_begin(TIMER_CODE_OBD_UPDATE);
    diag.update();
    block_counter+=2;

    timer_done(TIMER_CODE_OBD_UPDATE);
    if (timer_duration[TIMER_CODE_OBD_UPDATE] > timeout_to_add) {
        debug_add(DEBUG_CODE_KEEPALIVE_ERROR);
        return false;
    }
    return true;
}

bool obd_connect()
{
    debugln(F("Connecting to ECU"));
    block_counter = 0;

    debugln(F("Init "));

    debug_add(DEBUG_CODE_INIT_5BAUD);
    debugln(F("Using KLineKWP1281Lib to establish connection.."));

    // Attempt Connection to ECU
    if (diag.attemptConnect(get_userinput_ecu_address(), baud_rate, false) != KLineKWP1281Lib::SUCCESS) {
        debugln(F("Error"));
        delay(1300);
        return false;
    }

    draw_status_bar();
    debug_add(DEBUG_CODE_INIT_HANDSHAKE_CORRECT);
    debug_add(DEBUG_CODE_CONNECTBLOCKS_CORRECT);
    debugln(F(" "));
    debugln(F("------"));
    debugln(F("ECU connected"));
    debugln(F("------"));
    // Display the module's part number and identification.
    Serial.println();
    Serial.println(diag.getPartNumber());
    Serial.println(diag.getIdentification());
    // If it is available, display the module's extra identification.
    if (strlen(diag.getExtraIdentification()))
    {
        Serial.println(diag.getExtraIdentification());
    }
    // Put the module's coding value into a string and pad with 0s to show 5 characters.
    //char coding_str[6];
    //unsigned int coding = diag.getCoding();
    //sprintf(coding_str, "%05u", coding);
    
    // Put the module's wokshop code into a string and pad with 0s to show 5 characters.
    //char WSC_str[6];
    //unsigned long WSC = diag.getWorkshopCode();
    //sprintf(WSC_str, "%05lu", WSC);
    
    // Display the module's coding value and workshop code.
    //Serial.println();
    //Serial.print("Coding: ");
    //Serial.println(coding_str);
    //Serial.print("WSC: ");
    //Serial.println(WSC_str);
    //Serial.println();
    debugln(F("------"));

    connected = true;
    debug_add(DEBUG_CODE_INIT_CONNECTED);
    // draw_status_bar();
    g.setColor(font_color_correct);
    g.print("Connected!", 325, rows[15]);
    g.setColor(font_color);
    return true;
}


uint8_t userinput_current_row = 18;
uint8_t userinput_previous_row = userinput_current_row;
bool connect()
{

    init_status_bar();
    draw_status_bar();

    init_setup_config(get_userinput_ecu_address(), userinput_baudrate, userinput_debug_mode, userinput_current_row, userinput_simulation_mode, userinput_night_mode); // TODO not good if 10 hardcoded

    bool user_pressed_connect = false;
    bool setup_config_button_pressed = false;
    bool debug_screen_active = false;
    while (!user_pressed_connect)
    {
        draw_status_bar();

        if (!debug_screen_active)
        {

            // User input
            if (up_click())
            {
                userinput_previous_row = userinput_current_row;
                if (userinput_current_row <= 10)
                {
                    userinput_current_row = 18;
                }
                else if (userinput_current_row > 15 && userinput_current_row <= 17)
                {
                    userinput_current_row = 15;
                }
                else if (userinput_current_row == 15) {
                    userinput_current_row = 13;
                }
                else
                {
                    userinput_current_row--;
                }
                setup_config_button_pressed = true;
            }
            else if (down_click())
            {
                userinput_previous_row = userinput_current_row;
                if (userinput_current_row < 13 || userinput_current_row == 17)
                {
                    userinput_current_row++;
                }
                else if (userinput_current_row == 13)
                {
                    userinput_current_row = 15;
                }
                else if (userinput_current_row == 15) {
                    userinput_current_row = 17;
                }
                else
                {
                    userinput_current_row = 10;
                }
                setup_config_button_pressed = true;
            }
            else if ((userinput_current_row == 10) && (right_click() || left_click() || mid_click()))
            {
                // Boolean switch
                userinput_simulation_mode = !userinput_simulation_mode;
                setup_config_button_pressed = true;
            }
            else if ((userinput_current_row == 12) && (right_click() || left_click() || mid_click()))
            {
                // Boolean switch
                userinput_debug_mode = !userinput_debug_mode;
                setup_config_button_pressed = true;
            }
            else if ((userinput_current_row == 13) && (right_click() || left_click()))
            {
                increment_ecu_address();
                setup_config_button_pressed = true;
            }
            else if ((userinput_current_row == 15) && (right_click() || left_click() || mid_click()))
            {
                userinput_night_mode = !userinput_night_mode;

                remove_setup_config(userinput_current_row);
                remove_status_bar();
                init_colors(userinput_night_mode);
                g.setColor(font_color);
                g.setBackColor(back_color);
                g.fillScr(back_color);
                init_status_bar();
                init_setup_config(userinput_simulation_mode, userinput_baudrate, userinput_debug_mode, userinput_current_row, userinput_simulation_mode, userinput_night_mode);

                setup_config_button_pressed = true;
            }
            else if (right_click() && userinput_current_row == 11)
            {
                if (supported_baud_rates_counter >= supported_baud_rates_max)
                    supported_baud_rates_counter = 0;
                else
                    supported_baud_rates_counter++;
                userinput_baudrate = supported_baud_rates[supported_baud_rates_counter];
                setup_config_button_pressed = true;
            }
            else if (left_click() && userinput_current_row == 11)
            {
                if (supported_baud_rates_counter <= 0)
                    supported_baud_rates_counter = supported_baud_rates_max;
                else
                    supported_baud_rates_counter--;
                userinput_baudrate = supported_baud_rates[supported_baud_rates_counter];
                setup_config_button_pressed = true;
            }
            else if (mid_click() && userinput_current_row == 18)
            {
                switch(get_userinput_ecu_address()) {
                    case 0x01:     // Engine
                    case 0x08:     // Climatronic
                    case 0x17:    // Instruments
                    case 0x45:    // Comfort
                        user_pressed_connect = true;
                        g.print("->Connect<-", CENTER, rows[18]);
                        break;
                    default:
                        g.setColor(font_color_wrong);
                        for (int i = 0; i < 5; i++)
                        {
                            g.print("!ECU Addr not supported!", CENTER, rows[17]);
                            delay(333);
                            g.print("                        ", CENTER, rows[17]);
                            delay(33);
                        }
                        g.setColor(font_color);
                        break;
                }
                setup_config_button_pressed = true;
            }
            else if (mid_click() && userinput_current_row == 17)
            {
                // DEBUG SCREEN engage
                remove_setup_config(userinput_current_row);
                init_debug(false, true);
                debug_screen_active = true;
                continue;
            }
            else if ((set_click() || reset_click()) && userinput_current_row == 17)
            {
                // Clear debug messages
                debug_clear();
                continue;
            }

            // Draw screen changes
            update_setup_config(userinput_current_row, userinput_previous_row);
        } else {
            if (down_click()) {
                if ((debug_page_current + 1) * debug_page_row_distance >= debug_messages_as_code_length)
                {
                    remove_debug(true);
                    debug_page_current = 0;
                    init_debug(false, true);
                }
                else
                {
                    remove_debug(true);
                    debug_page_current++;
                    init_debug(false, true);
                    //delay(button_press_delay);
                }
            } else if (up_click()) {
                if (debug_page_current > 0) {
                    remove_debug(true);
                    debug_page_current--;
                    init_debug(false, true);
                    //delay(button_press_delay);
                }
            } else if (left_click() ||right_click()) {
                // Exit
                remove_debug(true);
                init_setup_config(get_userinput_ecu_address(), userinput_baudrate, userinput_debug_mode, userinput_current_row, userinput_simulation_mode, userinput_night_mode);
                debug_screen_active = false;
            }
        }

        // Userinput delay
        if (setup_config_button_pressed)
        {
            setup_config_button_pressed = false;
            delay(333);
        }
    }
    // Update values
    simulation_mode_active = userinput_simulation_mode;
    baud_rate = userinput_baudrate;
    debug_mode_enabled = userinput_debug_mode;
    NIGHT_MODE_ACTIVE = userinput_night_mode;
    addr_selected = get_userinput_ecu_address();
    
    remove_setup_config(userinput_current_row); // Clear used rows

    debugln(F("Saved configuration: "));
    debugstrnumln(F("--- DEBUG_MODE "), debug_mode_enabled);
    debugstrnumln(F("--- SIMULATION "), simulation_mode_active);
    debugstrnumln(F("--- baud "), baud_rate);
    debugstrnumln(F("--- addr "), get_userinput_ecu_address());
    debugstrnumln(F("Connect attempt: "), connection_attempts_counter);
    if (connection_attempts_counter > 0)
        debug_add(DEBUG_CODE_INIT_MULTIPLECONNECTATTEMPTS);
    

    // Connect to ECU
    connection_attempts_counter++;
    if (!simulation_mode_active && !obd_connect())
    {
        return false;
    }
    if (simulation_mode_active)
        connected = true;
    else {
      // Clear 
      destroy_connect();
    }
    timer_begin(TIMER_CODE_OBD_CONNECTIONSTART);
    menu_switch = true;
    return true;
}

void init_obdisplay() {
    pinMode(TX_pin, OUTPUT); // KLine interface expects a default of HIGH on the TX pin
    digitalWrite(TX_pin, HIGH);
    init_joystick(); // Joystick
    init_colors(true); // true = NIGHT mode , false = Eyeburn mode
    init_eeprom(); // EEPROM
    debug_clear(); // Custom debug messages
    reset_dtc_status_errors_array(); // DTC errors
    timer_start[TIMER_CODE_STARTUP_ANIMATION] = millis();
    init_display(LANDSCAPE, back_color, font_color); // Start display
    timer_end[TIMER_CODE_STARTUP_ANIMATION] = millis();
    //debugstrnumln(F("Startup animation duration: "), timer_end[TIMER_CODE_STARTUP_ANIMATION] - timer_start[TIMER_CODE_STARTUP_ANIMATION]);
    //debugln(F("Durations are in ms. 1 sec = 1.000 ms = 1.000.000 µs"));
    //debugln(F("Timer measurements complete. --------"));
    /* ------------------------------------------------------------------------------------ */
}
void init_klinelib() {
    // If debugging bus traffic was enabled, attach the debugging function.
    #if debug_traffic
        debugln("debug_traffic enabled");
      diag.KWP1281debugFunction(KWP1281debugFunction);
    #endif
    diag.customErrorFunction(errorFunction);
}
void setup() {
    
  #if DEBUG == 1 // USB Debug
      Serial.begin(115200);
  #endif 

    init_obdisplay(); // OBDisplay
    
    init_klinelib(); // KLineKWP1281Lib
    
    debugln(F("---------------------"));
    debugln(F("----- OBDisplay -----"));
    debug(F("----- Ver. "));
    debug(_VERSION);
    debugln(F("  -----"));
    debugln(F("---------------------"));
    return;
}

void showMeasurements(uint8_t block){
  // This will contain the amount of measurements in the current block, after calling the readGroup() function.
  uint8_t amount_of_measurements = 0;
  
  /*
    The readGroup() function can return:
      *KLineKWP1281Lib::SUCCESS      - received measurements
      *KLineKWP1281Lib::FAIL         - the requested block does not exist
      *KLineKWP1281Lib::ERROR        - communication error
      *KLineKWP1281Lib::GROUP_HEADER - received header for a "header+body" measurement; need to read again
  */
  
  // Read the requested group and store the return value.
  KLineKWP1281Lib::executionStatus readGroup_status = diag.readGroup(amount_of_measurements, block, measurements, sizeof(measurements));
  
  // Check the return value.
  switch (readGroup_status)
  {
    case KLineKWP1281Lib::ERROR:
      Serial.println("Error reading measurements!");
      return;
    
    case KLineKWP1281Lib::FAIL:
      Serial.print("Block ");
      Serial.print(block);
      Serial.println(" does not exist!");
      return;
    
    // Execute the code after the switch().
    case KLineKWP1281Lib::SUCCESS:
      break;
  }
  
  // If the block was read successfully, display its measurements.
  Serial.print("Block ");
  Serial.print(block);
  Serial.println(':');
    
  // Display each measurement.
  for (uint8_t i = 0; i < 4; i++)
  {
    // Format the values with a leading tab.
    Serial.print('\t');
    
    /*
      The getMeasurementType() function can return:
        *KLineKWP1281Lib::UNKNOWN - index out of range (measurement doesn't exist in block)
        *KLineKWP1281Lib::VALUE   - regular measurement, with a value and units
        *KLineKWP1281Lib::TEXT    - text measurement
    */
    
    //Get the current measurement's type and check the return value.
    switch (KLineKWP1281Lib::getMeasurementType(i, amount_of_measurements, measurements, sizeof(measurements)))
    {
      // "Value and units" type
      case KLineKWP1281Lib::VALUE:
      {
        // This will hold the measurement's units.
        char units_string[16];
        
        // Display the calculated value, with the recommended amount of decimals.
        Serial.print(KLineKWP1281Lib::getMeasurementValue(i, amount_of_measurements, measurements, sizeof(measurements)),
                     KLineKWP1281Lib::getMeasurementDecimals(i, amount_of_measurements, measurements, sizeof(measurements)));
        
        // The function getMeasurementUnits() returns the same string that it's given. It's the same as units_string.
        Serial.print(' ');
        Serial.println(KLineKWP1281Lib::getMeasurementUnits(i, amount_of_measurements, measurements, sizeof(measurements), units_string, sizeof(units_string)));
      }
      break;
      
      // "Text" type
      case KLineKWP1281Lib::TEXT:
      {
        // This will hold the measurement's text.
        char text_string[16];
        
        // The function getMeasurementText() returns the same string that it's given. It's the same as text_string.
        Serial.println(KLineKWP1281Lib::getMeasurementText(i, amount_of_measurements, measurements, sizeof(measurements), text_string, sizeof(text_string)));
      }
      break;
      
      // Invalid measurement index
      case KLineKWP1281Lib::UNKNOWN:
        Serial.println("N/A");
        break;
    }
  }

  // Leave an empty line.
  Serial.println();
}

#define SCHEDULER_DEBUG_DIRECTION_NOTDEFINED 0
#define SCHEDULER_DEBUG_DIRECTION_MINUS 1
#define SCHEDULER_DEBUG_DIRECTION_PLUS 2
#define SCHEDULER_DEBUG_DIRECTION_RESET 3
#define SCHEDULER_DEBUG_DIRECTION_INIT 4
bool scheduler_debug_isrunning = false;
bool scheduler_debug_removedone = false;
bool scheduler_debug_directiondone = false;
bool scheduler_debug_isrunning_removeonly = false;
uint8_t scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_NOTDEFINED;
uint8_t current_group_scheduler = 1;
// Cockpit scheduler
bool lock_user_input_for_scheduler = false;
void loop() {
    timer_begin(TIMER_CODE_START_FRAME);
    draw_status_bar();

    timer_begin(TIMER_CODE_OBD_CONNECT);    
    if (!connected && !connect())
    {
        reset();
        return;
    }
    timer_done(TIMER_CODE_OBD_CONNECT); 

    // Update values
    timer_begin(TIMER_CODE_OBD_MODE);
    if (!simulation_mode_active)
    {
        switch (kwp_mode)
        {
        case KWP_MODE_ACK:
            if (!keep_alive())
            {
                disconnect();
                return;
            }
            break;
        case KWP_MODE_READGROUP:
            if (!read_sensors(kwp_group))
            {
                disconnect();
                return;
            }
            break;
        case KWP_MODE_READSENSORS:
            // Read the sensor groups
            if (!read_sensors(current_group_scheduler))
                {
                    disconnect();
                    return;
                }
            if (current_group_scheduler >= 3) {
                current_group_scheduler = 1;
            } else {
                current_group_scheduler++;
            }
            break;
        default:
            debugln(F("Kwp_mode undefined EXIT"));

            break;
        }
    }
    timer_done(TIMER_CODE_OBD_MODE);

    // Compute stats
    compute_values();


    // User input 
    if ((!lock_user_input_for_scheduler) && millis()-timer_start[TIMER_CODE_BUTTON_PRESSED] > BUTTON_PRESS_DELAY && !scheduler_debug_isrunning_removeonly && !scheduler_debug_isrunning)
    {
        bool button_pressed_temp = false;
        if (left_click())
        {
            if (menu == 2)
            {
                scheduler_debug_isrunning_removeonly = true;
            }
            decrement_menu();
            button_pressed_temp = true;
        }
        else if (right_click())
        {
            if (menu == 2)
            {
                scheduler_debug_isrunning_removeonly = true;
            }
            increment_menu();
            button_pressed_temp = true;
        }
        else
        {
            switch (menu)
            {
            case 0:
                // Cockpit has 2 pages
                if (up_click() || down_click()) {
                    // Currently, page 2 is disabled
                    //remove_menu_cockpit();
                    //cockpit_menu_page_switch = !cockpit_menu_page_switch; 
                    //init_menu_cockpit(); 
                    //display_menu_cockpit(true);
                    //button_pressed_temp = true; 
                }
                break;
            case 1:
                // Experimental has 2 pages
                if (up_click() || down_click()) {
                    remove_menu_experimental();
                    experimental_menu_page_switch = !experimental_menu_page_switch; 
                    init_menu_experimental(); 
                    display_menu_experimental(true);
                    button_pressed_temp = true; 
                }
                break;
            case 2:
                // Debug TODO may be too slow for ECU
                if (up_click())
                {
                    if (debug_page_current > 0)
                    {
                        scheduler_debug_isrunning = true;
                        scheduler_debug_removedone = false;
                        scheduler_debug_directiondone = false;
                        scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_MINUS;
                        //delay(button_press_delay);
                    }
                    button_pressed_temp = true;
                }
                else if (down_click())
                {
                    if ((debug_page_current + 1) * debug_page_row_distance >= debug_messages_as_code_length)
                    {
                        scheduler_debug_isrunning = true;
                        scheduler_debug_removedone = false;
                        scheduler_debug_directiondone = false;
                        scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_RESET;
                    }
                    else
                    {
                        scheduler_debug_isrunning = true;
                        scheduler_debug_removedone = false;
                        scheduler_debug_directiondone = false;
                        scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_PLUS;
                    }
                    
                    button_pressed_temp = true;
                }
                break;
            case 3:
                // DTC menu
                if (mid_click())
                {
                    switch (dtc_menu_current_row)
                    {
                    case 16:
                        display_menu_dtc(true);
                        reset_dtc_status_errors_array();
                        display_menu_dtc();
                        button_pressed_temp = true;
                        break;
                    case 17:
                        display_menu_dtc(true);
                        if (!read_DTC_codes()) {
                            disconnect();
                            return;
                        }
                        display_menu_dtc();
                        button_pressed_temp = true;
                        break;
                    case 18:
                        display_menu_dtc(true);
                        if(delete_DTC_codes()) {
                            reset_dtc_status_errors_array();
                            disconnect();
                            return;
                        }
                        display_menu_dtc();
                        button_pressed_temp = true;
                        break;
                    }
                } else if (down_click()) {
                    dtc_menu_last_row = dtc_menu_current_row;
                    if (dtc_menu_current_row >= 18)
                        dtc_menu_current_row = 16;
                    else 
                        dtc_menu_current_row++;
                    display_menu_dtc();

                } else if (up_click()) {
                    dtc_menu_last_row = dtc_menu_current_row;
                    if (dtc_menu_current_row <= 16)
                        dtc_menu_current_row = 18;
                    else 
                        dtc_menu_current_row--;
                    display_menu_dtc();
                }
                break;
            case 4:
                // Settings
                bool should_remove_settings_menu = false;
                if (menu_selected_setting == 0 && mid_click()) {
                    if (DEBUG_BENCHMARK) {
                        remove_benchmark();
                    } else {
                        remove_benchmark_minimalistic();
                    }
                    increment_menu_settings_value();

                    button_pressed_temp = true;
                }
                else if (set_click() && menu_selected_setting != 0) {
                    if (!decrement_menu_settings_value())
                        should_remove_settings_menu = true;
                    
                    button_pressed_temp = true;
                }
                else if (reset_click() && menu_selected_setting != 0) {
                    if (!increment_menu_settings_value())
                        should_remove_settings_menu = true;

                    button_pressed_temp = true;
                }
                else if (up_click()) {
                    decrement_menu_settings();
                    button_pressed_temp = true;
                }
                else if (down_click()) {
                    increment_menu_settings();
                    button_pressed_temp = true;
                }
                else if ((mid_click() && (menu_selected_setting == 1 || menu_selected_setting == 3 || menu_selected_setting == 4)) || ((set_click() || reset_click()) && menu_selected_setting == 1))
                    if (!increment_menu_settings_value()) {
                        should_remove_settings_menu = true;

                    button_pressed_temp = true;
                }
                
                // Go back to setup page
                if (should_remove_settings_menu) {
                    remove_menu_settings();
                    disconnect();
                    return;
                }

                break;
            }
        }

        if (button_pressed_temp)
            timer_start[TIMER_CODE_BUTTON_PRESSED] = millis(); // Set button pressed timer
    }

    if (scheduler_debug_isrunning_removeonly) {
        scheduler_debug_isrunning_removeonly = !remove_debug();
        timer_done(TIMER_CODE_START_FRAME);    
        if (timer_useable(TIMER_CODE_START_FRAME)) // Should be always true
            g.printNumI(timer_duration[TIMER_CODE_START_FRAME], RIGHT, rows[19], 4, '0');
        return;
    }


    // Perform menu switch or update values on current menu
    if (menu_switch)
    {
        timer_begin(TIMER_CODE_START_MENU_SWITCH);

        // Remove old menu
        if (menu_last != menu) {
            switch (menu_last)
            {
            case 0:
                if (!remove_menu_cockpit()) {
                    lock_user_input_for_scheduler = true;
                    return;
                } else {
                    lock_user_input_for_scheduler = false;
                }
                break;
            case 1:
                remove_menu_experimental();
                break;
            case 2:
                remove_debug();
                break;
            case 3:
                remove_menu_dtc();
                break;
            case 4:
                remove_menu_settings();
                break;
            case 5:
                remove_menu_cockpit_graphic();
                break;
            }
        }

        init_status_bar();
        switch (menu)
        {
        case 0:
            init_menu_cockpit();
            _amount_remove_menu_cockpit_called = 0;
            _idx_UI_menu_cockpit_update = 0;
            display_menu_cockpit(true);
            break;
        case 1:
            init_menu_experimental();
            display_menu_experimental(true);
            break;
        case 2:
            scheduler_debug_isrunning = true;
            scheduler_debug_removedone = false;
            scheduler_debug_directiondone = false;
            scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_INIT;
            break;
        case 3:
            init_menu_dtc();
            display_menu_dtc();
            break;
        case 4:
            init_menu_settings();
            break;
        case 5:
            init_menu_cockpit_graphic();
            display_menu_cockpit_graphic();
            break;
        }
        menu_switch = false;
        timer_done(TIMER_CODE_START_MENU_SWITCH);
    }
    else
    {

        if (millis() - timer_start[TIMER_CODE_START_DRAWSCREEN] > DISPLAY_FRAME_LENGTH)
        {
        timer_begin(TIMER_CODE_START_DRAWSCREEN);
            draw_status_bar();
            switch (menu)
            {
            case 0:
                display_menu_cockpit();
                break;
            case 1:
                display_menu_experimental();
                break;
            case 2:
                // No need since its all static
                break;
            case 3:
                //display_menu_dtc(); No need, static contenet
                break;
            case 4:
                display_menu_settings();
                break;
            case 5:
                display_menu_cockpit_graphic();
                break;
            }
        timer_done(TIMER_CODE_START_DRAWSCREEN);
        }
    }

    // Debug screen scheduler to minimize ECU timeout
    if (scheduler_debug_isrunning && scheduler_debug_direction != SCHEDULER_DEBUG_DIRECTION_NOTDEFINED && menu == 2) {

        if (scheduler_debug_direction != SCHEDULER_DEBUG_DIRECTION_INIT && !scheduler_debug_removedone)
        {
            scheduler_debug_removedone = remove_debug();
        }
        else
        {
            if (!scheduler_debug_directiondone)
            {
                switch (scheduler_debug_direction)
                {
                case SCHEDULER_DEBUG_DIRECTION_RESET:
                    debug_page_current = 0;
                    break;
                case SCHEDULER_DEBUG_DIRECTION_PLUS:

                    debug_page_current++;
                    break;
                case SCHEDULER_DEBUG_DIRECTION_MINUS:

                    debug_page_current--;
                    break;
                }
                scheduler_debug_directiondone = true;
            }

            if (init_debug())
            {
                // Reset scheduler
                scheduler_debug_isrunning = false;
                scheduler_debug_removedone = false;
                scheduler_debug_directiondone = false;
                scheduler_debug_direction = SCHEDULER_DEBUG_DIRECTION_NOTDEFINED;
            }
        }
    }
    else
    {
        if (DEBUG_BENCHMARK)
        {
            init_benchmark();
        }
        else
        {
            init_benchmark_minimalistic();
        }
    }

    return;
}