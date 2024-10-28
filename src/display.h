


// Display 480x320 in landscape mode
// Max chars per row with smallfont: 60
// Max chars rper row with bigfont: 30
#define DISPLAY_FRAME_LENGTH 40// Length of 1 frame in ms // 111 to be safe from timeout // 
#define DISPLAY_MAX_X 480
#define DISPLAY_WIDTH 480
#define DISPLAY_MAX_Y 320
#define DISPLAY_HEIGHT 320
#define DISPLAY_MAX_CHAR_LENGTH 30
const int rows[20] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304};
const int cols[30] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304, 320, 336, 352, 368, 384, 400, 416, 432, 448, 464};
UTFT g(CTE40, 38, 39, 40, 41); // Graphics g
// Fonts
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[]; // Declare which fonts we will be using
// INPUT
const uint16_t BUTTON_PRESS_DELAY = 333;




void clearRow(byte row)
{
    g.print(String("                              "), 1, rows[row]);
}

void startup_animation(word back_color, word font_color)
{
    g.fillScr(back_color);
    // for(int i = 0; i<20;i++) {
    //     g.print("OBDisplay", CENTER, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     delay(333);
    // }
    // clearRow(20);
    g.setColor(font_color);
    // int x = 0;
    // for(int i =0; i<20; i++) {
    //     g.print("O B D i s p l a y", x, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     x+=10;
    //     delay(10);
    // }
    g.setFont(BigFont);
    g.print("Welcome to", CENTER, rows[3]);
    g.print("OBDisplay", CENTER, rows[5]);
    g.setFont(SmallFont);
    String temp = "Version ";
    temp += _VERSION;
    g.print(temp, CENTER, rows[6]);
    g.setFont(BigFont);
    g.drawRect(4 + 2, rows[17], 474, rows[17] + 12);
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 59; j++)
        {
            if ((i == 0 && j < 2) || (i == 7 && j >= 57))
                continue;
            g.drawLine(4 + i * 59 + j, rows[17], 4 + i * 59 + j + 1, rows[17] + 12);
            delay(1);
        }
        // g.fillRect(4+i*59+j, rows[17], 4+i*59+59, rows[17]+12);
    }
    clearRow(17);

    delay(222);

    // g.fillScr(back_color);
    clearRow(3);
}

void init_display(byte orientation, word back_color, word font_color) {
    
    // Display
    g.InitLCD(orientation);
    g.clrScr();
    g.fillScr(back_color);
    // g.setFont(SmallFont);
    g.setBackColor(back_color);
    g.setColor(font_color);
    //g.setFont(SmallFont);

    startup_animation(back_color, font_color);
    g.setColor(font_color);
}


void init_status_bar(bool destroy = false)
{
    g.setFont(BigFont);
    if (destroy)
        g.setColor(back_color);
    g.print("AVA:   | BC: 0x  ", LEFT, rows[0]);
    g.printNumI(Serial3.available(), cols[4], rows[0], 1, '0');
    char block_counter_hex[2];
    sprintf(block_counter_hex, "%X", block_counter);
    g.print(block_counter_hex, cols[16], rows[0]);
    g.drawHLine(cols[0], rows[1]+1, DISPLAY_WIDTH);
    g.setColor(font_color);
}
void draw_status_bar()
{

    g.setFont(BigFont);
    if (block_counter != block_counter_last)
    {
        
        char block_counter_hex[2];
        sprintf(block_counter_hex, "%X", block_counter);
        g.print(block_counter_hex, cols[16], rows[0]);
        block_counter_last = block_counter;
    }
    if (Serial3.available() != available_last)
    {
        g.printNumI(Serial3.available(), cols[4], rows[0], 1, '0');
        available_last = Serial3.available();
    }
}
void remove_status_bar() {
    init_status_bar(true);
}


/*
    0: Cockpit 

    Default font: 7SegmentFont 

    Scheduler must be used to not timeout the ECU

    Top left: 
        km driven

    Top right:
        L fuel remaining
        using BigFont (
            l/100 km fuel consumption since connect
            l/100 km fuel consumption on last {100m, 1000m, 2000m, 5000m}
        ) 

    Middle:
        engine icon /!|!\ fuel station icon
        engine icon with ! is visible when engine is not in operating temperature
            -> Red when HOT
            -> Blue when COLD
        fuel station icon with ! is visible when the fuel tank shows less than 5 liters or the fuel consumption is very high

    Debug reserved bottom left and bottom right
*/
uint8_t cockpit_menu_page_switch = false;
void init_menu_cockpit(word color = font_color)
{
    g.setColor(color);

    if (cockpit_menu_page_switch)
    {
        g.print("KMH", LEFT, rows[3]);
        g.print("RPM", LEFT, rows[5]);

        g.print("COOL", CENTER, rows[3]);
        g.print("OIL", CENTER, rows[5]);
        g.print("OILLVL", CENTER, rows[7]);
        g.print("FUEL", CENTER, rows[9]);
        g.print("L/km", CENTER, rows[11]);
    }
    else
    {
        int middle_line_x = cols[15];
        g.drawLine(cols[9]+5, rows[14], cols[11]+5, rows[3]);
        g.drawLine(cols[21], rows[14], cols[19], rows[3]);
        g.drawLine(middle_line_x, rows[11], middle_line_x, rows[10]);
        g.drawLine(middle_line_x, rows[8], middle_line_x, rows[7]);
        g.drawLine(middle_line_x, rows[5], middle_line_x, rows[4]);

        //g.print("kmh", cols[3], rows[6]);
        //g.print("cool C", cols[23], rows[6]);

        //if (color != back_color)
            //g.setColor(font_color_dark);
        //g.print("oil C", cols[23], rows[12]);

        //if (color != back_color)
            //g.setColor(font_color_alt_other);
        //g.print("km", CENTER, rows[19]);
        //g.print("L", cols[28], rows[17]);

        //if (color != back_color)
            //g.setColor(font_color);
    }

    g.setColor(font_color);
}
uint8_t _idx_UI_menu_cockpit_update = 0;
uint8_t _max_UI_menu_cockpit_update_7seg = 3; // Only render the 7 segment in a scheduler, since bigfont does not have much of an impact 
// Returns whether the scheduler has finished a cycle
bool display_menu_cockpit(bool force_update = false, bool destroy = false)
{
    bool is_finished_cycle = false;

    if (destroy) {
        g.setColor(back_color);
        force_update = true;
    }

    // The second menu page is currently disabled until the scheduler is developed
    if (cockpit_menu_page_switch)
    {
        if (vehicle_speed_updated || force_update)
        {
            g.printNumI(vehicle_speed, cols[0] + cols[5], rows[3], 3, '0');
            vehicle_speed_updated = false;
        }
        if (engine_rpm_updated || force_update)
        {
            g.printNumI(engine_rpm, cols[0] + cols[5], rows[5], 4, '0');

            engine_rpm_updated = false;
        }
        if (coolant_temp_updated || force_update)
        {
            g.printNumI(coolant_temp, cols[15] + cols[4], rows[3], 3, '0');
            coolant_temp_updated = false;
        }
        if (oil_temp_updated || force_update)
        {
            g.printNumI(oil_temp, cols[15] + cols[4], rows[5], 3, '0');
            oil_temp_updated = false;
        }
        if (fuel_level_updated || force_update)
        {
            g.printNumI(fuel_level, cols[15] + cols[4], rows[9], 2, '0');
            fuel_level_updated = false;
        }
        if (oil_level_ok_updated || force_update)
        {
            g.printNumI(oil_level_ok, cols[15] + cols[4], rows[7], 1, '-');
            oil_level_ok_updated = false;
        }
        if (fuel_per_100km_updated || force_update)
        {
            int fuel_per_100km_display_temp = (int)fuel_per_100km;
            if (fuel_per_100km_display_temp > 99)
                fuel_per_100km_display_temp = 99;
            else if (fuel_per_100km_display_temp < 0)
                fuel_per_100km_display_temp = 0;
            g.printNumI(fuel_per_100km_display_temp, cols[15] + cols[4], rows[11], 2, '0');
            fuel_per_100km_updated = false;
        }
        if (oil_pressure_min_updated || force_update)
        {
            // g.printNumI(oil_level_ok, 120, rows[6]);
            oil_pressure_min_updated = false;
        }
    }
    else
    {

        g.setFont(SevenSegNumFont);
        //if (vehicle_speed_updated || force_update)
        //{
        //    g.printNumI(vehicle_speed, cols[2], rows[3], 3, '0');
        //    vehicle_speed_updated = false;
        //}
        switch(_idx_UI_menu_cockpit_update) {
            case 0:
            // KM driven
                if (elpased_km_since_start_updated || force_update) {
                    if (!destroy) {
                        g.setColor(font_color_alt_other); 
                        if (elpased_km_since_start >= 999)
                            g.printNumI(999, LEFT, rows[3], 3, '0');
                        else 
                            g.printNumI(elpased_km_since_start, LEFT, rows[3], 3, '0');
                        g.setColor(font_color);
                    } else {
                        g.printNumI(100, LEFT, rows[3], 3, '0');
                    }
                    elpased_km_since_start_updated = false;
                    
                }
                    _idx_UI_menu_cockpit_update++;
                break;
            case 1:
            // Fuel liters     
                if (fuel_level_updated || force_update)
                {
                    if (!destroy)
                        g.setColor(font_color_alt_other);    
                    g.printNumI(fuel_level, RIGHT, rows[3], 2, '0');
                    if (!destroy)
                        g.setColor(font_color);
                    fuel_level_updated = false;
                }
                    _idx_UI_menu_cockpit_update++;
                break;
            case 2:
            // KM remaining
            
                if (fuel_km_left_updated || force_update)
                {
                    if (!destroy)
                        g.setColor(font_color_alt_other);   
                    if (fuel_km_left > 999)
                        g.printNumI(999, CENTER, rows[17], 3, '0');
                    else 
                        g.printNumI(fuel_km_left, CENTER, rows[17], 3, '0');
                    if (!destroy)
                        g.setColor(font_color);
                    fuel_km_left_updated = false;
                }
                _idx_UI_menu_cockpit_update++;
                break;
            case 3:
            // BigFont
                g.setFont(BigFont);

                if (fuel_per_100km_updated || force_update) {
                    if (!destroy)
                        g.setColor(font_color_alt_other);    
                    g.printNumI(fuel_per_100km, RIGHT, rows[10], 2, '0');
                    if (!destroy)
                        g.setColor(font_color);
                    fuel_per_100km_updated = false;
                }

                if (coolant_temp_updated || force_update) {
                    if (!destroy) {
                        if (coolant_temp < 90)
                            g.setColor(TFT_CYAN);
                        else if (coolant_temp >= 90 && coolant_temp < 95)
                            g.setColor(font_color_alt_other);
                        else 
                            g.setColor(TFT_RED);   
                    }
                    if (destroy) {
                        g.printNumI(coolant_temp, LEFT, rows[10], 3, '0');
                    } else {
                        if (coolant_temp > 99) 
                            g.printNumI(coolant_temp, LEFT, rows[10], 3, '0');
                        else 
                            g.printNumI(coolant_temp, LEFT, rows[10], 2, '0');
                    }

                    if (!destroy)
                        g.setColor(font_color);
                    coolant_temp_updated = false;

                }

                _idx_UI_menu_cockpit_update = 0;
                is_finished_cycle = true;
                break;
            default:
                _idx_UI_menu_cockpit_update = 0;
                is_finished_cycle = true;
                break;

        }
        //if (coolant_temp_updated || force_update)
        //{
        //    if (!destroy)
        //        g.setColor(font_color_alt);
        //    //g.printNumI(coolant_temp, cols[22], rows[3], 3, '0');
        //    if (!destroy)
        //        g.setColor(font_color);
        //    //coolant_temp_updated = false;
        //}
        //if (oil_temp_updated || force_update)
        //{
        //    if (!destroy)
        //        g.setColor(font_color_dark);
        //    g.printNumI(oil_temp, cols[22], rows[9], 3, '0');
        //    if (!destroy)
        //        g.setColor(font_color);
        //    oil_temp_updated = false;
        //}
        
        
        g.setFont(BigFont);
    }

        g.setFont(BigFont);
    g.setColor(font_color);
    return is_finished_cycle;
}
uint8_t _amount_remove_menu_cockpit_called = 0;
uint8_t _cockpit_remove_initiated = false;
bool remove_menu_cockpit() {
    if (!_cockpit_remove_initiated ) {
        // Reset
        _amount_remove_menu_cockpit_called = 0;
        _cockpit_remove_initiated  = true;
        _idx_UI_menu_cockpit_update = 0; // Reset scheduler
    }

    if (_amount_remove_menu_cockpit_called == 0) {
        init_menu_cockpit(back_color);
    }

    bool is_scheduler_done = display_menu_cockpit(true, true);
    if (is_scheduler_done && _cockpit_remove_initiated) {
        _cockpit_remove_initiated = false;
        _idx_UI_menu_cockpit_update = 0;
        _amount_remove_menu_cockpit_called = 0;
    } else {
        _amount_remove_menu_cockpit_called++;
    }

    return is_scheduler_done;
}

// Second cockpit
// First page: Car specific data
// Second page: Connection specific data
uint8_t experimental_menu_page_switch = false;
void init_menu_experimental(bool destroy = false)
{
    if (destroy)
        g.setColor(back_color);
    // TODO menu too slow!
    g.setFont(SmallFont);
    if (!experimental_menu_page_switch) { // Page 0
        g.print("ODO", LEFT, rows[3]);
        g.print("km", RIGHT, rows[3]);
        g.print("FUEL R", LEFT, rows[4]);
        g.print("ohm", RIGHT, rows[4]);
        g.print("OILPRESS", LEFT, rows[5]);
        g.print("bool", RIGHT, rows[5]);
        g.print("TIME ECU", LEFT, rows[6]);
        g.print("ms", RIGHT, rows[6]);
        // Computed values:
        g.print("TIME CON", LEFT, rows[7]);
        g.print("secs", RIGHT, rows[7]);
        g.print("Traveled", LEFT, rows[8]);
        g.print("km", RIGHT, rows[8]);
        g.print("Fuel used", LEFT, rows[9]);
        g.print("L", RIGHT, rows[9]);
        g.print("Fuel/h", LEFT, rows[10]);
        g.print("L/h", RIGHT, rows[10]);
    } else {                              // Page 1
        g.print("ENG bits", LEFT, rows[3]);
        g.print("8 bit", RIGHT, rows[3]);
        g.print("TB  angle", LEFT, rows[4]);
        g.print("deg", RIGHT, rows[4]);
        g.print("STR angle", LEFT, rows[5]);
        g.print("deg", RIGHT, rows[5]);
        g.print("Voltage", LEFT, rows[6]);
        g.print("v", RIGHT, rows[6]);
        g.print("ENG load", LEFT, rows[7]);
        g.print("<TBA>", RIGHT, rows[7]);
        g.print("AMB TEMP", LEFT, rows[8]);
        g.print("C", RIGHT, rows[8]);
        g.print("Pressure", LEFT, rows[9]);
        g.print("mbar", RIGHT, rows[9]);
        g.print("Lambda 1", LEFT, rows[10]);
        g.print("<TBA>", RIGHT, rows[10]);
        g.print("Lambda 2", LEFT, rows[11]);
        g.print("<TBA>", RIGHT, rows[11]);

    }

    g.setFont(BigFont);
    g.setColor(font_color);
}
void display_menu_experimental(bool force_update = false, bool destroy = false)
{
    if (destroy) {
        g.setColor(back_color);
        force_update = true;
    }
    g.setFont(SmallFont);

    if (!experimental_menu_page_switch) { // Page 0
        if (odometer_updated || force_update)
        {
            g.printNumI(odometer, CENTER, rows[3], 6, '0');
            odometer_updated = false;
        }
        if (fuel_sensor_resistance_updated || force_update)
        {
            g.printNumI(fuel_sensor_resistance, CENTER, rows[4], 4, '0');
            fuel_sensor_resistance_updated = false;
        }
        if (oil_pressure_min_updated || force_update)
        {
            g.printNumI(oil_pressure_min, CENTER, rows[5], 2, '0');
            oil_pressure_min_updated = false;
        }
        if (time_ecu_updated || force_update)
        {
            g.printNumI(time_ecu, CENTER, rows[6], 7, '0');
            time_ecu_updated = false;
        }
        if (elapsed_seconds_since_start_updated || force_update)
        {
            g.printNumI(elapsed_seconds_since_start, CENTER, rows[7], 6, '0');
            elapsed_seconds_since_start_updated = false;
        }
        if (elpased_km_since_start_updated || force_update)
        {
            g.printNumI(elpased_km_since_start, CENTER, rows[8], 4, '0');
            elpased_km_since_start_updated = false;
        }
        if (fuel_burned_since_start_updated || force_update)
        {
            g.printNumI(fuel_burned_since_start, CENTER, rows[9], 2, '0');
            fuel_burned_since_start_updated = false;
        }
        if (fuel_per_hour_updated || force_update)
        {

            g.printNumF(fuel_per_hour, 2, CENTER, rows[10], '.', 6, '0');
            if (destroy) {
                g.print("         ", CENTER, rows[10]);
            }
            
            fuel_per_hour_updated = false;
        }
    
    }
    else
    { // Page 1
        if (error_bits_updated || force_update) {
            if (destroy)
                g.printNumI(1000000000000000, cols[11], rows[3], 16, '0');
            else {
            g.setColor(font_color_warn);
            g.printNumI(exhaust_gas_recirculation_error, cols[11], rows[3], 1, '0');
            g.setColor(font_color_alt);
            g.printNumI(oxygen_sensor_heating_error, cols[12], rows[3], 1, '0');
            g.setColor(font_color_warn);
            g.printNumI(oxgen_sensor_error, cols[13], rows[3], 1, '0');
            g.setColor(font_color_alt);
            g.printNumI(air_conditioning_error, cols[14], rows[3], 1, '0');
            g.setColor(font_color_warn);
            g.printNumI(secondary_air_injection_error, cols[15], rows[3], 1, '0');
            g.setColor(font_color_alt);
            g.printNumI(evaporative_emissions_error, cols[16], rows[3], 1, '0');
            g.setColor(font_color_warn);
            g.printNumI(catalyst_heating_error, cols[17], rows[3], 1, '0');
            g.setColor(font_color_alt);
            g.printNumI(catalytic_converter, cols[18], rows[3], 1, '0');
            g.setColor(font_color);
            error_bits_updated = false;
            }
        }
        if (ambient_temp_updated || force_update)
        {
            g.printNumI(ambient_temp, CENTER, rows[8], 2, '0');
            ambient_temp_updated = false;
        }
        if (pressure_updated || force_update) {
            g.printNumI(pressure, CENTER, rows[9], 5, '0');
            pressure_updated = false;
        }
        if (tb_angle_updated || force_update) {
            g.printNumF(tb_angle, 1, CENTER, rows[4], '.', 4, '0');
            tb_angle_updated = false;
        }
        if (steering_angle_updated || force_update) {
            g.printNumF(steering_angle, 1, CENTER, rows[5], '.', 4, '0');
            steering_angle_updated = false;
        }
        if (voltage_updated || force_update) {
            g.printNumI(voltage, CENTER, rows[6], 2, '0');
            voltage_updated = false;
        }
        if (engine_load_updated || force_update) {
            g.printNumI(engine_load, CENTER, rows[7], 4, '0');
            engine_load_updated = false;
        }
        if (lambda_updated || force_update) {
            g.printNumI(lambda, CENTER, rows[10], 3, '0');
            lambda_updated = false;
        }
        if (lambda_2_updated || force_update) {
            g.printNumI(lambda_2, CENTER, rows[11], 3, '0');
            lambda_2_updated = false;
        }
    }


    g.setColor(font_color);
    g.setFont(BigFont);

}
void remove_menu_experimental() {
    init_menu_experimental(true);
    display_menu_experimental(true, true);
}

// Third cockpit // Group readings
uint8_t graphic_menu_page_switch = false;
void draw_circle_fix(int x, int y, int radius) {
    for (int y1 = -radius; y1 <= 0; y1++) {
        int16_t x_counter = 0;
		for (int x1 = -radius; x1 <= 0; x1++) {
			if (x1 * x1 + y1 * y1 <= radius * radius && (x_counter == 0 || x1+1 > 0))
			{
				g.drawPixel(x + x1, y + y1);
				g.drawPixel(x + x1, y - y1);
				break;
			}
            x_counter++;
        }
    }
    
}
void init_menu_cockpit_graphic(bool destroy = false) {
    if (destroy)
        g.setColor(back_color);

    //draw_circle_fix(cols[3], rows[5], 30);

    g.print("- Group Reading - ", CENTER, rows[19]);
    g.setColor(font_color);
}
void display_menu_cockpit_graphic(bool destroy = false) {
    if (destroy)
        g.setColor(back_color);

    g.setColor(font_color);
}
void remove_menu_cockpit_graphic() {
    init_menu_cockpit_graphic(true);
    display_menu_cockpit_graphic(true);
}


// DTC menu
uint8_t dtc_menu_current_row = 16;
uint8_t dtc_menu_last_row = dtc_menu_current_row;
const uint8_t dtc_menu_row_max = 18;
void init_menu_dtc(bool destroy = false)
{
    if (destroy)
        g.setColor(back_color);
    g.setFont(SmallFont);
    g.print("Reset DTC buff", CENTER, rows[16]);
    g.print("!Read DTC ECU!", CENTER, rows[17]);
    g.print("!Clear DTC ECU!", CENTER, rows[18]); // Ask the user if he is sure
    g.setColor(font_color);
    g.setFont(BigFont);
}
void display_menu_dtc(bool destroy = false)
{
    if (destroy)
        g.setColor(back_color);

    g.setFont(SmallFont);
    if (dtc_menu_last_row != dtc_menu_current_row)
        g.print("  ", cols[2], rows[dtc_menu_last_row]);
    g.print("->", cols[2], rows[dtc_menu_current_row]);

    if (!read_dtc_errors_done)
    {
        g.print("Read DTC from ECU...", CENTER, rows[10]);
    }
    else
    {
        if (no_dtc_errors_found)
            {
                g.print("No DTC errors in ECU", CENTER, rows[10]);
            }
        else {
            for (int i = 0; i < DTC_ERRORS_SIZE; i++) {
                if (dtc_errors[i] != 0xFFFF && dtc_status_bytes[i] != 0xFF) {
                    // Legit error, print
                    if (i>13)
                        continue;
                    uint8_t current_row_temp = rows[3+i];
                    g.printNumI(i, LEFT, current_row_temp, 2, '0');
                    g.print(String(dtc_errors[i], HEX), CENTER, current_row_temp);
                    g.print(String(dtc_status_bytes[i], HEX), RIGHT, current_row_temp);
                }
            }
        }
    }

    g.setColor(font_color);
    g.setFont(BigFont);
}
void remove_menu_dtc() {
    init_menu_dtc(true);
    display_menu_dtc(true);
}


String kwp_mode_string(uint8_t kwpmode) {
    char result[3] = {'N', ' ', 'A'};
    switch(kwpmode) {
        case KWP_MODE_ACK:
            result[0] = 'A';
            result[1] = 'C';
            result[2] = 'K';
        break;
        case KWP_MODE_READGROUP:
            result[0] = 'G';
            result[1] = 'R';
            result[2] = 'P';
        break;
        case KWP_MODE_READSENSORS:
            result[0] = 'S';
            result[1] = 'E';
            result[2] = 'N';
        break;
        default:
        break;
    }
    return String(result);
}
// Settings menu
void init_menu_settings(bool destroy = false)
{
    if (destroy)
        g.setColor(back_color);
    g.setFont(SmallFont);

    g.print("Settings", LEFT, rows[4]);

    g.print("Debug time:", LEFT, rows[6]);
    g.print(String(setting_debugbenchmark), RIGHT, rows[6]);
    g.print("KWP Mode:", LEFT, rows[7]);
    g.print(kwp_mode_string(kwp_mode), RIGHT, rows[7]);
    g.print("Contrast*:", LEFT, rows[8]);
    g.printNumI(setting_contrast, RIGHT, rows[8], 3, '0');
    g.print("-> Clear DEBUG <-", CENTER, rows[10]);
    g.print("-> Disconnect <-", CENTER, rows[12]);

    if (menu_selected_setting < 3)
        g.print("-->", CENTER, rows[menu_selected_setting + 6]);
    else if (menu_selected_setting >= 3 && menu_selected_setting <= 4)
        g.print(">", LEFT, rows[10]+rows[(menu_selected_setting-3)*2]);

    g.setColor(font_color);

}
void display_menu_settings()
{
    //if (setting_night_mode_last != setting_night_mode)
    //{
    //    setting_night_mode_last = setting_night_mode;
    //    g.print(String(setting_night_mode_last), RIGHT, rows[6]);
    //}
    g.setFont(SmallFont);
    if (kwp_mode_last != kwp_mode)
    {
        kwp_mode_last = kwp_mode;
        g.print(kwp_mode_string(kwp_mode), RIGHT, rows[7]);
    }
    if (setting_contrast_last != setting_contrast)
    {
        setting_contrast_last = setting_contrast;
        g.printNumI(setting_contrast_last, RIGHT, rows[8], 3, '0');
    }

    if (menu_selected_setting != menu_selected_setting_last)
    {
        byte menu_selected_setting_temp = menu_selected_setting;
        if (menu_selected_setting_last < 3)
            g.print("   ", CENTER, rows[menu_selected_setting_last + 6]);
        else if (menu_selected_setting_last >= 3 && menu_selected_setting_last <= 4)
            g.print(" ", LEFT, rows[10]+rows[(menu_selected_setting_last-3)*2]);
        if (menu_selected_setting_temp < 3)
            g.print("-->", CENTER, rows[menu_selected_setting_temp + 6]);
        else if (menu_selected_setting_temp >= 3 && menu_selected_setting_temp <= 4)
            g.print(">", LEFT, rows[10]+rows[(menu_selected_setting_temp-3)*2]);
        
        menu_selected_setting_last = menu_selected_setting_temp;
    }
    if (setting_debugbenchmark != setting_debugbenchmark_last) {
        setting_debugbenchmark_last = setting_debugbenchmark;
        g.print(String(setting_debugbenchmark), RIGHT, rows[6]);
    }
    g.setFont(BigFont);
}
void remove_menu_settings() {
    init_menu_settings(true);
}

// CONNECT SETUP GUI
void init_setup_config(uint8_t ecu_addr, uint16_t baud, bool debug, uint8_t userinput_currentrow, bool simulation_mode_active, bool userinput_night_mode) {
    //g.print("Configuration:", LEFT, rows[8]);
    g.print("-->", LEFT, rows[userinput_currentrow]);
    g.print("Mode:", CENTER, rows[10]);
    if (simulation_mode_active) 
        g.print("SIM", RIGHT, rows[10]);
    else 
        g.print("ECU", RIGHT, rows[10]); 
    g.print("Baud:", CENTER, rows[11]);
    g.printNumI(baud, RIGHT, rows[11], 5, '0');
    g.print("Debug:", CENTER, rows[12]);
    g.printNumI(debug, RIGHT, rows[12]); // TODO Arrows --> pointing and user input button
    g.print("ECU Addr:", CENTER, rows[13]);
    g.print("0x", cols[26], rows[13]);
    char temp_ecu_addr[2];
    sprintf(temp_ecu_addr, "%X", ecu_addr);
    
    if (ecu_addr < 10) {
        g.printNumI(ecu_addr, cols[28], rows[13], 2, '-');
    } else {
        g.print(temp_ecu_addr, cols[28], rows[13]);
    }
    g.print(temp_ecu_addr, cols[28], rows[13]);
    g.setColor(font_color_dark);
    g.print("Night mode:", CENTER, rows[15]);
    g.print(String(userinput_night_mode), RIGHT, rows[15]);
    g.setColor(font_color_warn);
    g.print("Debug screen", CENTER, rows[17]);
    g.setColor(font_color);
    g.print("Connect", CENTER, rows[18]);
}
void remove_setup_config(uint8_t arrow_row) {
    g.print("              ", CENTER, rows[5]);
    g.print("              ", CENTER, rows[6]);
    //g.print("              ", LEFT, rows[8]);
    g.print("   ", LEFT, rows[arrow_row]);
    g.print("     ", CENTER, rows[10]);
    g.print("   ", RIGHT, rows[10]);
    g.print("     ", CENTER, rows[11]);
    g.print("     ", RIGHT, rows[11]);
    g.print("      ", CENTER, rows[12]);
    g.print(" ", RIGHT, rows[12]); 
    g.print("         ", CENTER, rows[13]);
    g.print("  ", cols[26], rows[13]);
    g.print("  ", cols[28], rows[13]);
    g.print("            ", CENTER, rows[15]);
    g.print("  ", RIGHT, rows[15]);
    g.print("            ", CENTER, rows[17]);
    g.print("           ", CENTER, rows[18]);
}
void update_setup_config(uint8_t userinput_current_row, uint8_t userinput_previous_row) {
    // Draw arrow
        if (userinput_current_row != userinput_previous_row)
        {
            g.print("   ", LEFT, rows[userinput_previous_row]);
            g.print("-->", LEFT, rows[userinput_current_row]);
        }
        // Display updated setting values
        if (userinput_simulation_mode != userinput_simulation_mode_previous)
        {
            if (userinput_simulation_mode)
                g.print("SIM", RIGHT, rows[10]);
            else
                g.print("ECU", RIGHT, rows[10]);
            userinput_simulation_mode_previous = userinput_simulation_mode;
        }
        if (userinput_baudrate != userinput_baudrate_previous)
        {
            g.printNumI(userinput_baudrate, RIGHT, rows[11], 5, '0');
            userinput_baudrate_previous = userinput_baudrate;
        }
        if (userinput_debug_mode != userinput_debug_mode_previous)
        {
            g.print(String(userinput_debug_mode), RIGHT, rows[12]);
            userinput_debug_mode_previous = userinput_debug_mode;
        }
        if (get_userinput_ecu_address() != userinput_ecu_address_previous)
        {
            
            char temp_ecu_addr[2];
            sprintf(temp_ecu_addr, "%X", get_userinput_ecu_address());
            if (get_userinput_ecu_address() < 10) {
                g.printNumI(get_userinput_ecu_address(), cols[28], rows[13], 2, '-');
            } else {
                g.print(temp_ecu_addr, cols[28], rows[13]);
            }
            userinput_ecu_address_previous = get_userinput_ecu_address();
        }
        if (userinput_night_mode != userinput_night_mode_previous) {
            g.print(String(userinput_night_mode), RIGHT, rows[15]);
            userinput_night_mode_previous = userinput_night_mode;
        }
}

// DEBUG GUI
const uint8_t debug_page_row_start = 3;
const uint8_t debug_page_row_end = 19;
const uint8_t debug_page_row_distance = debug_page_row_end - debug_page_row_start;
uint8_t debug_page_current = 0;
#define SCHEDULER_DEBUG_SHOULDBREAK 0
#define SCHEDULER_DEBUG_SHOULDCONTINUE 1
#define SCHEDULER_DEBUG_SHOULDPRINT 2
const uint8_t SCHEDULER_DEBUG_MAX = 4;
uint8_t scheduler_debug = 0;
uint8_t scheduler_debug_instruction(uint8_t loop_counter)
{
    switch (scheduler_debug)
    {
    case 0:
        if (loop_counter >= 3) {
            return SCHEDULER_DEBUG_SHOULDBREAK;
        }
        else
            return SCHEDULER_DEBUG_SHOULDPRINT;
        break;
    case 1:
        if (loop_counter < 3) 
            return SCHEDULER_DEBUG_SHOULDCONTINUE;
        else if (loop_counter >= 6) {
            return SCHEDULER_DEBUG_SHOULDBREAK;
        }
        else
            return SCHEDULER_DEBUG_SHOULDPRINT;
        break;
    case 2:
        if (loop_counter < 6)
            return SCHEDULER_DEBUG_SHOULDCONTINUE;
        else if (loop_counter >= 9) {
            return SCHEDULER_DEBUG_SHOULDBREAK;
        }
        else
            return SCHEDULER_DEBUG_SHOULDPRINT;
        break;
    case 3:
        if (loop_counter < 9)
            return SCHEDULER_DEBUG_SHOULDCONTINUE;
        else if (loop_counter >= 12)  {
            return SCHEDULER_DEBUG_SHOULDBREAK;
        }
        else
            return SCHEDULER_DEBUG_SHOULDPRINT;
        break;
    case 4:
        if (loop_counter < 12)
            return SCHEDULER_DEBUG_SHOULDCONTINUE;
        else if (loop_counter > 15)  {
            return SCHEDULER_DEBUG_SHOULDBREAK;
        }
        else
            return SCHEDULER_DEBUG_SHOULDPRINT;
        break;
    }
}
bool init_debug(bool destroy = false, bool scheduler_deactivate = false) { // Return true if scheduler is done
    if (destroy)
        g.setColor(back_color);


    g.print("Page: ", CENTER, rows[19]);
    g.printNumI(debug_page_current+1, cols[19], rows[19], 1);
    g.print("/", cols[21], rows[19]);
    g.printNumI(debug_messages_as_code_length / debug_page_row_distance, cols[23], rows[19], 1);

    uint8_t loop_counter = 0;
    for (int i = debug_page_row_start; i < debug_page_row_end; i++) {
        int temp_row = (i+(debug_page_current * debug_page_row_distance))-debug_page_row_start;
        if (temp_row >= debug_messages_as_code_length)
            break;

        uint8_t scheduler_instruction = scheduler_debug_instruction(loop_counter);
        loop_counter++;

        if (scheduler_deactivate)
            scheduler_instruction = SCHEDULER_DEBUG_SHOULDPRINT;

        if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDBREAK) {
            break;
        } else if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDCONTINUE) {
            continue;
        } else if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDPRINT) {
            g.printNumI(temp_row, cols[0], rows[i], 2, '0');
        } else {
            // ERROR
        }
    }

    uint8_t temp_start = debug_page_current * debug_page_row_distance;
    int i_temp = debug_page_row_start;
    loop_counter = 0;
    for (int i = temp_start; i < temp_start + debug_page_row_distance; i++) {
        
        if (i >= debug_messages_as_code_length) {
            break;
        }
        // TODO: string magic
        if (destroy)
            g.setColor(back_color);
        else 
            g.setColor(debug_decode_string_color(debug_messages_as_code[i]));
        //debugln("Color should be " + String(color, HEX) + " but is " + String(color_is, HEX));

        uint8_t scheduler_instruction = scheduler_debug_instruction(loop_counter);
        loop_counter++;

        if (scheduler_deactivate)
            scheduler_instruction = SCHEDULER_DEBUG_SHOULDPRINT;

        if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDBREAK)
        {
            i_temp++;
            g.setColor(font_color);
            break;
        }
        else if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDCONTINUE)
        {
            i_temp++;
            g.setColor(font_color);
            continue;
        }
        else if (scheduler_instruction == SCHEDULER_DEBUG_SHOULDPRINT)
        {
            g.print(debug_get_string(i), cols[3], rows[i_temp]);
            i_temp++;
        }
        else
        {
            // ERROR
        }

        g.setColor(font_color);
    }

    // Update scheduler
    if (scheduler_deactivate) {
        scheduler_debug = 0;
        g.setColor(font_color);
        return true;
    }
    if (scheduler_debug < SCHEDULER_DEBUG_MAX) {
        scheduler_debug++;
    } else {
        scheduler_debug = 0;
        g.setColor(font_color);
        return true;
    }
        g.setColor(font_color);
    return false;
}
bool remove_debug(bool scheduler_deactivate = false) {
    return init_debug(true, scheduler_deactivate);
    //g.setColor(font_color);
    //g.print("                  ", CENTER, rows[3]);
    //for (int i = debug_page_row_start; i < debug_page_row_end; i++) {
    //    g.print("                              ", cols[0], rows[i]);
    //}
}
// --- DEBUG GUI

// Benchmark GUI bottom screen
uint16_t max_frame_time = 0;
uint32_t max_frame_time_timepoint_set = millis();
void init_benchmark(bool destroy = false) {
    if (destroy)
        g.setColor(back_color);
    else
        g.setColor(font_color_warn);

    g.print("Menu swtch:", LEFT, rows[17]);
    if (timer_useable(TIMER_CODE_START_MENU_SWITCH))
        g.printNumI(timer_duration[TIMER_CODE_START_MENU_SWITCH], CENTER, rows[17], 4, '0');

    g.print("Drawscreen:", LEFT, rows[18]);
    if (timer_useable(TIMER_CODE_START_DRAWSCREEN))
        g.printNumI(timer_duration[TIMER_CODE_START_DRAWSCREEN], CENTER, rows[18], 4, '0');
    g.printNumI(timer_max_drawscreen, RIGHT, rows[18], 4, '0');

    g.print("OBD mode:", LEFT, rows[19]);
    if (timer_useable(TIMER_CODE_OBD_MODE))
        g.printNumI(timer_duration[TIMER_CODE_OBD_MODE], CENTER, rows[19], 4, '0');
    if (destroy)
        g.print("    ", CENTER, rows[19]);

    // This must be last in order to capture whole frametime
    timer_done(TIMER_CODE_START_FRAME);
    if (timer_useable(TIMER_CODE_START_FRAME)) // Should be always true
        g.printNumI(timer_duration[TIMER_CODE_START_FRAME], RIGHT, rows[19], 4, '0');

    g.setColor(font_color);
}
void remove_benchmark() {
    init_benchmark(true);
}
void init_benchmark_minimalistic(bool destroy = false)
{
    if (!destroy)
        g.setColor(font_color_warn);
    else
        g.setColor(back_color);


    g.printNumI(timer_duration[TIMER_CODE_START_DRAWSCREEN], RIGHT, rows[17], 3, '-');
    

    if (timer_useable(TIMER_CODE_START_MENU_SWITCH)) {
        g.printNumI(timer_duration[TIMER_CODE_START_MENU_SWITCH], LEFT, rows[19], 4, '-');
    }


    // This must be last in order to capture whole frametime
    timer_done(TIMER_CODE_START_FRAME);
    if (timer_useable(TIMER_CODE_START_FRAME)) // Should be always true
        if (timer_duration[TIMER_CODE_START_FRAME] < 10000) {
            g.printNumI(timer_duration[TIMER_CODE_START_FRAME], RIGHT, rows[19], 4, '-');
            if (abs(millis() - max_frame_time_timepoint_set) > 1500) {
                max_frame_time = 0;
            }
            if (timer_duration[TIMER_CODE_START_FRAME] > max_frame_time) {
                max_frame_time_timepoint_set = millis();
                max_frame_time = timer_duration[TIMER_CODE_START_FRAME];
            }
            g.printNumI(max_frame_time, RIGHT, rows[18], 4, '-');
        }

    g.setColor(font_color);
}
void remove_benchmark_minimalistic()
{
    init_benchmark_minimalistic(true);
}
// Benchmark end


void init_connect(uint16_t baudrate, uint8_t address, bool destroy = false) {
    g.setColor(font_color_alt);
    if (destroy)
        g.setColor(back_color);
    g.print("Init connection", LEFT, rows[3]);
    g.printNumI(baudrate, cols[17], rows[3], 5, '0');
    g.print(" 0x", cols[23], rows[3]);
    g.printNumI(address, cols[26], rows[3], 2, '0');
    draw_status_bar();
    g.print("5 baud init", LEFT, rows[4]);
    g.print("Serial init", LEFT, rows[5]);
    g.print("Handshake init", LEFT, rows[6]);
}
void destroy_connect() {
    init_connect(0, 0, true);
    g.print("Connected!", 325, rows[15]);
    g.setColor(font_color);
}