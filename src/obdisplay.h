#include "utils.h"

// Display
bool DEBUG_BENCHMARK = false;        // 1 = display time information
bool NIGHT_MODE_ACTIVE = DEFAULT_UI_NIGHT_MODE;

// Startup configuration
bool userinput_simulation_mode = DEFAULT_UI_SIMULATION_MODE;
bool userinput_simulation_mode_previous = userinput_simulation_mode;
long userinput_baudrate = DEFAULT_UI_BAUDRATE;
uint16_t userinput_baudrate_previous = userinput_baudrate;
uint8_t supported_baud_rates_counter = supported_baud_rates_length;
bool userinput_debug_mode = DEFAULT_UI_DEBUG_MODE;
bool userinput_debug_mode_previous = userinput_debug_mode;
uint8_t userinput_ecu_address = DEFAULT_UI_ECU_ADDRESS; // 1 or 17
uint8_t userinput_ecu_address_previous = userinput_ecu_address;
const uint8_t userinput_ecu_address_all_size = 4;
uint8_t userinput_ecu_address_all[userinput_ecu_address_all_size] = {0x1, 0x8, 0x17, 0x45};
uint8_t userinput_ecu_address_idx = 2;
void increment_ecu_address() {
    userinput_ecu_address_idx++;
    if (userinput_ecu_address_idx >= userinput_ecu_address_all_size)
        userinput_ecu_address_idx = 0;
}
uint8_t get_userinput_ecu_address() {
    return userinput_ecu_address_all[userinput_ecu_address_idx];
}
bool userinput_night_mode = DEFAULT_UI_NIGHT_MODE;
bool userinput_night_mode_previous = userinput_night_mode;


#define _VERSION  "0.1"


// Backend
uint8_t reconnect_counter = 0;

bool simulation_mode_active = false; // If simulation mode is active the device will display imaginary values
bool debug_mode_enabled = false;
long baud_rate = 0; // 1200, 2400, 4800, 9600, 10400
uint8_t addr_selected = 0x00;               // Selected ECU address to connect to, see ECU Addresses constants
// Backend values
int connection_attempts_counter = 0;
uint8_t kwp_mode = KWP_MODE_READSENSORS;
uint8_t kwp_mode_last = kwp_mode;
uint8_t kwp_group = 1; // Dont go to group 0 its not good.
bool serial_active_last = false;
bool connected = false;
bool connected_last = connected; // Connection with ECU active
int available_last = 0;
uint8_t block_counter = 0;
uint8_t block_counter_last = block_counter; // Could be byte maybe?
bool com_error = false;
bool com_error_last = com_error; // Whether a communication warning occured // Block length warning. Expected 15 got " + String(data)
// Group measurements
/*Temporary Measurements for if you want to find out which values show up in your groups in a desired ECU address.
Just uncomment and add the logic in read_sensors(). This can also be done with VCDS or other tools.*/
byte k[4] = {0, 0, 0, 0};
float v[4] = {-1, -1, -1, -1};
uint8_t k_temp[4] = {0, 0, 0, 0};
bool k_temp_updated = false;
float v_temp[4] = {123.4, 123.4, 123.4, 123.4};
bool v_temp_updated = false;
String unit_temp[4] = {"N/A", "N/A", "N/A", "N/A"};
bool unit_temp_updated = false;

// Menu
byte menu_max = 5;
byte menu = DEFAULT_UI_MENU;
byte menu_last = menu;
bool menu_switch = false;

void increment_menu()
{
    menu_last = menu;
    if (menu >= menu_max)
    {
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
        menu_last = menu;
    if (menu == 0)
    {
        menu = menu_max;
    }
    else
    {
        menu--;
    }
    menu_switch = true;
}
// 5 Settings
bool menu_settings_switch = false;
byte menu_settings_max = 4;
byte menu_selected_setting = 0;
byte menu_selected_setting_last = 0;
byte setting_brightness = 16; // max 16
byte setting_brightness_last = setting_brightness;
char setting_contrast = 30; // max 64
byte setting_contrast_last = setting_contrast;
bool setting_debugbenchmark = DEBUG_BENCHMARK;
bool setting_debugbenchmark_last = setting_debugbenchmark;
void increment_menu_settings()
{
    if (menu_selected_setting >= menu_settings_max)
    {
        menu_selected_setting = 0;
    }
    else
    {
        menu_selected_setting++;
    }
    menu_settings_switch = true;
}
void decrement_menu_settings()
{
    if (menu_selected_setting == 0)
    {
        menu_selected_setting = menu_settings_max;
    }
    else
    {
        menu_selected_setting--;
    }
    menu_settings_switch = true;
}

bool increment_menu_settings_value()
{
    switch (menu_selected_setting)
    {
    case 0:
        // Switch debug benchmark
        setting_debugbenchmark_last = DEBUG_BENCHMARK;
        setting_debugbenchmark = !DEBUG_BENCHMARK;
        DEBUG_BENCHMARK = !DEBUG_BENCHMARK;
        break;
    case 1:
        // KWP_MODE
        kwp_mode_last = kwp_mode;
        kwp_mode++;
        if (kwp_mode > 2) 
            kwp_mode = 0;
        
        break;
    case 2:
        // Contrast
        if (setting_contrast < 64)
        {
            setting_contrast_last = setting_contrast;
            setting_contrast++;
            //g.setContrast(setting_contrast);
        }
        break;
    case 3:
        // Clear DEBUG array
        debug_clear();
        break;
    case 4:
        // Disconnect
        //disconnect();
        return false;
    }
    return true;

}
bool decrement_menu_settings_value()
{
    switch (menu_selected_setting)
    {
    case 0:
        // Switch debug benchmark
        setting_debugbenchmark_last = DEBUG_BENCHMARK;
        setting_debugbenchmark = !DEBUG_BENCHMARK;
        DEBUG_BENCHMARK = !DEBUG_BENCHMARK;
        break;
    case 1:
        // Brightness
        if (setting_brightness > 0)
        {
            setting_brightness_last = setting_brightness;
            setting_brightness--;
            //g.setBrightness(setting_brightness);
        }
        break;
    case 2:
        // Contrast
        if (setting_contrast > 0)
        {
            setting_contrast_last = setting_contrast;
            setting_contrast--;
            //g.setContrast(setting_contrast);
        }
        break;
    case 3:
        // Clear DEBUG array
        debug_clear();
        break;
    case 4:
        // Disconnect
        //disconnect();
        return false;
        break;
    }
    return true;
}


void reset_temp_group_array()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        k_temp[i] = 0;
        v_temp[i] = 123.4;
        unit_temp[i] = "N/A";
    }
}
// DTC error
#define DTC_ERRORS_SIZE 16
bool read_dtc_errors_done = false;
bool no_dtc_errors_found = false;
uint16_t dtc_errors[DTC_ERRORS_SIZE] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
bool dtc_errors_updated = false;
uint8_t dtc_status_bytes[DTC_ERRORS_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool dtc_status_bytes_updated = false;
uint8_t dtc_errors_count_empty() {
    uint8_t errors_counter = 0;
    for (int i = 0; i < DTC_ERRORS_SIZE; i++) {
        if (dtc_errors[i] != 0xFFFF && dtc_status_bytes[i] != 0xFF) {
            continue;
        }
        errors_counter++;
    }
    return errors_counter;
}
bool dtc_errors_empty_fast() {
    return dtc_errors[0 == 0xFFFF];
}
bool dtc_errors_empty() {
    return dtc_errors_count_empty == 0;
}
void reset_dtc_status_errors_array()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        dtc_errors[i] = (uint16_t)0xFFFF;
        dtc_status_bytes[i] = 0xFF;
    }
    read_dtc_errors_done = false;
    no_dtc_errors_found = false;
}
void reset_dtc_status_errors_array_random()
{
    reset_dtc_status_errors_array();
    for (uint8_t i = 0; i < 16; i++)
    {
        dtc_errors[i] = (uint16_t)(i * 1000);
        dtc_status_bytes[i] = i * 10;
    }
    read_dtc_errors_done = true;
}



// Utils
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

// ECU values
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
float odometer = 0;
bool odometer_updated = false;
float odometer_start = odometer;
uint8_t fuel_level = 0;
bool fuel_level_updated = false;
float fuel_level_start = fuel_level;
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
// Computed Stats
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
uint16_t fuel_km_left = 0;
bool fuel_km_left_updated = false;


// Compute values
void compute_values()
{
    elapsed_seconds_since_start = ((millis() - timer_start[TIMER_CODE_OBD_CONNECTIONSTART]) / 1000);
    elapsed_seconds_since_start_updated = true;
    elpased_km_since_start = abs(odometer - odometer_start);
    elpased_km_since_start_updated = true;
    fuel_burned_since_start = abs(fuel_level_start - fuel_level);
    fuel_burned_since_start_updated = true;
    if (elpased_km_since_start == 0)
        fuel_per_100km = 99;
    else
        fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
    fuel_per_100km_updated = true;
    fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;
    fuel_per_hour_updated = true;
    float km_per_liter_fuel = 0;
    if (fuel_burned_since_start == 0)
        km_per_liter_fuel = (float)elpased_km_since_start / (float) fuel_burned_since_start;
    fuel_km_left = 100*abs(fuel_level / fuel_per_100km);
    fuel_km_left_updated = true;
}

// Simulation mode
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
    //simulate_values_helper(fuel_level, 1, fuel_level_switch, fuel_level_updated, 57);           // Fuel
    if (fuel_level <= 0) {
        fuel_level = 52;
        fuel_level_start = fuel_level;
    }
    fuel_level-=0.004;
    odometer+=0.05;

    fuel_level_updated = true;
    odometer_updated = true;
}




