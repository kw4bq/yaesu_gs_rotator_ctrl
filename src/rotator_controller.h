#include <Arduino.h>

#define CODE_VERSION "2019.04.30.01"

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/wdt.h>

#define FEATURE_I2C_LCD
#define FEATURE_LCD_DISPLAY
#define FEATURE_WIRE_SUPPORT

#include "rotator_features.h"

#include <LiquidCrystal_I2C.h>
#include "rotator_k3ngdisplay.h"
#include <Wire.h>

#include "rotator.h"
#include "rotator_pins.h"
#include "rotator_settings.h"

#include "rotator_language.h"
#include "rotator_debug.h"

HardwareSerial * control_port;

byte incoming_serial_byte = 0;

byte reset_the_unit = 0;

int azimuth = 0;
int raw_azimuth = 0;
int target_azimuth = 0;
int target_raw_azimuth = 0;
int azimuth_starting_point = 0; //AZIMUTH_STARTING_POINT_DEFAULT;
int azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;

byte control_port_buffer[COMMAND_BUFFER_SIZE];
int control_port_buffer_index = 0;
byte az_state = IDLE;
byte debug_mode = DEFAULT_DEBUG_STATE;
int analog_az = 0;
unsigned long last_debug_output_time = 0;
unsigned long az_last_rotate_initiation = 0;
byte azimuth_button_was_pushed = 0;
byte brake_az_engaged = 0;
byte brake_el_engaged = 0;
byte configuration_dirty = 0;
unsigned long last_serial_receive_time = 0;

byte az_slowstart_active = AZ_SLOWSTART_DEFAULT;
byte az_slowdown_active = AZ_SLOWDOWN_DEFAULT;

byte az_request = 0;
int az_request_parm = 0;
byte az_request_queue_state = NONE;

unsigned long az_slowstart_start_time = 0;
byte az_slow_start_step = 0;
unsigned long az_last_step_time = 0;
byte az_slow_down_step = 0;
unsigned long az_timed_slow_down_start_time = 0;
byte backslash_command = 0;

struct config_t {
  byte magic_number;
  int analog_az_full_ccw;
  int analog_az_full_cw;
  int analog_el_0_degrees;
  int analog_el_max_elevation;
  float last_azimuth;
  float last_elevation;
  long last_az_incremental_encoder_position;
  int last_el_incremental_encoder_position;
  float azimuth_offset;
  float elevation_offset;
  byte az_stepper_motor_last_pin_state;
  byte el_stepper_motor_last_pin_state;
  byte az_stepper_motor_last_direction;
  byte el_stepper_motor_last_direction;
  int azimuth_starting_point;
  int azimuth_rotation_capability;
  byte brake_az_disabled;
  float clock_timezone_offset;
  byte autopark_active;
  unsigned int autopark_time_minutes;
  byte azimuth_display_mode;
} configuration;

#ifdef FEATURE_TIMED_BUFFER
int timed_buffer_azimuths[TIMED_INTERVAL_ARRAY_SIZE];
int timed_buffer_number_entries_loaded = 0;
int timed_buffer_entry_pointer = 0;
int timed_buffer_interval_value_seconds = 0;
unsigned long last_timed_buffer_action_time = 0;
byte timed_buffer_status = EMPTY;
int timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
#endif // FEATURE_TIMED_BUFFER

byte normal_az_speed_voltage = 0;
byte current_az_speed_voltage = 0;

int elevation = 0;
int target_elevation = 0;

byte el_request = 0;
int el_request_parm = 0;
byte el_request_queue_state = NONE;
byte el_slowstart_active = EL_SLOWSTART_DEFAULT;
byte el_slowdown_active = EL_SLOWDOWN_DEFAULT;
unsigned long el_slowstart_start_time = 0;
byte el_slow_start_step = 0;
unsigned long el_last_step_time = 0;
byte el_slow_down_step = 0;
unsigned long el_timed_slow_down_start_time = 0;
byte normal_el_speed_voltage = 0;
byte current_el_speed_voltage = 0;

int display_elevation = 0;
byte el_state = IDLE;
int analog_el = 0;

unsigned long el_last_rotate_initiation = 0;
byte elevation_button_was_pushed = 0;
byte push_lcd_update = 0;

double latitude = DEFAULT_LATITUDE;
double longitude = DEFAULT_LONGITUDE;

#ifdef FEATURE_AZIMUTH_CORRECTION
const float azimuth_calibration_from[]  = AZIMUTH_CALIBRATION_FROM_ARRAY;
const float azimuth_calibration_to[]    = AZIMUTH_CALIBRATION_TO_ARRAY;
#endif // FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
const float elevation_calibration_from[]  = ELEVATION_CALIBRATION_FROM_ARRAY;
const float elevation_calibration_to[]    = ELEVATION_CALIBRATION_TO_ARRAY;
#endif // FEATURE_ELEVATION_CORRECTION

#ifdef FEATURE_AUTOCORRECT
byte autocorrect_state_az = AUTOCORRECT_INACTIVE;
float autocorrect_az = 0;
unsigned long autocorrect_az_submit_time = 0;
#ifdef FEATURE_ELEVATION_CONTROL
byte autocorrect_state_el = AUTOCORRECT_INACTIVE;
float autocorrect_el = 0;
unsigned long autocorrect_el_submit_time = 0;
#endif //FEATURE_ELEVATION_CONTROL
#endif //FEATURE_AUTOCORRECT

DebugClass debug;

K3NGdisplay k3ngdisplay(LCD_COLUMNS, LCD_ROWS, LCD_UPDATE_TIME);

void read_headings();
void service_blink_led();
void check_for_reset_flag();
void check_brake_release();
void brake_release(byte az_or_el, byte operation);
void check_overlap();
void clear_command_buffer();
void check_serial();
void check_buttons();
void update_display();
void get_keystroke();
void print_wrote_to_memory();
void clear_serial_buffer();
void read_settings_from_eeprom();
void initialize_eeprom_with_defaults();
void write_settings_to_eeprom();
void az_check_operation_timeout();
void read_azimuth(byte force_read);
void output_debug();
void print_to_port(char * print_this,byte port);
void print_help(byte port);
void el_check_operation_timeout();
void read_elevation(byte force_read);
void update_el_variable_outputs(byte speed_voltage);
void update_az_variable_outputs(byte speed_voltage);
void rotator(byte rotation_action, byte rotation_type);
void initialize_interrupts();
void initialize_pins();
void initialize_serial();
void initialize_display();
void initialize_peripherals();
void submit_request(byte axis, byte request, int parm, byte called_by);
void service_rotation();
void service_request_queue();
void stop_all_tracking();
void check_for_dirty_configuration();
void service_rotation_indicator_pin();
void check_limit_sense();
void pinModeEnhanced(uint8_t pin, uint8_t mode);
void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue);
void analogWriteEnhanced(uint8_t pin, int writevalue);
void port_flush();
void service_power_switch();
void service_analog_output_pins();
void submit_autocorrect(byte axis,float heading);
void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string);
byte process_backslash_command(byte input_buffer[], int input_buffer_index, byte source_port, char * return_string);
void clear_timed_buffer();
void initiate_timed_buffer(byte source_port);
void print_timed_buffer_empty_message();
void check_timed_interval();
byte current_az_state();
byte current_el_state();
byte get_analog_pin(byte pin_number);
byte calibrate_az_el(float new_az, float new_el);
int digitalReadEnhanced(uint8_t pin);
int analogReadEnhanced(uint8_t pin);
float correct_azimuth(float azimuth_in);
float correct_elevation(float elevation_in);
char * azimuth_direction(int azimuth_in);
char * idle_status();
char *coordinates_to_maidenhead(float latitude_degrees,float longitude_degrees);
