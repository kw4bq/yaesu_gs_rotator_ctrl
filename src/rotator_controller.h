#include <Arduino.h>

#define CODE_VERSION "2019.04.30.01"

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/wdt.h>

#define SERIAL_PORT_CLASS HardwareSerial

#define FEATURE_I2C_LCD
#define FEATURE_LCD_DISPLAY
#define FEATURE_WIRE_SUPPORT

#include "rotator_features.h"
#include "rotator_dependencies.h"

#include <LiquidCrystal_I2C.h>
#include "rotator_k3ngdisplay.h"
#include <Wire.h>

#if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
  #include <moon2.h>
#endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

#ifdef FEATURE_SUN_TRACKING
  #include <sunpos.h>
#endif // FEATURE_SUN_TRACKING

#ifdef FEATURE_GPS
  #include <TinyGPS.h>
#endif // FEATURE_GPS

#ifdef FEATURE_RTC_DS1307
  #include <RTClib.h>
#endif // FEATURE_RTC_DS1307

#ifdef FEATURE_RTC_PCF8583
  #include <PCF8583.h>
#endif //FEATURE_RTC_PCF8583

#include "rotator.h"
#include "rotator_pins.h"
#include "rotator_settings.h"

#include "rotator_language.h"
#include "rotator_debug.h"

SERIAL_PORT_CLASS * control_port;

byte incoming_serial_byte = 0;

byte reset_the_unit = 0;

int azimuth = 0;
int raw_azimuth = 0;
int target_azimuth = 0;
int target_raw_azimuth = 0;
int azimuth_starting_point = AZIMUTH_STARTING_POINT_DEFAULT;
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
#endif // FEATURE_TIMED_BUFFER

byte normal_az_speed_voltage = 0;
byte current_az_speed_voltage = 0;

#ifdef FEATURE_ELEVATION_CONTROL
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

  #ifdef FEATURE_TIMED_BUFFER
    int timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
  #endif // FEATURE_TIMED_BUFFER
  byte elevation_button_was_pushed = 0;
#endif // FEATURE_ELEVATION_CONTROL

byte push_lcd_update = 0;

#ifdef FEATURE_GPS
  TinyGPS gps;
  SERIAL_PORT_CLASS * gps_port;
  byte gps_data_available = 0;
  #ifdef GPS_MIRROR_PORT
    SERIAL_PORT_CLASS * (gps_mirror_port);
  #endif //GPS_MIRROR_PORT
#endif //defined(FEATURE_GPS)

double latitude = DEFAULT_LATITUDE;
double longitude = DEFAULT_LONGITUDE;

#ifdef FEATURE_MOON_TRACKING
  byte moon_tracking_active = 0;
  byte moon_visible = 0;
  double moon_azimuth = 0;
  double moon_elevation = 0;
#endif // FEATURE_MOON_TRACKING

#ifdef FEATURE_SUN_TRACKING
  float sun_azimuth = 0;
  float sun_elevation = 0;
  cTime c_time;
  cLocation c_loc;
  cSunCoordinates c_sposn;
  byte sun_visible = 0;
  byte sun_tracking_active = 0;
#endif // FEATURE_SUN_TRACKING

#ifdef FEATURE_CLOCK
  unsigned long clock_years = 0;
  unsigned long clock_months = 0;
  unsigned long clock_days = 0;
  unsigned long clock_hours = 0;
  unsigned long clock_minutes = 0;
  unsigned long clock_seconds = 0;
  long local_clock_years = 0;
  long local_clock_months = 0;
  long local_clock_days = 0;
  long local_clock_hours = 0;
  long local_clock_minutes = 0;
  long local_clock_seconds = 0;
  int clock_year_set = 2017;
  byte clock_month_set = 1;
  byte clock_day_set = 1;
  byte clock_sec_set = 0;
  unsigned long clock_hour_set = 0;
  unsigned long clock_min_set = 0;
  unsigned long millis_at_last_calibration = 0;
#endif // FEATURE_CLOCK

#if defined(FEATURE_GPS) || defined(FEATURE_RTC) || defined(FEATURE_CLOCK)
  byte clock_status = FREE_RUNNING;
#endif // defined(FEATURE_GPS) || defined(FEATURE_RTC)

#ifdef FEATURE_POWER_SWITCH
  unsigned long last_activity_time = 0;
#endif //FEATURE_POWER_SWITCH

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

K3NGdisplay k3ngdisplay(LCD_COLUMNS,LCD_ROWS,LCD_UPDATE_TIME);

#ifdef FEATURE_RTC_DS1307
  RTC_DS1307 rtc;
#endif //FEATURE_RTC_DS1307

#ifdef FEATURE_RTC_PCF8583
  PCF8583 rtc(0xA0);
#endif //FEATURE_RTC_PCF8583

void read_headings();
void service_blink_led();
void check_for_reset_flag();
void check_az_speed_pot();
void check_az_preset_potentiometer();
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
void stop_all_tracking();
void check_for_dirty_configuration();
void service_rotation_indicator_pin();
void check_limit_sense();
void az_position_incremental_encoder_interrupt_handler();
void el_position_incremental_encoder_interrupt_handler();
void pinModeEnhanced(uint8_t pin, uint8_t mode);
void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue);
void analogWriteEnhanced(uint8_t pin, int writevalue);
void port_flush();
void service_power_switch();
void service_analog_output_pins();
void submit_autocorrect(byte axis,float heading);
void update_sun_position();
void update_moon_position();
void update_time();
void service_gps();
void service_rtc();
void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string);
void service_moon_tracking();
void service_sun_tracking();
void check_moon_pushbutton_calibration();
void check_sun_pushbutton_calibration();
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
