

#define FEATURE_ELEVATION_CONTROL      // uncomment this for AZ/EL rotators
#define FEATURE_YAESU_EMULATION
#define CONTROL_PROTOCOL_EMULATION

// #define FEATURE_AUTOCORRECT

#define LANGUAGE_ENGLISH


#define FEATURE_AZ_POSITION_POTENTIOMETER
#define FEATURE_EL_POSITION_POTENTIOMETER

#define FEATURE_SUNFOUNDER_I2C_LCD

#define OPTION_GS_232B_EMULATION

#define FEATURE_TIMED_BUFFER           // Support for Yaesu timed buffer commands
#define OPTION_SERIAL_HELP_TEXT        // Yaesu help command prints help

// #define OPTION_C_COMMAND_SENDS_AZ_AND_EL  // uncomment this when using Yaesu emulation with Ham Radio Deluxe
// #define OPTION_DELAY_C_CMD_OUTPUT         // uncomment this when using Yaesu emulation with Ham Radio Deluxe

#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#define FEATURE_AZIMUTH_CORRECTION        // correct the azimuth using a calibration table in rotator_settings.h
// #define FEATURE_ELEVATION_CORRECTION      // correct the elevation using a calibration table in rotator_settings.h

#define OPTION_EL_SPEED_FOLLOWS_AZ_SPEED    // changing the azimith speed with Yaesu X commands or an azimuth speed pot will also change elevation speed

#define OPTION_DISPLAY_STATUS
#define OPTION_DISPLAY_HEADING
#define OPTION_DISPLAY_HEADING_AZ_ONLY
#define OPTION_DISPLAY_HEADING_EL_ONLY
#define OPTION_DISPLAY_HHMM_CLOCK  // display HH:MM clock  (set position with #define LCD_HHMM_CLOCK_POSITION)
// #define OPTION_DISPLAY_HHMMSS_CLOCK  // display HH:MM:SS clock  (set position with #define LCD_HHMMSS_CLOCK_POSITION)
// #define OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD // display alternating HH:MM clock and maidenhead on LCD row 1 (set position with #define LCD_HHMMCLOCK_POSITION)
// #define OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD // display constant HH:MM:SS clock and maidenhead on LCD row 1 (set position with #define LCD_CONSTANT_HHMMSSCLOCK_MAIDENHEAD_POSITION)
// #define OPTION_DISPLAY_BIG_CLOCK   // display date & time clock (set row with #define LCD_BIG_CLOCK_ROW)
// #define OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
#define OPTION_DISPLAY_DIRECTION_STATUS // N, W, E, S, NW, etc. direction indicator

#define OPTION_DISPLAY_VERSION_ON_STARTUP
// #define OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE

// #define OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
#define OPTION_BLINK_OVERLAP_LED

#define DEFAULT_DEBUG_STATE 1 // 1 = activate debug mode at startup

// #define DEBUG_DUMP
// #define DEBUG_LOOP
// #define DEBUG_SERIAL
// #define DEBUG_SERVICE_REQUEST_QUEUE
// #define DEBUG_EEPROM
// #define DEBUG_AZ_SPEED_POT
// #define DEBUG_AZ_PRESET_POT
// #define DEBUG_BRAKE
// #define DEBUG_OVERLAP
// #define DEBUG_DISPLAY
// #define DEBUG_AZ_CHECK_OPERATION_TIMEOUT
// #define DEBUG_TIMED_BUFFER
// #define DEBUG_EL_CHECK_OPERATION_TIMEOUT
// #define DEBUG_VARIABLE_OUTPUTS
// #define DEBUG_ROTATOR
// #define DEBUG_SUBMIT_REQUEST
// #define DEBUG_SERVICE_ROTATION
// #define DEBUG_HEADING_READING_TIME
// #define DEBUG_ROTATION_INDICATION_PIN
// #define DEBUG_PARK
#define DEBUG_PROCESS_YAESU
// #define DEBUG_AUTOCORRECT
