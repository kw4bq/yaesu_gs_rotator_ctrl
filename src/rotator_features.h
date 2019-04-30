

#define FEATURE_ELEVATION_CONTROL      // uncomment this for AZ/EL rotators
#define FEATURE_YAESU_EMULATION

#if defined(FEATURE_YAESU_EMULATION)
  #define CONTROL_PROTOCOL_EMULATION
#endif

// #define FEATURE_MOON_TRACKING
// #define FEATURE_SUN_TRACKING
// #define FEATURE_CLOCK
// #define FEATURE_GPS
// #define FEATURE_RTC_DS1307
// #define FEATURE_RTC_PCF8583
// #define FEATURE_AUTOCORRECT

#define LANGUAGE_ENGLISH


#define FEATURE_AZ_POSITION_POTENTIOMETER
#define FEATURE_EL_POSITION_POTENTIOMETER

// #define FEATURE_4_BIT_LCD_DISPLAY // Uncomment for classic 4 bit LCD display (most common)
#define FEATURE_SUNFOUNDER_I2C_LCD

// #define FEATURE_ANALOG_OUTPUT_PINS

// #define FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION
// #define FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION

#define OPTION_GS_232B_EMULATION
// #define FEATURE_ROTATION_INDICATOR_PIN     // activate rotation_indication_pin to indicate rotation
// #define FEATURE_LIMIT_SENSE
#define FEATURE_TIMED_BUFFER           // Support for Yaesu timed buffer commands
#define OPTION_SERIAL_HELP_TEXT        // Yaesu help command prints help
// #define OPTION_AZ_MANUAL_ROTATE_LIMITS    // this option will automatically stop the L and R commands when hitting a CCW or CW limit (settings are AZ_MANUAL_ROTATE_CCW_LIMIT, AZ_MANUAL_ROTATE_CW_LIMIT)
// #define OPTION_EL_MANUAL_ROTATE_LIMITS    // (settings are EL_MANUAL_ROTATE_DOWN_LIMIT, EL_MANUAL_ROTATE_UP_LIMIT)


// #define OPTION_C_COMMAND_SENDS_AZ_AND_EL  // uncomment this when using Yaesu emulation with Ham Radio Deluxe
// #define OPTION_DELAY_C_CMD_OUTPUT         // uncomment this when using Yaesu emulation with Ham Radio Deluxe

#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#define FEATURE_AZIMUTH_CORRECTION        // correct the azimuth using a calibration table in rotator_settings.h
// #define FEATURE_ELEVATION_CORRECTION      // correct the elevation using a calibration table in rotator_settings.h

// #define FEATURE_ANCILLARY_PIN_CONTROL     // control I/O pins with serial commands \F, \N, \P
// #define FEATURE_JOYSTICK_CONTROL          // analog joystick support
// #define OPTION_JOYSTICK_REVERSE_X_AXIS
// #define OPTION_JOYSTICK_REVERSE_Y_AXIS
#define OPTION_EL_SPEED_FOLLOWS_AZ_SPEED    // changing the azimith speed with Yaesu X commands or an azimuth speed pot will also change elevation speed
// #define OPTION_PULSE_IGNORE_AMBIGUOUS_PULSES // for azimuth and elevation position pulse input feature, ignore pulses that arrive when no rotation is active
// #define OPTION_BUTTON_RELEASE_NO_SLOWDOWN  // disables slowdown when CW or CCW button is released, or stop button is depressed
#define OPTION_SYNC_RTC_TO_GPS // if both realtime clock and GPS are present, synchronize realtime clock to GPS

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
#define OPTION_DISPLAY_GPS_INDICATOR  // display GPS indicator on LCD - set position with LCD_GPS_INDICATOR_POSITION and LCD_GPS_INDICATOR_ROW
// #define OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY
// #define OPTION_DISPLAY_DIRECTION_STATUS // N, W, E, S, NW, etc. direction indicator
// #define OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY

#define OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL
#define OPTION_DISPLAY_VERSION_ON_STARTUP
// #define OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE

// #define FEATURE_POWER_SWITCH
// #define OPTION_EXTERNAL_ANALOG_REFERENCE

// #define OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
// #define OPTION_BLINK_OVERLAP_LED

// #define OPTION_DONT_READ_GPS_PORT_AS_OFTEN
// #define OPTION_GPS_DO_PORT_FLUSHES
// #define OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING  // change OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING_STRING in settings file
// #define OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING
// #define OPTION_MORE_SERIAL_CHECKS

#define DEFAULT_DEBUG_STATE 1 // 1 = activate debug mode at startup

#define DEBUG_DUMP  // normally compile with this activated unless you're really trying to save memory
// #define DEBUG_LOOP
// #define DEBUG_BUTTONS
// #define DEBUG_SERIAL
// #define DEBUG_SERVICE_REQUEST_QUEUE
#define DEBUG_EEPROM
// #define DEBUG_AZ_SPEED_POT
// #define DEBUG_AZ_PRESET_POT
#define DEBUG_PRESET_ENCODERS
// #define DEBUG_AZ_MANUAL_ROTATE_LIMITS
// #define DEBUG_EL_MANUAL_ROTATE_LIMITS
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
// #define DEBUG_POSITION_ROTARY_ENCODER
// #define DEBUG_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY
// #define DEBUG_POSITION_PULSE_INPUT
// #define DEBUG_ACCEL
// #define DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
// #define DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER_BAD_DATA
// #define DEBUG_HEADING_READING_TIME
// #define DEBUG_JOYSTICK
// #define DEBUG_ROTATION_INDICATION_PIN
// #define DEBUG_HH12
// #define DEBUG_PARK
// #define DEBUG_LIMIT_SENSE
// #define DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
// #define DEBUG_EL_POSITION_INCREMENTAL_ENCODER
// #define DEBUG_MOON_TRACKING
// #define DEBUG_SUN_TRACKING
// #define DEBUG_GPS
// #define DEBUG_GPS_SERIAL
// #define DEBUG_OFFSET
// #define DEBUG_RTC
// #define DEBUG_PROCESS_YAESU
// #define DEBUG_ETHERNET
// #define DEBUG_PROCESS_SLAVE
// #define DEBUG_MEMSIC_2125
// #define DEBUG_SYNC_MASTER_CLOCK_TO_SLAVE
// #define DEBUG_SYNC_MASTER_COORDINATES_TO_SLAVE
// #define DEBUG_HMC5883L
// #define DEBUG_POLOLU_LSM303_CALIBRATION
// #define DEBUG_STEPPER
// #define DEBUG_AUTOCORRECT
// #define DEBUG_A2_ENCODER
// #define DEBUG_A2_ENCODER_LOOPBACK_TEST
// #define DEBUG_QMC5883
