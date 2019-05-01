#include "rotator_controller.h"

void setup() {
  delay(1000);
  initialize_serial();
  initialize_peripherals();
  read_settings_from_eeprom();
  initialize_pins();
  read_azimuth(0);
  initialize_display();
  initialize_interrupts();
} // setup

void loop() {

  #ifdef DEBUG_LOOP
  debug.print("loop()\n");
  Serial.flush();
  #endif // DEBUG_LOOP

  check_serial();

  read_headings();

  #ifdef FEATURE_LCD_DISPLAY
  update_display();
  #endif

  #ifdef DEBUG_DUMP
  output_debug();
  #endif //DEBUG_DUMP

  read_headings();

  check_for_dirty_configuration();

  #ifdef FEATURE_JOYSTICK_CONTROL
  check_joystick();
  #endif // FEATURE_JOYSTICK_CONTROL

  #ifdef FEATURE_ROTATION_INDICATOR_PIN
  service_rotation_indicator_pin();
  #endif // FEATURE_ROTATION_INDICATOR_PIN

  #ifdef FEATURE_LIMIT_SENSE
  check_limit_sense();
  #endif // FEATURE_LIMIT_SENSE

  #ifdef FEATURE_MOON_TRACKING
  service_moon_tracking();
  #endif // FEATURE_MOON_TRACKING

  #ifdef FEATURE_SUN_TRACKING
  service_sun_tracking();
  #endif // FEATURE_SUN_TRACKING

  #ifdef FEATURE_GPS
  service_gps();
  #endif // FEATURE_GPS

  read_headings();

  #ifdef FEATURE_RTC
  service_rtc();
  #endif // FEATURE_RTC

  #ifdef FEATURE_POWER_SWITCH
  service_power_switch();
  #endif //FEATURE_POWER_SWITCH

  service_blink_led();

  #ifdef FEATURE_ANALOG_OUTPUT_PINS
  service_analog_output_pins();
  #endif //FEATURE_ANALOG_OUTPUT_PINS

  #if defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)
  check_sun_pushbutton_calibration();
  #endif //defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)

  #if defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)
  check_moon_pushbutton_calibration();
  #endif //defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)

  #if defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) || defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER)
  service_a2_encoders();
  #endif //defined(FEATURE_AZ_POSITION_A2_ABSOLUTE_ENCODER) || defined(FEATURE_EL_POSITION_A2_ABSOLUTE_ENCODER)

  check_for_reset_flag();

} // loop

void read_headings() {
  #ifdef DEBUG_LOOP
  debug.print("read_headings()\n");
  #endif // DEBUG_LOOP

  read_azimuth(0);

  #ifdef FEATURE_ELEVATION_CONTROL
  read_elevation(0);
  #endif
} // read_headings

void service_blink_led() {
  #ifdef blink_led
  static unsigned long last_blink_led_transition = 0;
  static byte blink_led_status = 0;


  if (((millis() - last_blink_led_transition) >= 1000) && (blink_led != 0)) {
    if (blink_led_status) {
      digitalWriteEnhanced(blink_led, LOW);
      blink_led_status = 0;
    } else {
      digitalWriteEnhanced(blink_led, HIGH);
      blink_led_status = 1;
    }
    last_blink_led_transition = millis();
  }
  #endif // blink_led

} // service_blink_led

void check_for_reset_flag() {

  static unsigned long detected_reset_flag_time = 0;

  if (reset_the_unit){
    if (detected_reset_flag_time == 0){
      detected_reset_flag_time = millis();
    } else {
      if ((millis()-detected_reset_flag_time) > 5000){  // let things run for 5 seconds


        #ifdef reset_pin
        digitalWrite(reset_pin,HIGH);
        #else // reset_pin

        #ifdef OPTION_RESET_METHOD_JMP_ASM_0
        asm volatile ("  jmp 0"); // reboot!     // doesn't work on Arduino Mega but works on SainSmart Mega.
        //wdt_enable(WDTO_30MS); while(1) {};  //doesn't work on Mega
        #else //OPTION_RESET_METHOD_JMP_ASM_0
        setup();
        reset_the_unit = 0;
        #endif //OPTION_RESET_METHOD_JMP_ASM_0

        #endif //reset_pin
      }
    }
  }

} // check_for_reset_flag

void check_az_speed_pot() {

  static unsigned long last_pot_check_time = 0;
  int pot_read = 0;
  byte new_azimuth_speed_voltage = 0;

  if (az_speed_pot && ((millis() - last_pot_check_time) > 500)) {
    pot_read = analogReadEnhanced(az_speed_pot);
    new_azimuth_speed_voltage = map(pot_read, SPEED_POT_LOW, SPEED_POT_HIGH, SPEED_POT_LOW_MAP, SPEED_POT_HIGH_MAP);
    if (new_azimuth_speed_voltage != normal_az_speed_voltage) {
      #ifdef DEBUG_AZ_SPEED_POT
      if (debug_mode) {
        debug.print("check_az_speed_pot: normal_az_speed_voltage: ");
        debug.print(normal_az_speed_voltage);
        debug.print(" new_azimuth_speed_voltage:");
        debug.print(new_azimuth_speed_voltage);
        debug.println("");
      }
      #endif // DEBUG_AZ_SPEED_POT
      normal_az_speed_voltage = new_azimuth_speed_voltage;
      update_az_variable_outputs(normal_az_speed_voltage);
      #if defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED) && defined(FEATURE_ELEVATION_CONTROL)
      normal_el_speed_voltage = new_azimuth_speed_voltage;
      update_el_variable_outputs(normal_el_speed_voltage);
      #endif // OPTION_EL_SPEED_FOLLOWS_AZ_SPEED
    }
    last_pot_check_time = millis();
  }

} // check_az_speed_pot

void check_az_preset_potentiometer() {

  byte check_pot = 0;
  static unsigned long last_pot_check_time = 0;
  static int last_pot_read = 9999;
  int pot_read = 0;
  int new_pot_azimuth = 0;
  byte button_read = 0;
  static byte pot_changed_waiting = 0;

  if (az_preset_pot) {
    if (last_pot_read == 9999) {  // initialize last_pot_read the first time we hit this subroutine
      last_pot_read = analogReadEnhanced(az_preset_pot);
    }

    if (!pot_changed_waiting) {
      if (preset_start_button) { // if we have a preset start button, check it
        button_read = digitalReadEnhanced(preset_start_button);
        if (button_read == BUTTON_ACTIVE_STATE) {
          check_pot = 1;
        }
      } else {  // if not, check the pot every 500 mS
        if ((millis() - last_pot_check_time) < 250) {
          check_pot = 1;
        }
      }

      if (check_pot) {
        pot_read = analogReadEnhanced(az_preset_pot);
        new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
        if ((abs(last_pot_read - pot_read) > 4) && (abs(new_pot_azimuth - (raw_azimuth / HEADING_MULTIPLIER)) > AZIMUTH_TOLERANCE)) {
          pot_changed_waiting = 1;
          #ifdef DEBUG_AZ_PRESET_POT
          if (debug_mode) {
            debug.println("check_az_preset_potentiometer: in pot_changed_waiting");
          }
          #endif // DEBUG_AZ_PRESET_POT
          last_pot_read = pot_read;
        }
      }
      last_pot_check_time = millis();
    } else {  // we're in pot change mode
    pot_read = analogReadEnhanced(az_preset_pot);
    if (abs(pot_read - last_pot_read) > 3) {  // if the pot has changed, reset the timer
      last_pot_check_time = millis();
      last_pot_read = pot_read;
    } else {
      if ((millis() - last_pot_check_time) >= 250) {  // has it been awhile since the last pot change?
        new_pot_azimuth = map(pot_read, AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP);
        #ifdef DEBUG_AZ_PRESET_POT
        if (debug_mode) {
          debug.print("check_az_preset_potentiometer: pot change - current raw_azimuth: ");
          debug.print(raw_azimuth / HEADING_MULTIPLIER,0);
          debug.print(" new_azimuth: ");
          debug.print(new_pot_azimuth);
          debug.println("");
        }
        #endif // DEBUG_AZ_PRESET_POT
        submit_request(AZ, REQUEST_AZIMUTH_RAW, new_pot_azimuth * HEADING_MULTIPLIER, 44);
        pot_changed_waiting = 0;
        last_pot_read = pot_read;
        last_pot_check_time = millis();
      }
    }
  }
} // if (az_preset_pot)
} // check_az_preset_potentiometer

void check_brake_release() {


  static byte in_az_brake_release_delay = 0;
  static unsigned long az_brake_delay_start_time = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
  static byte in_el_brake_release_delay = 0;
  static unsigned long el_brake_delay_start_time = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if ((az_state == IDLE) && (brake_az_engaged)) {
    if (in_az_brake_release_delay) {
      if ((millis() - az_brake_delay_start_time) > AZ_BRAKE_DELAY) {
        brake_release(AZ, BRAKE_RELEASE_OFF);
        in_az_brake_release_delay = 0;
      }
    } else {
      az_brake_delay_start_time = millis();
      in_az_brake_release_delay = 1;
    }
  }

  if ((az_state != IDLE) && (brake_az_engaged)) {in_az_brake_release_delay = 0;}

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((el_state == IDLE) && (brake_el_engaged)) {
    if (in_el_brake_release_delay) {
      if ((millis() - el_brake_delay_start_time) > EL_BRAKE_DELAY) {
        brake_release(EL, BRAKE_RELEASE_OFF);
        in_el_brake_release_delay = 0;
      }
    } else {
      el_brake_delay_start_time = millis();
      in_el_brake_release_delay = 1;
    }
  }

  if ((el_state != IDLE) && (brake_el_engaged)) {in_el_brake_release_delay = 0;}
  #endif // FEATURE_ELEVATION_CONTROL

} /* check_brake_release */

void brake_release(byte az_or_el, byte operation) {

  if (az_or_el == AZ) {
    if (brake_az && (configuration.brake_az_disabled == 0)) {
      if (operation == BRAKE_RELEASE_ON) {
        digitalWriteEnhanced(brake_az, BRAKE_ACTIVE_STATE);
        brake_az_engaged = 1;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_az BRAKE_RELEASE_ON");
        #endif // DEBUG_BRAKE
      } else {
        digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
        brake_az_engaged = 0;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_az BRAKE_RELEASE_OFF");
        #endif // DEBUG_BRAKE
      }
    }
  } else {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (brake_el) {
      if (operation == BRAKE_RELEASE_ON) {
        digitalWriteEnhanced(brake_el, BRAKE_ACTIVE_STATE);
        brake_el_engaged = 1;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_el BRAKE_RELEASE_ON");
        #endif // DEBUG_BRAKE
      } else {
        digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
        brake_el_engaged = 0;
        #ifdef DEBUG_BRAKE
        debug.println("brake_release: brake_el BRAKE_RELEASE_OFF");
        #endif // DEBUG_BRAKE
      }
    }
    #endif // FEATURE_ELEVATION_CONTROL
  }
} // brake_release

void check_overlap() {

  static byte overlap_led_status = 0;
  static unsigned long last_check_time;
  #ifdef OPTION_BLINK_OVERLAP_LED
  static unsigned long last_overlap_led_transition = 0;
  static byte blink_status = 0;
  #endif //OPTION_BLINK_OVERLAP_LED

  if ((overlap_led) && ((millis() - last_check_time) > 500)) {
    // if ((analog_az > (500*HEADING_MULTIPLIER)) && (azimuth > (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (!overlap_led_status)) {
    if ((raw_azimuth > (ANALOG_AZ_OVERLAP_DEGREES * HEADING_MULTIPLIER)) && (!overlap_led_status)) {
      digitalWriteEnhanced(overlap_led, OVERLAP_LED_ACTIVE_STATE);
      overlap_led_status = 1;
      #ifdef OPTION_BLINK_OVERLAP_LED
      last_overlap_led_transition = millis();
      blink_status = 1;
      #endif //OPTION_BLINK_OVERLAP_LED
      #ifdef DEBUG_OVERLAP
      debug.println("check_overlap: in overlap");
      #endif // DEBUG_OVERLAP
    } else {
      // if (((analog_az < (500*HEADING_MULTIPLIER)) || (azimuth < (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER))) && (overlap_led_status)) {
      if ((raw_azimuth < (ANALOG_AZ_OVERLAP_DEGREES * HEADING_MULTIPLIER)) && (overlap_led_status)) {
        digitalWriteEnhanced(overlap_led, OVERLAP_LED_INACTIVE_STATE);
        overlap_led_status = 0;
        #ifdef DEBUG_OVERLAP
        debug.println("check_overlap: overlap off");
        #endif // DEBUG_OVERLAP
      }
    }
    last_check_time = millis();

  }

  #ifdef OPTION_BLINK_OVERLAP_LED
  if ((overlap_led_status) && ((millis() - last_overlap_led_transition) >= OPTION_OVERLAP_LED_BLINK_MS)){
    if (blink_status){
      digitalWriteEnhanced(overlap_led, OVERLAP_LED_INACTIVE_STATE);
      blink_status = 0;
    } else {
      digitalWriteEnhanced(overlap_led, OVERLAP_LED_ACTIVE_STATE);
      blink_status = 1;
    }
    last_overlap_led_transition = millis();
  }
  #endif //OPTION_BLINK_OVERLAP_LED

} // check_overlap

void clear_command_buffer() {

  control_port_buffer_index = 0;
  control_port_buffer[0] = 0;
} // clear_command_buffer

void check_serial() {

  #ifdef DEBUG_LOOP
  debug.print("check_serial\n");
  Serial.flush();
  #endif // DEBUG_LOOP

  static unsigned long serial_led_time = 0;
  float tempfloat = 0;
  char return_string[100] = "";

  #if defined(FEATURE_GPS)
  static byte gps_port_read = 0;
  static byte gps_port_read_data_sent = 0;
  static byte gps_missing_terminator_flag = 0;
  #endif

  #ifdef FEATURE_CLOCK
  int temp_year = 0;
  byte temp_month = 0;
  byte temp_day = 0;
  byte temp_minute = 0;
  byte temp_hour = 0;
  #endif // FEATURE_CLOCK

  #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
  char grid[10] = "";
  byte hit_error = 0;
  #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

  #ifdef FEATURE_GPS

  if (gps_missing_terminator_flag){
    gps.encode('$');
    gps_missing_terminator_flag = 0;
    gps_port_read_data_sent = 1;
  } else {

    #if defined(OPTION_DONT_READ_GPS_PORT_AS_OFTEN)
    if (gps_port->available()) {
      gps_port_read = gps_port->read();
      #ifdef GPS_MIRROR_PORT
      gps_mirror_port->write(gps_port_read);
      #endif //GPS_MIRROR_PORT
      #if defined(DEBUG_GPS_SERIAL)
      debug.write(gps_port_read);
      if (gps_port_read == 10){debug.write(13);}
      #endif //DEBUG_GPS_SERIAL
      #if defined(DEBUG_GPS_SERIAL) || defined(OPTION_GPS_DO_PORT_FLUSHES)
      port_flush();
      #endif

      #if defined(OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING)
      if (gps.encode(gps_port_read)) {
        gps_data_available = 1;

        #ifdef DEBUG_GPS
        unsigned long gps_chars = 0;
        unsigned short gps_good_sentences = 0;
        unsigned short gps_failed_checksum = 0;
        char gps_temp_string[12] = "";
        float gps_lat_temp = 0;
        float gps_long_temp = 0;

        debug.print("\tGPS: satellites:");
        gps_chars = gps.satellites();
        //if (gps_chars == 255){gps_chars = 0;}
        dtostrf(gps_chars,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        unsigned long gps_fix_age_temp = 0;
        gps.f_get_position(&gps_lat_temp,&gps_long_temp,&gps_fix_age_temp);
        debug.print("  lat:");
        debug.print(gps_lat_temp,4);
        debug.print("  long:");
        debug.print(gps_long_temp,4);
        debug.print("  altitude(m):");
        debug.print(gps.altitude()/100,0);
        debug.print("  fix_age_mS:");
        dtostrf(gps_fix_age_temp,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        gps.stats(&gps_chars,&gps_good_sentences,&gps_failed_checksum);
        debug.print("  data_chars:");
        dtostrf(gps_chars,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.print("  good_sentences:");
        dtostrf(gps_good_sentences,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.print("  failed_checksum:");
        dtostrf(gps_failed_checksum,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.println("");
        #endif //FEATURE_GPS


      }
      #else
      if ((gps_port_read == '$') && (gps_port_read_data_sent)){ // handle missing LF/CR
        if (gps.encode('\r')) {
          gps_data_available = 1;
          gps_missing_terminator_flag = 1;
        } else {
          gps.encode(gps_port_read);
        }
      } else {
        if (gps.encode(gps_port_read)) {
          gps_data_available = 1;
          gps_port_read_data_sent = 0;

          #ifdef DEBUG_GPS
          unsigned long gps_chars = 0;
          unsigned short gps_good_sentences = 0;
          unsigned short gps_failed_checksum = 0;
          char gps_temp_string[12] = "";
          float gps_lat_temp = 0;
          float gps_long_temp = 0;

          debug.print("\tGPS: satellites:");
          gps_chars = gps.satellites();
          //if (gps_chars == 255){gps_chars = 0;}
          dtostrf(gps_chars,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          unsigned long gps_fix_age_temp = 0;
          gps.f_get_position(&gps_lat_temp,&gps_long_temp,&gps_fix_age_temp);
          debug.print("  lat:");
          debug.print(gps_lat_temp,4);
          debug.print("  long:");
          debug.print(gps_long_temp,4);
          debug.print("  fix_age_mS:");
          dtostrf(gps_fix_age_temp,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          gps.stats(&gps_chars,&gps_good_sentences,&gps_failed_checksum);
          debug.print("  data_chars:");
          dtostrf(gps_chars,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          debug.print("  good_sentences:");
          dtostrf(gps_good_sentences,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          debug.print("  failed_checksum:");
          dtostrf(gps_failed_checksum,0,0,gps_temp_string);
          debug.print(gps_temp_string);
          debug.println("");
          #endif //FEATURE_GPS


        } else {
          gps_port_read_data_sent = 1;
        }
      }
      #endif  //  OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING
    }
    #else //OPTION_DONT_READ_GPS_PORT_AS_OFTEN
    while ((gps_port->available()) /*&& (!gps_data_available)*/) {
      gps_port_read = gps_port->read();
      #ifdef GPS_MIRROR_PORT
      gps_mirror_port->write(gps_port_read);
      #endif //GPS_MIRROR_PORT
      #if defined(DEBUG_GPS_SERIAL)
      debug.write(gps_port_read);
      if (gps_port_read == 10){debug.write(13);}
      #endif //DEBUG_GPS_SERIAL
      #if defined(DEBUG_GPS_SERIAL) || defined(OPTION_GPS_DO_PORT_FLUSHES)
      port_flush();
      #endif
      #if defined(OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING)
      if (gps.encode(gps_port_read)) {
        gps_data_available = 1;
      }
      #else
      if ((gps_port_read == '$') && (gps_port_read_data_sent)){ // handle missing LF/CR
        if (gps.encode('\r')) {
          gps_data_available = 1;
          gps_missing_terminator_flag = 1;
        } else {
          gps.encode(gps_port_read);
        }
      } else {
        if (gps.encode(gps_port_read)) {
          gps_data_available = 1;
          gps_port_read_data_sent = 0;
        } else {
          gps_port_read_data_sent = 1;
        }
      }
      #endif  //  OPTION_GPS_EXCLUDE_MISSING_LF_CR_HANDLING

    }
    #endif //OPTION_DONT_READ_GPS_PORT_AS_OFTEN

  } // if (gps_missing_terminator_flag)

  #endif // FEATURE_GPS

  #if defined(GPS_MIRROR_PORT) && defined(FEATURE_GPS)
  if (gps_mirror_port->available()) {
    gps_port->write(gps_mirror_port->read());
  }
  #endif //defined(GPS_MIRROR_PORT) && defined(FEATURE_GPS)

} // check_serial

void check_buttons() {

  #ifdef FEATURE_ELEVATION_CONTROL
  #ifdef FEATURE_ADAFRUIT_BUTTONS
  if (buttons & 0x08) {
    #else
    if (button_up && (digitalReadEnhanced(button_up) == BUTTON_ACTIVE_STATE)) {
      #endif // FEATURE_ADAFRUIT_BUTTONS
      if (elevation_button_was_pushed == 0) {
        #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
        if (elevation < (EL_MANUAL_ROTATE_UP_LIMIT * HEADING_MULTIPLIER)) {
          submit_request(EL, REQUEST_UP, 0, 66);
          elevation_button_was_pushed = 1;
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: button_up pushed");
          #endif // DEBUG_BUTTONS
        } else {
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: button_up pushed but at EL_MANUAL_ROTATE_UP_LIMIT");
          #endif // DEBUG_BUTTONS
        }
        #else
        submit_request(EL, REQUEST_UP, 0, 66);
        elevation_button_was_pushed = 1;
        #ifdef DEBUG_BUTTONS
        debug.println("check_buttons: button_up pushed");
        #endif // DEBUG_BUTTONS
        #endif //OPTION_EL_MANUAL_ROTATE_LIMITS
      }
    } else {
      #ifdef FEATURE_ADAFRUIT_BUTTONS
      if (buttons & 0x04) {
        #else
        if (button_down && (digitalReadEnhanced(button_down) == BUTTON_ACTIVE_STATE)) {
          #endif // FEATURE_ADAFRUIT_BUTTONS
          if (elevation_button_was_pushed == 0) {

            #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
            if (elevation > (EL_MANUAL_ROTATE_DOWN_LIMIT * HEADING_MULTIPLIER)) {
              submit_request(EL, REQUEST_DOWN, 0, 67);
              elevation_button_was_pushed = 1;
              #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: button_down pushed");
              #endif // DEBUG_BUTTONS
            } else {
              #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: button_down pushed but at EL_MANUAL_ROTATE_DOWN_LIMIT");
              #endif // DEBUG_BUTTONS
            }
            #else
            submit_request(EL, REQUEST_DOWN, 0, 67);
            elevation_button_was_pushed = 1;
            #ifdef DEBUG_BUTTONS
            debug.println("check_buttons: button_down pushed");
            #endif // DEBUG_BUTTONS
            #endif
          }
        }
      }

      #ifdef FEATURE_ADAFRUIT_BUTTONS
      if ((elevation_button_was_pushed) && (!(buttons & 0x0C))) {
        #ifdef DEBUG_BUTTONS
        debug.println("check_buttons: no EL button depressed");
        #endif // DEBUG_BUTTONS
        #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
        submit_request(EL, REQUEST_STOP, 0, 68);
        #else
        submit_request(EL, REQUEST_KILL, 0, 69);
        #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
        elevation_button_was_pushed = 0;
      }

      #else
      if ((elevation_button_was_pushed) && (digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
        delay(200);
        if ((digitalReadEnhanced(button_up) == BUTTON_INACTIVE_STATE) && (digitalReadEnhanced(button_down) == BUTTON_INACTIVE_STATE)) {
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: no EL button depressed");
          #endif // DEBUG_BUTTONS
          #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          submit_request(EL, REQUEST_STOP, 0, 70);
          #else
          submit_request(EL, REQUEST_KILL, 0, 71);
          #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          elevation_button_was_pushed = 0;
        }
      }
      #endif // FEATURE_ADAFRUIT_BUTTONS

      #endif // FEATURE_ELEVATION_CONTROL

      if (button_stop) {
        if ((digitalReadEnhanced(button_stop) == BUTTON_ACTIVE_STATE)) {
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: button_stop pushed");
          #endif // DEBUG_BUTTONS
          #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          submit_request(AZ, REQUEST_STOP, 0, 74);
          #else
          submit_request(AZ, REQUEST_KILL, 0, 75);
          #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          #ifdef FEATURE_ELEVATION_CONTROL
          #ifndef OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          submit_request(EL, REQUEST_STOP, 0, 76);
          #else
          submit_request(EL, REQUEST_KILL, 0, 77);
          #endif // OPTION_BUTTON_RELEASE_NO_SLOWDOWN
          #endif // FEATURE_ELEVATION_CONTROL
        }
      }

      #ifdef FEATURE_MOON_TRACKING
      static byte moon_tracking_button_pushed = 0;
      static unsigned long last_time_moon_tracking_button_pushed = 0;
      if (moon_tracking_button) {
        if ((digitalReadEnhanced(moon_tracking_button) == BUTTON_ACTIVE_STATE)) {
          moon_tracking_button_pushed = 1;
          last_time_moon_tracking_button_pushed = millis();
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: moon_tracking_button pushed");
          #endif // DEBUG_BUTTONS
        } else {
          if ((moon_tracking_button_pushed) && ((millis() - last_time_moon_tracking_button_pushed) >= 250)) {
            if (!moon_tracking_active) {
              #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: moon tracking on");
              #endif // DEBUG_BUTTONS
              moon_tracking_active = 1;
              #ifdef FEATURE_SUN_TRACKING
              sun_tracking_active = 0;
              #endif // FEATURE_SUN_TRACKING
            } else {
              #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: moon tracking off");
              #endif // DEBUG_BUTTONS
              moon_tracking_active = 0;
            }
            moon_tracking_button_pushed = 0;
          }
        }
      }
      #endif // FEATURE_MOON_TRACKING

      #ifdef FEATURE_SUN_TRACKING
      static byte sun_tracking_button_pushed = 0;
      static unsigned long last_time_sun_tracking_button_pushed = 0;
      if (sun_tracking_button) {
        if ((digitalReadEnhanced(sun_tracking_button) == BUTTON_ACTIVE_STATE)) {
          sun_tracking_button_pushed = 1;
          last_time_sun_tracking_button_pushed = millis();
          #ifdef DEBUG_BUTTONS
          debug.println("check_buttons: sun_tracking_button pushed");
          #endif // DEBUG_BUTTONS
        } else {
          if ((sun_tracking_button_pushed) && ((millis() - last_time_sun_tracking_button_pushed) >= 250)) {
            if (!sun_tracking_active) {
              #ifdef DEBUG_BUTTONS
              debug.println("check_buttons: sun tracking on");
              #endif // DEBUG_BUTTONS
              sun_tracking_active = 1;
              #ifdef FEATURE_MOON_TRACKING
              moon_tracking_active = 0;
              #endif // FEATURE_MOON_TRACKING
            } else {
              #ifdef DEBUG_BUTTONS
              debug.print("check_buttons: sun tracking off");
              #endif // DEBUG_BUTTONS
              sun_tracking_active = 0;
            }
            sun_tracking_button_pushed = 0;
          }
        }
      }
      #endif // FEATURE_SUN_TRACKING

    } /* check_buttons */

    #ifdef FEATURE_LCD_DISPLAY
    char * idle_status(){


      #ifdef OPTION_DISPLAY_DIRECTION_STATUS
      return azimuth_direction(azimuth);
      #endif //OPTION_DISPLAY_DIRECTION_STATUS

      return("");



    }


    #endif //FEATURE_LCD_DISPLAY


    #if defined(FEATURE_LCD_DISPLAY) && defined(OPTION_DISPLAY_DIRECTION_STATUS)
    char * azimuth_direction(int azimuth_in){

      azimuth_in = azimuth_in / HEADING_MULTIPLIER;



      if (azimuth_in > 348) {
        return N_STRING;
      }
      if (azimuth_in > 326) {
        return NNW_STRING;
      }
      if (azimuth_in > 303) {
        return NW_STRING;
      }
      if (azimuth_in > 281) {
        return WNW_STRING;
      }
      if (azimuth_in > 258) {
        return W_STRING;
      }
      if (azimuth_in > 236) {
        return WSW_STRING;
      }
      if (azimuth_in > 213) {
        return SW_STRING;
      }
      if (azimuth_in > 191) {
        return SSW_STRING;
      }
      if (azimuth_in > 168) {
        return S_STRING;
      }
      if (azimuth_in > 146) {
        return SSE_STRING;
      }
      if (azimuth_in > 123) {
        return SE_STRING;
      }
      if (azimuth_in > 101) {
        return ESE_STRING;
      }
      if (azimuth_in > 78) {
        return E_STRING;
      }
      if (azimuth_in > 56) {
        return ENE_STRING;
      }
      if (azimuth_in > 33) {
        return NE_STRING;
      }
      if (azimuth_in > 11) {
        return NNE_STRING;
      }
      return N_STRING;

    } /* azimuth_direction */
    #endif /* ifdef FEATURE_LCD_DISPLAY */


    #if defined(FEATURE_LCD_DISPLAY)
    void update_display(){


      byte force_display_update_now = 0;
      char workstring[32] = "";
      char workstring2[32] = "";
      byte row_override[LCD_ROWS+1];

      for (int x = 0;x < (LCD_ROWS+1);x++){row_override[x] = 0;}

      k3ngdisplay.clear_pending_buffer();

      #ifdef FEATURE_MOON_TRACKING
      static unsigned long last_moon_tracking_check_time = 0;
      #endif

      #ifdef FEATURE_SUN_TRACKING
      static unsigned long last_sun_tracking_check_time = 0;
      #endif

      // OPTION_DISPLAY_DIRECTION_STATUS - azimuth direction display ***********************************************************************************
      #if defined(OPTION_DISPLAY_DIRECTION_STATUS)
      strcpy(workstring,azimuth_direction(azimuth));  // TODO - add left/right/center
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_DIRECTION_ROW-1,LCD_STATUS_FIELD_SIZE);
      #endif //defined(OPTION_DISPLAY_DIRECTION_STATUS)


      // OPTION_DISPLAY_HEADING - show heading ***********************************************************************************
      #if defined(OPTION_DISPLAY_HEADING)
      #if !defined(FEATURE_ELEVATION_CONTROL)                    // ---------------- az only -----------------------------------
      strcpy(workstring,AZIMUTH_STRING);
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        dtostrf(azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        break;
        case AZ_DISPLAY_MODE_RAW:
        dtostrf(raw_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        break;
      }
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
        case AZ_DISPLAY_MODE_RAW:
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
      }
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && ((raw_azimuth/LCD_HEADING_MULTIPLIER) > ANALOG_AZ_OVERLAP_DEGREES)){
        strcat(workstring,"+");
      }
      strcat(workstring,workstring2);
      strcat(workstring,DISPLAY_DEGREES_STRING);
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HEADING_ROW-1,LCD_HEADING_FIELD_SIZE);
      #else                                                       // --------------------az & el---------------------------------
      #if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      if ((azimuth >= 1000) && (elevation >= 1000)) {
        strcpy(workstring,AZ_STRING);
      } else {
        strcpy(workstring,AZ_SPACE_STRING);
      }
      #else
      strcpy(workstring,AZ_SPACE_STRING);
      #endif // efined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        dtostrf(azimuth / LCD_HEADING_MULTIPLIER, 3, LCD_DECIMAL_PLACES, workstring2);
        break;
        case AZ_DISPLAY_MODE_RAW:
        dtostrf(raw_azimuth / LCD_HEADING_MULTIPLIER, 3, LCD_DECIMAL_PLACES, workstring2);
        break;
      }
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
        case AZ_DISPLAY_MODE_RAW:
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
      }
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && ((raw_azimuth/LCD_HEADING_MULTIPLIER) > ANALOG_AZ_OVERLAP_DEGREES)){
        strcat(workstring,"+");
      }
      strcat(workstring,workstring2);
      #if !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && !defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      if (LCD_COLUMNS > 14) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #else
      if ((LCD_COLUMNS > 18) || ((azimuth < 100) && (elevation < 100))) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #endif
      #if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      if ((elevation >= 1000) && (azimuth >= 1000)) {
        strcat(workstring,SPACE_EL_STRING);
      } else {
        strcat(workstring,SPACE_EL_SPACE_STRING);
      }
      #else
      strcat(workstring,SPACE_EL_SPACE_STRING);
      #endif // defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      dtostrf(elevation / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if ((elevation/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
      if ((elevation/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      strcat(workstring,workstring2);
      #if !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && !defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      if (LCD_COLUMNS > 14) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #else
      if ((LCD_COLUMNS > 18) || ((azimuth < 100) && (elevation < 100))) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #endif
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HEADING_ROW-1,LCD_HEADING_FIELD_SIZE);
      #endif // FEATURE_ELEVATION_CONTROL
      #endif //defined(OPTION_DISPLAY_HEADING)

      // OPTION_DISPLAY_HEADING_AZ_ONLY - show heading ***********************************************************************************
      #if defined(OPTION_DISPLAY_HEADING_AZ_ONLY)
      strcpy(workstring,AZIMUTH_STRING);
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        dtostrf(azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        break;
        case AZ_DISPLAY_MODE_RAW:
        dtostrf(raw_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        break;
      }
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      switch(configuration.azimuth_display_mode){
        case AZ_DISPLAY_MODE_NORMAL:
        case AZ_DISPLAY_MODE_OVERLAP_PLUS:
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
        case AZ_DISPLAY_MODE_RAW:
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
        if ((raw_azimuth/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
        break;
      }
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && ((raw_azimuth/LCD_HEADING_MULTIPLIER) > ANALOG_AZ_OVERLAP_DEGREES)){
        strcat(workstring,"+");
      }
      strcat(workstring,workstring2);
      strcat(workstring,DISPLAY_DEGREES_STRING);
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_AZ_ONLY_HEADING_ROW-1,LCD_AZ_ONLY_HEADING_FIELD_SIZE);
      #endif //defined(OPTION_DISPLAY_HEADING_AZ_ONLY)


      // OPTION_DISPLAY_HEADING_EL_ONLY - show heading ***********************************************************************************
      #if defined(OPTION_DISPLAY_HEADING_EL_ONLY) && defined(FEATURE_ELEVATION_CONTROL)
      // #if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      // if ((elevation >= 1000)) {
      // strcpy(workstring,SPACE_EL_STRING);
      // } else {
      // strcpy(workstring,SPACE_EL_SPACE_STRING);
      // }
      // #else
      strcpy(workstring,ELEVATION_STRING);
      // #endif // defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) || defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      dtostrf(elevation / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
      #ifdef OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      if ((elevation/ LCD_HEADING_MULTIPLIER) < 100){strcat(workstring," ");}
      if ((elevation/ LCD_HEADING_MULTIPLIER) < 10){strcat(workstring," ");}
      #endif //OPTION_LCD_HEADING_FIELD_FIXED_DECIMAL_PLACE
      strcat(workstring,workstring2);
      #if !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && !defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
      if (LCD_COLUMNS > 14) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #else
      if ((LCD_COLUMNS > 18) || (elevation < 100)) {
        strcat(workstring,DISPLAY_DEGREES_STRING);
      }
      #endif
      k3ngdisplay.print_center_fixed_field_size(workstring,LCD_EL_ONLY_HEADING_ROW-1,LCD_EL_ONLY_HEADING_FIELD_SIZE);
      #endif //defined(OPTION_DISPLAY_HEADING_EL_ONLY)

      // OPTION_DISPLAY_STATUS***********************************************************************************
      #if defined(OPTION_DISPLAY_STATUS)
      #if !defined(FEATURE_ELEVATION_CONTROL) // ---------------- az only ----------------------------------------------
      if (az_state != IDLE) {
        if (az_request_queue_state == IN_PROGRESS_TO_TARGET) {
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
          strcat(workstring," ");
          switch(configuration.azimuth_display_mode){
            case AZ_DISPLAY_MODE_NORMAL:
            case AZ_DISPLAY_MODE_OVERLAP_PLUS:
            dtostrf(target_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
            break;
            case AZ_DISPLAY_MODE_RAW:
            dtostrf(target_raw_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
            break;
          }
          if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && ((raw_azimuth/LCD_HEADING_MULTIPLIER) > ANALOG_AZ_OVERLAP_DEGREES)){
            strcat(workstring,"+");
          }
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
        } else {
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
        row_override[LCD_STATUS_ROW] = 1;
      }


      #ifdef FEATURE_AZ_PRESET_ENCODER
      float target = 0;
      if (preset_encoders_state == ENCODER_AZ_PENDING) {
        target = az_encoder_raw_degrees;
        if (target > (359 * LCD_HEADING_MULTIPLIER)) {
          target = target - (360 * LCD_HEADING_MULTIPLIER);
        }
        if (target > (359 * LCD_HEADING_MULTIPLIER)) {
          target = target - (360 * LCD_HEADING_MULTIPLIER);
        }
        strcpy(workstring,TARGET_STRING);
        dtostrf(target / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        strcat(workstring,workstring2);
        strcat(workstring,DISPLAY_DEGREES_STRING);
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
        row_override[LCD_STATUS_ROW] = 1;
      }
      #endif //FEATURE_AZ_PRESET_ENCODER

      #else                          // az & el ----------------------------------------------------------------------------



      strcpy(workstring,"");
      if (az_state != IDLE) {
        if (az_request_queue_state == IN_PROGRESS_TO_TARGET) {
          if (current_az_state() == ROTATING_CW) {
            strcat(workstring,CW_STRING);
          } else {
            strcat(workstring,CCW_STRING);
          }
          strcat(workstring," ");
          switch(configuration.azimuth_display_mode){
            case AZ_DISPLAY_MODE_NORMAL:
            case AZ_DISPLAY_MODE_OVERLAP_PLUS:
            dtostrf(target_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
            break;
            case AZ_DISPLAY_MODE_RAW:
            dtostrf(target_raw_azimuth / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
            break;
          }
          if ((configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS) && ((raw_azimuth/LCD_HEADING_MULTIPLIER) > ANALOG_AZ_OVERLAP_DEGREES)){
            strcat(workstring,"+");
          }
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          row_override[LCD_STATUS_ROW] = 1;
        } else {
          if (current_az_state() == ROTATING_CW) {
            strcpy(workstring,CW_STRING);
          } else {
            strcpy(workstring,CCW_STRING);
          }
        }
      }
      if (el_state != IDLE) {
        if (az_state != IDLE){
          strcat(workstring," ");
        }
        if (el_request_queue_state == IN_PROGRESS_TO_TARGET) {
          if (current_el_state() == ROTATING_UP) {
            strcat(workstring,UP_STRING);
          } else {
            strcat(workstring,DOWN_STRING);
          }
          strcat(workstring," ");
          dtostrf(target_elevation / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          row_override[LCD_STATUS_ROW] = 1;
        } else {
          if (current_el_state() == ROTATING_UP) {
            strcat(workstring,UP_STRING);
          } else {
            strcat(workstring,DOWN_STRING);
          }
        }
      }

      if ((az_state != IDLE) || (el_state != IDLE)){
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
      } //<added


      #if defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER)
      float target = 0;
      if (preset_encoders_state == ENCODER_AZ_PENDING) {
        target = az_encoder_raw_degrees;
        if (target > (359 * LCD_HEADING_MULTIPLIER)) {
          target = target - (360 * LCD_HEADING_MULTIPLIER);
        }
        if (target > (359 * LCD_HEADING_MULTIPLIER)) {
          target = target - (360 * LCD_HEADING_MULTIPLIER);
        }
        strcpy(workstring,TARGET_STRING);
        dtostrf(target / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
        strcat(workstring,workstring2);
        strcat(workstring,DISPLAY_DEGREES_STRING);
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
        row_override[LCD_STATUS_ROW] = 1;
      }
      #endif //defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER)

      #if defined(FEATURE_AZ_PRESET_ENCODER) && defined(FEATURE_EL_PRESET_ENCODER)
      float target = az_encoder_raw_degrees;
      if (target > (359 * LCD_HEADING_MULTIPLIER)) {
        target = target - (360 * LCD_HEADING_MULTIPLIER);
      }
      if (target > (359 * LCD_HEADING_MULTIPLIER)) {
        target = target - (360 * LCD_HEADING_MULTIPLIER);
      }

      if (preset_encoders_state != ENCODER_IDLE) {
        switch (preset_encoders_state) {
          case ENCODER_AZ_PENDING:
          strcpy(workstring,AZ_TARGET_STRING);
          dtostrf(target / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
          row_override[LCD_STATUS_ROW] = 1;
          break;
          case ENCODER_EL_PENDING:
          strcpy(workstring,EL_TARGET_STRING);
          dtostrf(el_encoder_degrees / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
          row_override[LCD_STATUS_ROW] = 1;
          break;
          case ENCODER_AZ_EL_PENDING:
          strcpy(workstring,TARGET_STRING);
          dtostrf(target / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          strcat(workstring," ");
          dtostrf(el_encoder_degrees / LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring2);
          strcat(workstring,workstring2);
          strcat(workstring,DISPLAY_DEGREES_STRING);
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_STATUS_ROW-1,LCD_STATUS_FIELD_SIZE);
          row_override[LCD_STATUS_ROW] = 1;
          break;
        } // switch
      } //if (preset_encoders_state != ENCODER_IDLE)
      #endif  //defined(FEATURE_AZ_PRESET_ENCODER) && !defined(FEATURE_EL_PRESET_ENCODER)
      /*
      */
      #endif //!defined(FEATURE_ELEVATION_CONTROL)

      #endif //defined(OPTION_DISPLAY_STATUS)

      // OPTION_DISPLAY_HHMMSS_CLOCK **************************************************************************************************
      #if defined(OPTION_DISPLAY_HHMMSS_CLOCK) && defined(FEATURE_CLOCK)

      static int last_clock_seconds = 0;

      if (!row_override[LCD_HHMMSS_CLOCK_ROW]){
        update_time();
        #ifdef OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        if (local_clock_hours < 10) {
          strcpy(workstring, "0");
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcat(workstring,workstring2);
        } else {
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcpy(workstring,workstring2);
        }
        #else
        dtostrf(local_clock_hours, 0, 0, workstring2);
        strcpy(workstring,workstring2);
        #endif //OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        strcat(workstring,":");
        if (local_clock_minutes < 10) {
          strcat(workstring, "0");
        }
        dtostrf(local_clock_minutes, 0, 0, workstring2);
        strcat(workstring,workstring2);
        strcat(workstring,":");
        if (local_clock_seconds < 10) {
          strcat(workstring, "0");
        }
        dtostrf(local_clock_seconds, 0, 0, workstring2);
        strcat(workstring,workstring2);
        if (LCD_HHMMSS_CLOCK_POSITION == LEFT){
          k3ngdisplay.print_left_fixed_field_size(workstring,LCD_HHMMSS_CLOCK_ROW-1,8);
        }
        if (LCD_HHMMSS_CLOCK_POSITION == RIGHT){
          k3ngdisplay.print_right_fixed_field_size(workstring,LCD_HHMMSS_CLOCK_ROW-1,8);
        }
        if (LCD_HHMMSS_CLOCK_POSITION == CENTER){
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HHMMSS_CLOCK_ROW-1,8);
        }
        if (last_clock_seconds != clock_seconds) {force_display_update_now = 1;}
        last_clock_seconds = clock_seconds;
      }
      #endif //defined(OPTION_DISPLAY_HHMMSS_CLOCK) && defined(FEATURE_CLOCK)

      // OPTION_DISPLAY_HHMM_CLOCK **************************************************************************************************
      #if defined(OPTION_DISPLAY_HHMM_CLOCK) && defined(FEATURE_CLOCK)
      if (!row_override[LCD_HHMM_CLOCK_ROW]){
        update_time();
        #ifdef OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        if (local_clock_hours < 10) {
          strcpy(workstring, "0");
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcat(workstring,workstring2);
        } else {
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcpy(workstring,workstring2);
        }
        #else
        dtostrf(local_clock_hours, 0, 0, workstring2);
        strcpy(workstring,workstring2);
        #endif //OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        strcat(workstring,":");
        if (local_clock_minutes < 10) {
          strcat(workstring, "0");
        }
        dtostrf(local_clock_minutes, 0, 0, workstring2);
        strcat(workstring,workstring2);
        if (LCD_HHMM_CLOCK_POSITION == LEFT){
          k3ngdisplay.print_left_fixed_field_size(workstring,LCD_HHMM_CLOCK_ROW-1,5);
        }
        if (LCD_HHMM_CLOCK_POSITION == RIGHT){
          k3ngdisplay.print_right_fixed_field_size(workstring,LCD_HHMM_CLOCK_ROW-1,5);
        }
        if (LCD_HHMM_CLOCK_POSITION == CENTER){
          k3ngdisplay.print_center_fixed_field_size(workstring,LCD_HHMM_CLOCK_ROW-1,5);
        }
      }
      #endif //defined(OPTION_DISPLAY_HHMM_CLOCK) && defined(FEATURE_CLOCK)

      // OPTION_DISPLAY_GPS_INDICATOR ********************************************************************
      #if defined(OPTION_DISPLAY_GPS_INDICATOR) && defined(FEATURE_GPS) && defined(FEATURE_CLOCK)
      if (((clock_status == GPS_SYNC) || (clock_status == SLAVE_SYNC_GPS)) && (!row_override[LCD_GPS_INDICATOR_ROW])){
        if (LCD_GPS_INDICATOR_POSITION == LEFT){
          k3ngdisplay.print_left_fixed_field_size(GPS_STRING,LCD_GPS_INDICATOR_ROW-1,3);
        }
        if (LCD_GPS_INDICATOR_POSITION == RIGHT){
          k3ngdisplay.print_right_fixed_field_size(GPS_STRING,LCD_GPS_INDICATOR_ROW-1,3);
        }
        if (LCD_GPS_INDICATOR_POSITION == CENTER){
          k3ngdisplay.print_center_fixed_field_size(GPS_STRING,LCD_GPS_INDICATOR_ROW-1,3);
        }
      }
      #endif //defined(OPTION_DISPLAY_GPS_INDICATOR) && defined(FEATURE_GPS)  && defined(FEATURE_CLOCK)


      // OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY *************************************************************
      #if defined(OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY) && defined(FEATURE_MOON_TRACKING)

      // static unsigned long last_moon_tracking_check_time = 0;

      if (!row_override[LCD_MOON_TRACKING_ROW]){
        if (((millis()-last_moon_tracking_check_time) > LCD_MOON_TRACKING_UPDATE_INTERVAL)) {
          update_moon_position();
          last_moon_tracking_check_time = millis();
        }
        strcpy(workstring,"");
        if (moon_tracking_active){
          if (moon_visible){
            strcat(workstring,TRACKING_ACTIVE_CHAR);
          } else {
            strcat(workstring,TRACKING_INACTIVE_CHAR);
          }
        }
        strcat(workstring,MOON_STRING);
        dtostrf(moon_azimuth,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((moon_azimuth < 100) || (abs(moon_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        strcat(workstring," ");
        dtostrf(moon_elevation,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((moon_azimuth < 100) || (abs(moon_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        if (moon_tracking_active){
          if (moon_visible){
            strcat(workstring,TRACKING_ACTIVE_CHAR);
          } else {
            strcat(workstring,TRACKING_INACTIVE_CHAR);
          }
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_MOON_TRACKING_ROW-1,LCD_COLUMNS);
      } else {
        #if defined(DEBUG_DISPLAY)
        debug.println(F("update_display: OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY row override"));
        #endif
      }
      #endif //defined(OPTION_DISPLAY_MOON_TRACKING_CONTINUOUSLY) && defined(FEATURE_MOON_TRACKING)

      // OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY **********************************************************
      #if defined(OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY) && defined(FEATURE_SUN_TRACKING)

      // static unsigned long last_sun_tracking_check_time = 0;

      if (!row_override[LCD_SUN_TRACKING_ROW]){
        if ((millis()-last_sun_tracking_check_time) > LCD_SUN_TRACKING_UPDATE_INTERVAL) {
          update_sun_position();
          last_sun_tracking_check_time = millis();
        }
        strcpy(workstring,"");
        if (sun_tracking_active){
          if (sun_visible){
            strcat(workstring,TRACKING_ACTIVE_CHAR);
          } else {
            strcat(workstring,TRACKING_INACTIVE_CHAR);
          }
        }
        strcat(workstring,SUN_STRING);
        dtostrf(sun_azimuth,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((sun_azimuth < 100) || (abs(sun_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        strcat(workstring," ");
        dtostrf(sun_elevation,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((sun_azimuth < 100) || (abs(sun_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        if (sun_tracking_active){
          if (sun_visible){
            strcat(workstring,TRACKING_ACTIVE_CHAR);
          } else {
            strcat(workstring,TRACKING_INACTIVE_CHAR);
          }
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_SUN_TRACKING_ROW-1,LCD_COLUMNS);
      } else {
        #if defined(DEBUG_DISPLAY)
        debug.println(F("update_display: OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY row override"));
        #endif
      }
      #endif //defined(OPTION_DISPLAY_SUN_TRACKING_CONTINUOUSLY) && defined(FEATURE_SUN_TRACKING)


      // OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD ****************************************************
      #if defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) && defined(FEATURE_CLOCK)

      static byte displaying_clock = 1;
      static unsigned long last_hhmm_clock_maidenhead_switch_time = 0;


      if (!row_override[LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW]){
        if ((millis()-last_hhmm_clock_maidenhead_switch_time) > 5000){
          if (displaying_clock){
            displaying_clock = 0;
          } else {
            displaying_clock = 1;
          }
          last_hhmm_clock_maidenhead_switch_time = millis();
        }
        if (displaying_clock){
          update_time();
          strcpy(workstring, "");
          #ifdef OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
          if (local_clock_hours < 10) {
            strcpy(workstring, "0");
            dtostrf(local_clock_hours, 0, 0, workstring2);
            strcat(workstring,workstring2);
          } else {
            dtostrf(local_clock_hours, 0, 0, workstring2);
            strcpy(workstring,workstring2);
          }
          #else
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcpy(workstring,workstring2);
          #endif //OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
          strcat(workstring,":");
          if (local_clock_minutes < 10) {
            strcat(workstring, "0");
          }
          dtostrf(local_clock_minutes, 0, 0, workstring2);
          strcat(workstring,workstring2);
          switch (LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_POSITION){
            case LEFT: k3ngdisplay.print_left_fixed_field_size(workstring,LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
            case RIGHT: k3ngdisplay.print_right_fixed_field_size(workstring,LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
            case CENTER: k3ngdisplay.print_center_fixed_field_size(workstring,LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
          }
        } else {
          switch (LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_POSITION){
            case LEFT: k3ngdisplay.print_left_fixed_field_size(coordinates_to_maidenhead(latitude,longitude),LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
            case RIGHT: k3ngdisplay.print_right_fixed_field_size(coordinates_to_maidenhead(latitude,longitude),LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
            case CENTER: k3ngdisplay.print_center_fixed_field_size(coordinates_to_maidenhead(latitude,longitude),LCD_ALT_HHMM_CLOCK_AND_MAIDENHEAD_ROW-1,6); break;
          }
        }
      }
      #endif //defined(OPTION_DISPLAY_ALT_HHMM_CLOCK_AND_MAIDENHEAD) && defined(FEATURE_CLOCK)

      // OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD **********************************************************************
      #if defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD) && defined(FEATURE_CLOCK)

      static int last_clock_seconds_clock_and_maidenhead = 0;

      if (!row_override[LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_ROW]){
        update_time();
        #ifdef OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        if (local_clock_hours < 10) {
          strcpy(workstring, "0");
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcat(workstring,workstring2);
        } else {
          dtostrf(local_clock_hours, 0, 0, workstring2);
          strcpy(workstring,workstring2);
        }
        #else
        dtostrf(local_clock_hours, 0, 0, workstring2);
        strcpy(workstring,workstring2);
        #endif //OPTION_CLOCK_ALWAYS_HAVE_HOUR_LEADING_ZERO
        strcat(workstring,":");
        if (local_clock_minutes < 10) {
          strcat(workstring, "0");
        }
        dtostrf(local_clock_minutes, 0, 0, workstring2);
        strcat(workstring,workstring2);
        strcat(workstring,":");
        if (local_clock_seconds < 10) {
          strcat(workstring, "0");
        }
        dtostrf(local_clock_seconds, 0, 0, workstring2);
        strcat(workstring,workstring2);
        strcat(workstring," ");
        strcat(workstring,coordinates_to_maidenhead(latitude,longitude));
        switch(LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_POSITION){
          case LEFT: k3ngdisplay.print_left_fixed_field_size(workstring,LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_ROW-1,LCD_COLUMNS); break;
          case RIGHT: k3ngdisplay.print_right_fixed_field_size(workstring,LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_ROW-1,LCD_COLUMNS); break;
          case CENTER: k3ngdisplay.print_center_fixed_field_size(workstring,LCD_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD_ROW-1,LCD_COLUMNS); break;
        }
        if (last_clock_seconds_clock_and_maidenhead != local_clock_seconds) {force_display_update_now = 1;}
        last_clock_seconds_clock_and_maidenhead = local_clock_seconds;
      }

      #endif //defined(OPTION_DISPLAY_CONSTANT_HHMMSS_CLOCK_AND_MAIDENHEAD) && defined(FEATURE_CLOCK)



      // OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL *******************************************************
      #ifdef OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL

      //  moon tracking ----
      #ifdef FEATURE_MOON_TRACKING

      // static unsigned long last_moon_tracking_check_time = 0;

      if ((!row_override[LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW])  && (moon_tracking_active)) {
        if (((millis()-last_moon_tracking_check_time) > LCD_MOON_TRACKING_UPDATE_INTERVAL)) {
          update_moon_position();
          last_moon_tracking_check_time = millis();
        }
        if (moon_visible){
          strcpy(workstring,TRACKING_ACTIVE_CHAR);
        } else {
          strcpy(workstring,TRACKING_INACTIVE_CHAR);
        }
        strcat(workstring,MOON_STRING);
        dtostrf(moon_azimuth,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((moon_azimuth < 100) || (abs(moon_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        strcat(workstring," ");
        dtostrf(moon_elevation,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((moon_azimuth < 100) || (abs(moon_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        if (moon_visible){
          strcat(workstring,TRACKING_ACTIVE_CHAR);
        } else {
          strcat(workstring,TRACKING_INACTIVE_CHAR);
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW-1,LCD_COLUMNS);
      }
      #endif //FEATURE_MOON_TRACKING


      //  sun tracking ----
      #ifdef FEATURE_SUN_TRACKING
      // static unsigned long last_sun_tracking_check_time = 0;

      if ((!row_override[LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW]) && (sun_tracking_active)){
        if ((millis()-last_sun_tracking_check_time) > LCD_SUN_TRACKING_UPDATE_INTERVAL) {
          update_sun_position();
          last_sun_tracking_check_time = millis();
        }
        if (sun_visible){
          strcpy(workstring,TRACKING_ACTIVE_CHAR);
        } else {
          strcpy(workstring,TRACKING_INACTIVE_CHAR);
        }
        strcat(workstring,SUN_STRING);
        dtostrf(sun_azimuth,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((sun_azimuth < 100) || (abs(sun_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        strcat(workstring," ");
        dtostrf(sun_elevation,0,LCD_DECIMAL_PLACES,workstring2);
        strcat(workstring,workstring2);
        if ((LCD_COLUMNS>16) && ((sun_azimuth < 100) || (abs(sun_elevation)<100))) {strcat(workstring,DISPLAY_DEGREES_STRING);}
        if (sun_visible){
          strcat(workstring,TRACKING_ACTIVE_CHAR);
        } else {
          strcat(workstring,TRACKING_INACTIVE_CHAR);
        }
        k3ngdisplay.print_center_fixed_field_size(workstring,LCD_MOON_OR_SUN_TRACKING_CONDITIONAL_ROW-1,LCD_COLUMNS);
      }

      #endif //FEATURE_SUN_TRACKING

      #endif //OPTION_DISPLAY_MOON_OR_SUN_TRACKING_CONDITIONAL


      // OPTION_DISPLAY_BIG_CLOCK **********************************************************
      #if defined(OPTION_DISPLAY_BIG_CLOCK) && defined(FEATURE_CLOCK)

      static byte big_clock_last_clock_seconds = 0;

      if (!row_override[LCD_BIG_CLOCK_ROW]){
        update_time();
        k3ngdisplay.print_center_entire_row(timezone_modified_clock_string(),LCD_BIG_CLOCK_ROW-1,0);
        if (big_clock_last_clock_seconds != clock_seconds) {
          force_display_update_now = 1;
          big_clock_last_clock_seconds = clock_seconds;
        }
      }
      #endif //defined(OPTION_DISPLAY_BIG_CLOCK) && defined(FEATURE_CLOCK)



      // TODO: develop status row with HH:MM time, rotation status, direction, and GPS status?

      // TODO: FEATURE_AZ_PRESET_ENCODER and FEATURE_EL_PRESET_ENCODER in status widget {done, need to test}


      //zzzzzz

      static unsigned long last_full_screen_redraw = 0;

      if ((millis() - last_full_screen_redraw) > 59999){
        k3ngdisplay.clear();
        k3ngdisplay.redraw();
        last_full_screen_redraw = millis();
      } else {


        // do it ! ************************************
        k3ngdisplay.service(force_display_update_now);
        //force_display_update_now = 0;

      }

    }
    #endif // defined(FEATURE_LCD_DISPLAY)



    #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)
    void get_keystroke(){
      while (control_port->available() == 0) {
      }
      while (control_port->available() > 0)
      incoming_serial_byte = control_port->read();
    }
    #endif // defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)


    #ifdef FEATURE_YAESU_EMULATION
    void print_wrote_to_memory(){

      control_port->println(F("Wrote to memory"));

    }

    #endif // FEATURE_YAESU_EMULATION

    #ifdef FEATURE_YAESU_EMULATION
    void clear_serial_buffer(){

      delay(200);
      while (control_port->available()) incoming_serial_byte = control_port->read();
    }

    #endif // FEATURE_YAESU_EMULATION

    void read_settings_from_eeprom(){

      byte * p = (byte *)(void *)&configuration;
      unsigned int i;
      int ee = 0;

      for (i = 0; i < sizeof(configuration); i++) {
        *p++ = EEPROM.read(ee++);
      }

      if (configuration.magic_number == EEPROM_MAGIC_NUMBER) {
        #ifdef DEBUG_EEPROM
        if (debug_mode) {
          debug.println("read_settings_from_eeprom: reading settings from eeprom: ");
          debug.print("\nanalog_az_full_ccw");
          debug.print(configuration.analog_az_full_ccw);
          debug.print("\nanalog_az_full_cw");
          debug.print(configuration.analog_az_full_cw);
          debug.print("\nanalog_el_0_degrees");
          debug.print(configuration.analog_el_0_degrees);
          debug.print("\nanalog_el_max_elevation");
          debug.print(configuration.analog_el_max_elevation);
          debug.print("\nlast_azimuth:");
          debug.print(configuration.last_azimuth, 1);
          debug.print("\nlast_elevation:");
          debug.print(configuration.last_elevation, 1);
          debug.print("\nlast_az_incremental_encoder_position:");
          debug.print(configuration.last_az_incremental_encoder_position);
          debug.print("\nlast_el_incremental_encoder_position:");
          debug.print(configuration.last_el_incremental_encoder_position);
          debug.print("\naz_offset:");
          debug.print(configuration.azimuth_offset,2);
          debug.print("\nel_offset:");
          debug.print(configuration.elevation_offset,2);
          debug.print("az starting point:");
          debug.print(configuration.azimuth_starting_point);
          debug.print("az rotation capability:");
          debug.print(configuration.azimuth_rotation_capability);
          debug.print("autopark_active:");
          debug.print(configuration.autopark_active);
          debug.print("autopark_time_minutes:");
          debug.print(configuration.autopark_time_minutes);
          debug.print("azimuth_display_mode:");
          debug.print(configuration.azimuth_display_mode);
          debug.println("");
        }
        #endif // DEBUG_EEPROM

        azimuth_starting_point = configuration.azimuth_starting_point;
        azimuth_rotation_capability = configuration.azimuth_rotation_capability;

        #if defined(FEATURE_AZ_POSITION_INCREMENTAL_ENCODER)
        az_incremental_encoder_position = configuration.last_az_incremental_encoder_position;
        #endif

        #if defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER)
        el_incremental_encoder_position = configuration.last_el_incremental_encoder_position;
        #endif


        #if defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
        raw_azimuth = int(configuration.last_azimuth * HEADING_MULTIPLIER);
        if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) {
          azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
        } else {
          azimuth = raw_azimuth;
        }
        #endif

        #if defined(FEATURE_ELEVATION_CONTROL) && (defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY))
        elevation = int(configuration.last_elevation * HEADING_MULTIPLIER);
        #endif



        #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
        raw_azimuth = int(configuration.last_azimuth * HEADING_MULTIPLIER);
        if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) {
          azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
        } else {
          azimuth = raw_azimuth;
        }
        az_position_pulse_input_azimuth = configuration.last_azimuth;
        #endif // FEATURE_AZ_POSITION_PULSE_INPUT

        #if defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_POSITION_PULSE_INPUT)
        elevation = int(configuration.last_elevation * HEADING_MULTIPLIER);
        el_position_pulse_input_elevation = configuration.last_elevation;
        #endif // FEATURE_EL_POSITION_PULSE_INPUT

        #if defined(FEATURE_AZ_POSITION_PULSE_INPUT) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
        configuration.azimuth_offset = 0;
        #endif

        #if defined(FEATURE_EL_POSITION_PULSE_INPUT) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER_USE_PJRC_LIBRARY)
        configuration.elevation_offset = 0;
        #endif



      } else {  // initialize eeprom with default values
        #ifdef DEBUG_EEPROM
        debug.println("read_settings_from_eeprom: uninitialized eeprom, calling initialize_eeprom_with_defaults()");
        #endif // DEBUG_EEPROM
        initialize_eeprom_with_defaults();
      }
    } // read_settings_from_eeprom

    void initialize_eeprom_with_defaults() {

      #ifdef DEBUG_LOOP
      debug.print("initialize_eeprom_with_defaults()\n");
      Serial.flush();
      #endif // DEBUG_LOOP

      #ifdef DEBUG_EEPROM
      debug.println("initialize_eeprom_with_defaults: writing eeprom");
      #endif // DEBUG_EEPROM

      configuration.analog_az_full_ccw = ANALOG_AZ_FULL_CCW;
      configuration.analog_az_full_cw = ANALOG_AZ_FULL_CW;
      configuration.analog_el_0_degrees = ANALOG_EL_0_DEGREES;
      configuration.analog_el_max_elevation = ANALOG_EL_MAX_ELEVATION;
      configuration.last_azimuth = raw_azimuth;
      configuration.last_az_incremental_encoder_position = 0;
      configuration.last_el_incremental_encoder_position = 0;
      configuration.azimuth_offset = 0;
      configuration.elevation_offset = 0;
      configuration.azimuth_starting_point = AZIMUTH_STARTING_POINT_DEFAULT;
      configuration.azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;
      configuration.brake_az_disabled = 0; //(brake_az ? 1 : 0);
      configuration.clock_timezone_offset = 0;
      configuration.autopark_active = 0;
      configuration.autopark_time_minutes = 0;
      configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;

      #ifdef FEATURE_ELEVATION_CONTROL
      configuration.last_elevation = elevation;
      #else
      configuration.last_elevation = 0;
      #endif

      #ifdef FEATURE_STEPPER_MOTOR
      configuration.az_stepper_motor_last_direction = STEPPER_UNDEF;
      configuration.az_stepper_motor_last_pin_state = LOW;
      configuration.el_stepper_motor_last_direction = STEPPER_UNDEF;
      configuration.el_stepper_motor_last_pin_state = LOW;
      #endif //FEATURE_STEPPER_MOTOR

      write_settings_to_eeprom();

    } // initialize_eeprom_with_defaults

    void write_settings_to_eeprom() {

      #ifdef DEBUG_EEPROM
      debug.println("write_settings_to_eeprom: writing settings to eeprom");
      #endif // DEBUG_EEPROM

      configuration.magic_number = EEPROM_MAGIC_NUMBER;

      const byte * p = (const byte *)(const void *)&configuration;
      unsigned int i;
      int ee = 0;
      for (i = 0; i < sizeof(configuration); i++) {
        EEPROM.write(ee++, *p++);
      }

      configuration_dirty = 0;

    } // write_settings_to_eeprom

    void az_check_operation_timeout() {

      // check if the last executed rotation operation has been going on too long

      if (((millis() - az_last_rotate_initiation) > OPERATION_TIMEOUT) && (az_state != IDLE)) {
        submit_request(AZ, REQUEST_KILL, 0, 78);
        #ifdef DEBUG_AZ_CHECK_OPERATION_TIMEOUT
        debug.println("az_check_operation_timeout: timeout reached, aborting rotation");
        #endif // DEBUG_AZ_CHECK_OPERATION_TIMEOUT
      }
    } // az_check_operation_timeout

    void read_azimuth(byte force_read) {

      unsigned int previous_raw_azimuth = raw_azimuth;
      static unsigned long last_measurement_time = 0;

      if (heading_reading_inhibit_pin) {
        if (digitalReadEnhanced(heading_reading_inhibit_pin)) {
          return;
        }
      }

      #ifdef DEBUG_HEADING_READING_TIME
      static unsigned long last_time = 0;
      static unsigned long last_print_time = 0;
      static float average_read_time = 0;
      #endif // DEBUG_HEADING_READING_TIME

      if (((millis() - last_measurement_time) > AZIMUTH_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

        #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
        analog_az = analogReadEnhanced(rotator_analog_az);
        raw_azimuth = map(analog_az, configuration.analog_az_full_ccw, configuration.analog_az_full_cw, (azimuth_starting_point * HEADING_MULTIPLIER), ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER));

        #ifdef FEATURE_AZIMUTH_CORRECTION
        raw_azimuth = (correct_azimuth(raw_azimuth / (float) HEADING_MULTIPLIER) * HEADING_MULTIPLIER);
        #endif // FEATURE_AZIMUTH_CORRECTION

        raw_azimuth = raw_azimuth + (configuration.azimuth_offset * HEADING_MULTIPLIER);

        if (AZIMUTH_SMOOTHING_FACTOR > 0) {
          raw_azimuth = (raw_azimuth * (1 - (AZIMUTH_SMOOTHING_FACTOR / 100.))) + (previous_raw_azimuth * (AZIMUTH_SMOOTHING_FACTOR / 100.));
        }

        if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) {

          azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
          if (azimuth >= (360 * HEADING_MULTIPLIER)) {
            azimuth = azimuth - (360 * HEADING_MULTIPLIER);
          }

        } else {

          if (raw_azimuth < 0) {
            azimuth = raw_azimuth + (360 * HEADING_MULTIPLIER);
          } else {
            azimuth = raw_azimuth;
          }

        }
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER

        last_measurement_time = millis();
      }
    } // read_azimuth

    void output_debug() {

      #ifdef DEBUG_DUMP
      char tempstring[32] = "";
      #if defined(CONTROL_PROTOCOL_EMULATION)

      if (((millis() - last_debug_output_time) >= 3000) && (debug_mode)) {

        #if defined(DEBUG_GPS_SERIAL)
        debug.println("");
        #endif //DEBUG_GPS_SERIAL

        //port_flush();

        debug.print("debug: \t");
        debug.print(CODE_VERSION);
        debug.print("\t\t");

        #ifdef FEATURE_CLOCK
        update_time();
        if (configuration.clock_timezone_offset != 0){
          sprintf(tempstring, "%s", timezone_modified_clock_string());
          debug.print(tempstring);
          debug.print("UTC");
          if (configuration.clock_timezone_offset > 0){
            debug.print("+");
          }
          if (configuration.clock_timezone_offset == int(configuration.clock_timezone_offset)){
            debug.print(int(configuration.clock_timezone_offset));
          } else {

            debug.print(configuration.clock_timezone_offset);
          }
          debug.print("\t");
          sprintf(tempstring, "%s", zulu_clock_string());
          debug.print(tempstring);
        } else {
          sprintf(tempstring, "%s", zulu_clock_string());
          debug.print(tempstring);
        }
        #else // FEATURE_CLOCK
        dtostrf((millis() / 1000),0,0,tempstring);
        debug.print(tempstring);
        #endif // FEATURE_CLOCK

        #if defined(FEATURE_GPS) || defined(FEATURE_RTC) || (defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE))
        debug.print("\t");
        debug.print(clock_status_string());
        #endif // defined(FEATURE_GPS) || defined(FEATURE_RTC) || (defined(FEATURE_CLOCK) && defined(OPTION_SYNC_MASTER_CLOCK_TO_SLAVE))

        #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
        debug.print("\t");
        sprintf(tempstring, "%s", coordinate_string());
        debug.print(tempstring);
        debug.print(" ");
        debug.print(coordinates_to_maidenhead(latitude,longitude));
        #endif

        debug.print("\t\t");

        #ifdef FEATURE_YAESU_EMULATION
        debug.print("GS-232");
        #ifdef OPTION_GS_232B_EMULATION
        debug.print("B");
        #else
        debug.print("A");
        #endif
        #endif // FEATURE_YAESU_EMULATION

        debug.println("");

        debug.print("\tAZ:");
        switch (az_state) {
          case IDLE: debug.print("IDLE"); break;
          case SLOW_START_CW: debug.print("SLOW_START_CW"); break;
          case SLOW_START_CCW: debug.print("SLOW_START_CCW"); break;
          case NORMAL_CW: debug.print("NORMAL_CW"); break;
          case NORMAL_CCW: debug.print("NORMAL_CCW"); break;
          case SLOW_DOWN_CW: debug.print("SLOW_DOWN_CW"); break;
          case SLOW_DOWN_CCW: debug.print("SLOW_DOWN_CCW"); break;
          case INITIALIZE_SLOW_START_CW: debug.print("INITIALIZE_SLOW_START_CW"); break;
          case INITIALIZE_SLOW_START_CCW: debug.print("INITIALIZE_SLOW_START_CCW"); break;
          case INITIALIZE_TIMED_SLOW_DOWN_CW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CW"); break;
          case INITIALIZE_TIMED_SLOW_DOWN_CCW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CCW"); break;
          case TIMED_SLOW_DOWN_CW: debug.print("TIMED_SLOW_DOWN_CW"); break;
          case TIMED_SLOW_DOWN_CCW: debug.print("TIMED_SLOW_DOWN_CCW"); break;
          case INITIALIZE_DIR_CHANGE_TO_CW: debug.print("INITIALIZE_DIR_CHANGE_TO_CW"); break;
          case INITIALIZE_DIR_CHANGE_TO_CCW: debug.print("INITIALIZE_DIR_CHANGE_TO_CCW"); break;
          case INITIALIZE_NORMAL_CW: debug.print("INITIALIZE_NORMAL_CW"); break;
          case INITIALIZE_NORMAL_CCW: debug.print("INITIALIZE_NORMAL_CCW"); break;
        }

        debug.print("  Q:");
        switch (az_request_queue_state) {
          case NONE: debug.print("-"); break;
          case IN_QUEUE: debug.print("IN_QUEUE"); break;
          case IN_PROGRESS_TIMED: debug.print("IN_PROGRESS_TIMED"); break;
          case IN_PROGRESS_TO_TARGET: debug.print("IN_PROGRESS_TO_TARGET"); break;
        }

        debug.print("  AZ:");
        debug.print((azimuth / LCD_HEADING_MULTIPLIER), LCD_DECIMAL_PLACES);
        debug.print("  AZ_raw:");
        debug.print((raw_azimuth / LCD_HEADING_MULTIPLIER), LCD_DECIMAL_PLACES);

        if (az_state != IDLE) {
          debug.print("  Target:");
          debug.print((target_azimuth / LCD_HEADING_MULTIPLIER), LCD_DECIMAL_PLACES);
          debug.print("  Target_raw: ");
          debug.print((target_raw_azimuth / LCD_HEADING_MULTIPLIER), LCD_DECIMAL_PLACES);
          debug.print("  Secs_left:");
          debug.print((OPERATION_TIMEOUT - (millis() - az_last_rotate_initiation)) / 1000);
        }

        #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
        debug.print("  Analog:");
        dtostrf(analog_az,0,0,tempstring);
        debug.print(tempstring);
        debug.print("  Range:");
        dtostrf(configuration.analog_az_full_ccw,0,0,tempstring);
        debug.print(tempstring);
        debug.print("-");
        dtostrf(configuration.analog_az_full_cw,0,0,tempstring);
        debug.print(tempstring);
        #endif // FEATURE_AZ_POSITION_POTENTIOMETER

        debug.print(F("  Start:"));
        debug.print(azimuth_starting_point);
        debug.print(F("  Rotation_Capability:"));
        debug.print(azimuth_rotation_capability,0);
        debug.print(F("  Raw_Az_Range:"));
        debug.print(azimuth_starting_point);
        debug.print("-");
        debug.print((azimuth_starting_point+azimuth_rotation_capability),0);
        debug.print("  AZ_Speed_Norm:");
        debug.print(normal_az_speed_voltage);
        debug.print("  Current:");
        debug.print(current_az_speed_voltage);
        if (az_speed_pot) {
          debug.print("  AZ_Speed_Pot:");
          debug.print(analogReadEnhanced(az_speed_pot));
        }
        if (az_preset_pot) {
          debug.print(F("  AZ_Preset_Pot_Analog:"));
          debug.print(analogReadEnhanced(az_preset_pot));
          debug.print(F("  AZ_Preset_Pot_Setting: "));
          dtostrf((map(analogReadEnhanced(az_preset_pot), AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP)),0,0,tempstring);
          debug.print(tempstring);
        }
        debug.print("  Offset:");
        dtostrf(configuration.azimuth_offset,0,2,tempstring);
        debug.print(tempstring);
        debug.println("");

        #ifdef FEATURE_ELEVATION_CONTROL
        debug.print("\tEL:");
        switch (el_state) {
          case IDLE: debug.print("IDLE"); break;
          case SLOW_START_UP: debug.print("SLOW_START_UP"); break;
          case SLOW_START_DOWN: debug.print("SLOW_START_DOWN"); break;
          case NORMAL_UP: debug.print("NORMAL_UP"); break;
          case NORMAL_DOWN: debug.print("NORMAL_DOWN"); break;
          case SLOW_DOWN_DOWN: debug.print("SLOW_DOWN_DOWN"); break;
          case SLOW_DOWN_UP: debug.print("SLOW_DOWN_UP"); break;
          case TIMED_SLOW_DOWN_UP: debug.print("TIMED_SLOW_DOWN_UP"); break;
          case TIMED_SLOW_DOWN_DOWN: debug.print("TIMED_SLOW_DOWN_DOWN"); break;
        }

        debug.print("  Q:");
        switch (el_request_queue_state) {
          case NONE: debug.print("-"); break;
          case IN_QUEUE: debug.print("IN_QUEUE"); break;
          case IN_PROGRESS_TIMED: debug.print("IN_PROGRESS_TIMED"); break;
          case IN_PROGRESS_TO_TARGET: debug.print("IN_PROGRESS_TO_TARGET"); break;
        }
        debug.print("  EL:");
        dtostrf(elevation / LCD_HEADING_MULTIPLIER, 0, LCD_DECIMAL_PLACES,tempstring);
        debug.print(tempstring);
        if (el_state != IDLE) {
          debug.print("  Target:");
          dtostrf(target_elevation / LCD_HEADING_MULTIPLIER, 0, LCD_DECIMAL_PLACES,tempstring);
          debug.print(tempstring);
        }

        #ifdef FEATURE_EL_POSITION_POTENTIOMETER
        debug.print("  EL_Analog:");
        dtostrf(analog_el,0,0,tempstring);
        debug.print(tempstring);
        debug.print("  Range:");
        dtostrf(configuration.analog_el_0_degrees,0,0,tempstring);
        debug.print(tempstring);
        debug.print("-");
        dtostrf(configuration.analog_el_max_elevation,0,0,tempstring);
        debug.print(tempstring);
        #endif // FEATURE_EL_POSITION_POTENTIOMETER

        debug.println("");
        #endif // FEATURE_ELEVATION_CONTROL

        //port_flush();

        #ifdef FEATURE_TIMED_BUFFER
        if (timed_buffer_status != EMPTY) {
          debug.print("  Timed_interval_buff:");
          switch (timed_buffer_status) {
            // case EMPTY: debug.print("EMPTY"); break;
            case LOADED_AZIMUTHS: debug.print("LOADED_AZIMUTHS"); break;
            case RUNNING_AZIMUTHS: debug.print("RUNNING_AZIMUTHS"); break;
            #ifdef FEATURE_ELEVATION_CONTROL
            case LOADED_AZIMUTHS_ELEVATIONS: debug.print("LOADED_AZIMUTHS_ELEVATIONS"); break;
            case RUNNING_AZIMUTHS_ELEVATIONS: debug.print("RUNNING_AZIMUTHS_ELEVATIONS"); break;
            #endif
          }

          debug.print("  Interval_secs:");
          debug.print(timed_buffer_interval_value_seconds);
          debug.print("  Entries:");
          debug.print(timed_buffer_number_entries_loaded);
          debug.print("  Entry_ptr:");
          debug.print(timed_buffer_entry_pointer);
          debug.print("  Secs_since_last_action:");
          debug.print((millis() - last_timed_buffer_action_time) / 1000);

          if (timed_buffer_number_entries_loaded > 0) {
            for (int x = 0; x < timed_buffer_number_entries_loaded; x++) {
              debug.print(x + 1);
              debug.print("\t:");
              debug.print(timed_buffer_azimuths[x] / HEADING_MULTIPLIER);
              #ifdef FEATURE_ELEVATION_CONTROL
              debug.print("\t- ");
              debug.print(timed_buffer_elevations[x] / HEADING_MULTIPLIER);
              #endif
              debug.print("\n");
            }
            debug.println("");
          }

        } // if (timed_buffer_status != EMPTY)
        #endif // FEATURE_TIMED_BUFFER


        #ifdef FEATURE_MOON_TRACKING
        update_moon_position();
        debug.print(moon_status_string());
        #endif // FEATURE_MOON_TRACKING

        #ifdef FEATURE_SUN_TRACKING
        update_sun_position();
        debug.print(sun_status_string());
        #endif // FEATURE_SUN_TRACKING

        #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)
        debug.println("");
        #endif //defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

        #ifdef FEATURE_GPS
        unsigned long gps_chars = 0;
        unsigned short gps_good_sentences = 0;
        unsigned short gps_failed_checksum = 0;
        char gps_temp_string[12] = "";
        float gps_lat_temp = 0;
        float gps_long_temp = 0;

        debug.print("\tGPS: satellites:");
        gps_chars = gps.satellites();
        //if (gps_chars == 255){gps_chars = 0;}
        dtostrf(gps_chars,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        unsigned long gps_fix_age_temp = 0;
        gps.f_get_position(&gps_lat_temp,&gps_long_temp,&gps_fix_age_temp);
        debug.print("  lat:");
        debug.print(gps_lat_temp,4);
        debug.print("  long:");
        debug.print(gps_long_temp,4);
        debug.print("  altitude(m):");
        debug.print(gps.altitude()/100,0);
        debug.print("  fix_age_mS:");
        dtostrf(gps_fix_age_temp,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        gps.stats(&gps_chars,&gps_good_sentences,&gps_failed_checksum);
        debug.print("  data_chars:");
        dtostrf(gps_chars,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.print("  good_sentences:");
        dtostrf(gps_good_sentences,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.print("  failed_checksum:");
        dtostrf(gps_failed_checksum,0,0,gps_temp_string);
        debug.print(gps_temp_string);
        debug.println("");
        #endif //FEATURE_GPS


        #ifdef FEATURE_AUTOCORRECT
        debug.print("\t\tAutocorrect: AZ:");
        switch(autocorrect_state_az){
          case AUTOCORRECT_INACTIVE: debug.print("INACTIVE"); break;
          case AUTOCORRECT_WAITING_AZ: debug.print("AUTOCORRECT_WAITING_AZ: "); debug.print(autocorrect_az,2); break;
          case AUTOCORRECT_WATCHING_AZ: debug.print("AUTOCORRECT_WATCHING_AZ: "); debug.print(autocorrect_az,2); break;
        }

        #ifdef FEATURE_ELEVATION_CONTROL
        debug.print(" EL:");
        switch(autocorrect_state_el){
          case AUTOCORRECT_INACTIVE: debug.print("INACTIVE"); break;
          case AUTOCORRECT_WAITING_EL: debug.print("AUTOCORRECT_WAITING_EL: "); debug.print(autocorrect_el,2); break;
          case AUTOCORRECT_WATCHING_EL: debug.print("AUTOCORRECT_WATCHING_EL: "); debug.print(autocorrect_el,2); break;
        }
        #endif //FEATURE_ELEVATION_CONTROL
        #endif //DEBUG_AUTOCORRECT

        if ((raw_azimuth / LCD_HEADING_MULTIPLIER) < azimuth_starting_point){
          debug.print(F("\tWARNING: raw azimuth is CCW of configured starting point of "));
          debug.print(azimuth_starting_point);
          debug.println("!");
        }

        if ((raw_azimuth / LCD_HEADING_MULTIPLIER) > (azimuth_starting_point+azimuth_rotation_capability)){
          debug.print(F("\tWARNING: raw azimuth is CW of configured ending point of "));
          debug.print((azimuth_starting_point+azimuth_rotation_capability),0);
          debug.println("!");
        }

        #if !defined(TEENSYDUINO)
        void * HP = malloc(4);
        if (HP) {free(HP);}
        unsigned long free = (unsigned long)SP - (unsigned long)HP;
        sprintf(tempstring,"%lu",(unsigned long)free);
        if ((free < 500) || (free > 10000)){
          debug.print(F("WARNING: Low memory: "));
          debug.print(tempstring);
          debug.println(F("b free"));
        }
        #endif
        debug.println("\n\n\n");
        last_debug_output_time = millis();
      }
      #endif // defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION)

      #endif //DEBUG_DUMP

    } // output_debug

    void print_to_port(char * print_this,byte port) {
      switch(port) {
        case CONTROL_PORT0: control_port->println(print_this);break;
      }
    } // print_to_port

    void print_help(byte port) {

      // The H command

      #if defined(OPTION_SERIAL_HELP_TEXT) && defined(FEATURE_YAESU_EMULATION)
      print_to_port("R Rotate Azimuth Clockwise\n",port);
      print_to_port("L Rotate Azimuth Counter Clockwise\n",port);
      print_to_port("A Stop\n",port);
      print_to_port("C Report Azimuth in Degrees\n",port);
      print_to_port("M### Rotate to ### degrees\n",port);
      print_to_port("MTTT XXX XXX XXX ... Timed Interval Direction Setting  (TTT = Step value in seconds, XXX = Azimuth in degrees)\n",port);
      print_to_port("T Start Timed Interval Tracking\n",port);
      print_to_port("N Report Total Number of M Timed Interval Azimuths\n",port);
      print_to_port("X1 Horizontal Rotation Low Speed\n",port);
      print_to_port("X2 Horizontal Rotation Middle 1 Speed\n",port);
      print_to_port("X3 Horizontal Rotation Middle 2 Speed\n",port);
      print_to_port("X4 Horizontal Rotation High Speed\n",port);
      print_to_port("S Stop\n",port);
      print_to_port("O Offset Calibration\n",port);
      print_to_port("F Full Scale Calibration\n",port);
      #ifdef FEATURE_ELEVATION_CONTROL
      print_to_port("U Rotate Elevation Up\n",port);
      print_to_port("D Rotate Elevation Down\n",port);
      print_to_port("E Stop Elevation Rotation\n",port);
      print_to_port("B Report Elevation in Degrees\n",port);
      print_to_port("Wxxx yyy Rotate Azimuth to xxx Degrees and Elevation to yyy Degrees\n",port);
      print_to_port("O2 Elevation Offset Calibration (0 degrees)\n",port);
      print_to_port("F2 Elevation Full Scale Calibration (180 degrees (or maximum))\n",port);
      #endif // FEATURE_ELEVATION_CONTROL
      #endif // defined(OPTION_SERIAL_HELP_TEXT) && defined(FEATURE_YAESU_EMULATION)


    } // print_help

    void el_check_operation_timeout() {

      // check if the last executed rotation operation has been going on too long

      if (((millis() - el_last_rotate_initiation) > OPERATION_TIMEOUT) && (el_state != IDLE)) {
        submit_request(EL, REQUEST_KILL, 0, 85);
        #ifdef DEBUG_EL_CHECK_OPERATION_TIMEOUT
        if (debug_mode) {
          debug.print(F("el_check_operation_timeout: timeout reached, aborting rotation\n"));
        }
        #endif // DEBUG_EL_CHECK_OPERATION_TIMEOUT
      }

    } // el_check_operation_timeout

    void read_elevation(byte force_read) {

      unsigned int previous_elevation = elevation;
      static unsigned long last_measurement_time = 0;

      if (heading_reading_inhibit_pin) {
        if (digitalReadEnhanced(heading_reading_inhibit_pin)) {
          return;
        }
      }

      #ifdef DEBUG_HEADING_READING_TIME
      static unsigned long last_time = 0;
      static unsigned long last_print_time = 0;
      static float average_read_time = 0;
      #endif // DEBUG_HEADING_READING_TIME

      if (((millis() - last_measurement_time) > ELEVATION_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

        #ifdef FEATURE_EL_POSITION_POTENTIOMETER
        analog_el = analogReadEnhanced(rotator_analog_el);
        elevation = (map(analog_el, configuration.analog_el_0_degrees, configuration.analog_el_max_elevation, 0, (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER)));
        #ifdef FEATURE_ELEVATION_CORRECTION
        elevation = (correct_elevation(elevation / (float) HEADING_MULTIPLIER) * HEADING_MULTIPLIER);
        #endif // FEATURE_ELEVATION_CORRECTION
        elevation = elevation + (configuration.elevation_offset * HEADING_MULTIPLIER);
        if (ELEVATION_SMOOTHING_FACTOR > 0) {
          elevation = (elevation * (1 - (ELEVATION_SMOOTHING_FACTOR / 100))) + (previous_elevation * (ELEVATION_SMOOTHING_FACTOR / 100));
        }
        if (elevation < 0) {
          elevation = 0;
        }
        #endif // FEATURE_EL_POSITION_POTENTIOMETER

        last_measurement_time = millis();

      }

    } // read_elevation

    void update_el_variable_outputs(byte speed_voltage) {

      #ifdef DEBUG_VARIABLE_OUTPUTS
      debug.print("update_el_variable_outputs: speed_voltage: ");
      debug.print(speed_voltage);
      #endif // DEBUG_VARIABLE_OUTPUTS

      if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_up_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_up_pwm, speed_voltage);
      }

      if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_down_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_down_pwm, speed_voltage);
      }

      if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN) ||
      (el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_down_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_up_down_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_up_down_pwm, speed_voltage);
      }

      if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (rotate_up_freq)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_up_freq");
        #endif // DEBUG_VARIABLE_OUTPUTS
        tone(rotate_up_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
      }

      if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (rotate_down_freq)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_down_freq");
        #endif // DEBUG_VARIABLE_OUTPUTS
        tone(rotate_down_freq, map(speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
      }

      if (elevation_speed_voltage) {
        analogWriteEnhanced(elevation_speed_voltage, speed_voltage);
      }

      #ifdef DEBUG_VARIABLE_OUTPUTS
      debug.println("");
      #endif // DEBUG_VARIABLE_OUTPUTS

      current_el_speed_voltage = speed_voltage;

    } // update_el_variable_outputs

    void update_az_variable_outputs(byte speed_voltage){

      #ifdef DEBUG_VARIABLE_OUTPUTS
      int temp_int = 0;

      debug.print("update_az_variable_outputs: az_state: ");
      switch (az_state) {
        case IDLE: debug.print("IDLE"); break;
        case SLOW_START_CW: debug.print("SLOW_START_CW"); break;
        case SLOW_START_CCW: debug.print("SLOW_START_CCW"); break;
        case NORMAL_CW: debug.print("NORMAL_CW"); break;
        case NORMAL_CCW: debug.print("NORMAL_CCW"); break;
        case SLOW_DOWN_CW: debug.print("SLOW_DOWN_CW"); break;
        case SLOW_DOWN_CCW: debug.print("SLOW_DOWN_CCW"); break;
        case INITIALIZE_SLOW_START_CW: debug.print("INITIALIZE_SLOW_START_CW"); break;
        case INITIALIZE_SLOW_START_CCW: debug.print("INITIALIZE_SLOW_START_CCW"); break;
        case INITIALIZE_TIMED_SLOW_DOWN_CW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CW"); break;
        case INITIALIZE_TIMED_SLOW_DOWN_CCW: debug.print("INITIALIZE_TIMED_SLOW_DOWN_CCW"); break;
        case TIMED_SLOW_DOWN_CW: debug.print("TIMED_SLOW_DOWN_CW"); break;
        case TIMED_SLOW_DOWN_CCW: debug.print("TIMED_SLOW_DOWN_CCW"); break;
        case INITIALIZE_DIR_CHANGE_TO_CW: debug.print("INITIALIZE_DIR_CHANGE_TO_CW"); break;
        case INITIALIZE_DIR_CHANGE_TO_CCW: debug.print("INITIALIZE_DIR_CHANGE_TO_CCW"); break;
        case INITIALIZE_NORMAL_CW: debug.print("INITIALIZE_NORMAL_CW"); break;
        case INITIALIZE_NORMAL_CCW: debug.print("INITIALIZE_NORMAL_CCW"); break;
        default: debug.print("UNDEF"); break;
      }
      debug.print(" speed_voltage: ");
      debug.print(speed_voltage);
      #endif // DEBUG_VARIABLE_OUTPUTS

      if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_cw_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_cw_pwm, speed_voltage);
      }

      if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_ccw_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_ccw_pwm, speed_voltage);
      }

      if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW) || (az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_cw_ccw_pwm)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_cw_ccw_pwm");
        #endif // DEBUG_VARIABLE_OUTPUTS
        analogWriteEnhanced(rotate_cw_ccw_pwm, speed_voltage);
      }

      if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (rotate_cw_freq)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_cw_freq: ");
        temp_int = map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH);
        tone(rotate_cw_freq, temp_int);
        debug.print(temp_int);
        #else // DEBUG_VARIABLE_OUTPUTS
        tone(rotate_cw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
        #endif // DEBUG_VARIABLE_OUTPUTS
      }

      if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (rotate_ccw_freq)) {
        #ifdef DEBUG_VARIABLE_OUTPUTS
        debug.print("\trotate_ccw_freq: ");
        temp_int = map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH);
        tone(rotate_ccw_freq, temp_int);
        debug.print(temp_int);
        #else // DEBUG_VARIABLE_OUTPUTS
        tone(rotate_ccw_freq, map(speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
        #endif // DEBUG_VARIABLE_OUTPUTS
      }

      if (azimuth_speed_voltage) {
        analogWriteEnhanced(azimuth_speed_voltage, speed_voltage);
      }

      #ifdef DEBUG_VARIABLE_OUTPUTS
      debug.println("");
      #endif // DEBUG_VARIABLE_OUTPUTS

      current_az_speed_voltage = speed_voltage;

    } // update_az_variable_outputs

    void rotator(byte rotation_action, byte rotation_type) {

      #ifdef DEBUG_ROTATOR
      if (debug_mode) {
        control_port->flush();
        debug.print(F("rotator: rotation_action:"));
        debug.print(rotation_action);
        debug.print(F(" rotation_type:"));
        control_port->flush();
        debug.print(rotation_type);
        debug.print(F("->"));
        control_port->flush();
        // delay(1000);
      }
      #endif // DEBUG_ROTATOR
      switch (rotation_type) {
        case CW:
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("CW ")); control_port->flush();
        }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          brake_release(AZ, BRAKE_RELEASE_ON);
          if (az_slowstart_active) {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            }
          } else {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, normal_az_speed_voltage);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_freq) {
              tone(rotate_cw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            }
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_ACTIVE_VALUE);
            #if defined(pin_led_cw)
            digitalWriteEnhanced(pin_led_cw, PIN_LED_ACTIVE_STATE);
            #endif
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_ccw)
            digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_ACTIVE_VALUE);
          }
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("rotator: normal_az_speed_voltage:"));
            control_port->println(normal_az_speed_voltage);
            //control_port->flush();
          }
          #endif // DEBUG_ROTATOR
        } else {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          if (rotate_cw_pwm) {
            analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
          }
          if (rotate_cw_ccw_pwm) {
            analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_cw)
            digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_INACTIVE_VALUE);
          }
          if (rotate_cw_freq) {
            noTone(rotate_cw_freq);
          }
        }
        break;
        case CCW:
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("CCW ")); control_port->flush();
        }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          brake_release(AZ, BRAKE_RELEASE_ON);
          if (az_slowstart_active) {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, 0);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, 0);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              noTone(rotate_ccw_freq);
            }
          } else {
            if (rotate_cw_pwm) {
              analogWriteEnhanced(rotate_cw_pwm, 0); digitalWriteEnhanced(rotate_cw_pwm, LOW);
            }
            if (rotate_ccw_pwm) {
              analogWriteEnhanced(rotate_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_ccw_pwm) {
              analogWriteEnhanced(rotate_cw_ccw_pwm, normal_az_speed_voltage);
            }
            if (rotate_cw_freq) {
              noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq) {
              tone(rotate_ccw_freq, map(normal_az_speed_voltage, 0, 255, AZ_VARIABLE_FREQ_OUTPUT_LOW, AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }
          }
          if (rotate_cw) {
            digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_cw)
            digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_ACTIVE_VALUE);
            #if defined(pin_led_ccw)
            digitalWriteEnhanced(pin_led_ccw, PIN_LED_ACTIVE_STATE);
            #endif
          }
          if (rotate_cw_ccw){
            digitalWriteEnhanced(rotate_cw_ccw, ROTATE_PIN_ACTIVE_VALUE);
          }
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("rotator: normal_az_speed_voltage:"));
            control_port->println(normal_az_speed_voltage);
            control_port->flush();
          }
          #endif // DEBUG_ROTATOR
        } else {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          if (rotate_ccw_pwm) {
            analogWriteEnhanced(rotate_ccw_pwm, 0); digitalWriteEnhanced(rotate_ccw_pwm, LOW);
          }
          if (rotate_ccw) {
            digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_ccw)
            digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_ccw_freq) {
            noTone(rotate_ccw_freq);
          }
        }
        break;
        case UP:
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("ROTATION_UP "));
        }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          brake_release(EL, BRAKE_RELEASE_ON);
          if (el_slowstart_active) {
            if (rotate_up_pwm) {
              analogWriteEnhanced(rotate_up_pwm, 0);
            }
            if (rotate_down_pwm) {
              analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, 0);
            }
            if (rotate_up_freq) {
              noTone(rotate_up_freq);
            }
            if (rotate_down_freq) {
              noTone(rotate_down_freq);
            }
          } else {
            if (rotate_up_pwm) {
              analogWriteEnhanced(rotate_up_pwm, normal_el_speed_voltage);
            }
            if (rotate_down_pwm) {
              analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
            }
            if (rotate_up_freq) {
              tone(rotate_up_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
            }
            if (rotate_down_freq) {
              noTone(rotate_down_freq);
            }
          }
          if (rotate_up) {
            digitalWriteEnhanced(rotate_up, ROTATE_PIN_ACTIVE_VALUE);
            #if defined(pin_led_up)
            digitalWriteEnhanced(pin_led_up, PIN_LED_ACTIVE_STATE);
            #endif
          }
          if (rotate_down) {
            digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_down)
            digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_up_or_down) {
            digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_ACTIVE_VALUE);
          }
        } else {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          if (rotate_up) {
            digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_up)
            digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_up_pwm) {
            analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, 0);
          }
          if (rotate_up_freq) {
            noTone(rotate_up_freq);
          }
          if (rotate_up_or_down) {
            digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_INACTIVE_VALUE);
          }
        }
        break;
        case DOWN:
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("ROTATION_DOWN "));
        }
        #endif // DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("ACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          brake_release(EL, BRAKE_RELEASE_ON);
          if (el_slowstart_active) {
            if (rotate_down_pwm) {
              analogWriteEnhanced(rotate_down_pwm, 0);
            }
            if (rotate_up_pwm) {
              analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, 0);
            }
            if (rotate_up_freq) {
              noTone(rotate_up_freq);
            }
            if (rotate_down_freq) {
              noTone(rotate_down_freq);
            }
          } else {
            if (rotate_down_pwm) {
              analogWriteEnhanced(rotate_down_pwm, normal_el_speed_voltage);
            }
            if (rotate_up_pwm) {
              analogWriteEnhanced(rotate_up_pwm, 0); digitalWriteEnhanced(rotate_up_pwm, LOW);
            }
            if (rotate_up_down_pwm) {
              analogWriteEnhanced(rotate_up_down_pwm, normal_el_speed_voltage);
            }
            if (rotate_down_freq) {
              tone(rotate_down_freq, map(normal_el_speed_voltage, 0, 255, EL_VARIABLE_FREQ_OUTPUT_LOW, EL_VARIABLE_FREQ_OUTPUT_HIGH));
            }
            if (rotate_up_freq) {
              noTone(rotate_up_freq);
            }
          }
          if (rotate_up) {
            digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_up)
            digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_down) {
            digitalWriteEnhanced(rotate_down, ROTATE_PIN_ACTIVE_VALUE);
            #if defined(pin_led_down)
            digitalWriteEnhanced(pin_led_down, PIN_LED_ACTIVE_STATE);
            #endif
          }
          if (rotate_up_or_down) {
            digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_ACTIVE_VALUE);
          }
        } else {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {
            debug.print(F("DEACTIVATE\n"));
          }
          #endif // DEBUG_ROTATOR
          if (rotate_down) {
            digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
            #if defined(pin_led_down)
            digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
            #endif
          }
          if (rotate_down_pwm) {
            analogWriteEnhanced(rotate_down_pwm, 0); digitalWriteEnhanced(rotate_down_pwm, LOW);
          }
          if (rotate_up_down_pwm) {
            analogWriteEnhanced(rotate_up_down_pwm, 0);
          }
          if (rotate_down_freq) {
            noTone(rotate_down_freq);
          }
          if (rotate_up_or_down) {
            digitalWriteEnhanced(rotate_up_or_down, ROTATE_PIN_INACTIVE_VALUE);
          }
        }
        break;
      }

      #ifdef DEBUG_ROTATOR
      if (debug_mode) {
        debug.print(F("rotator: exiting\n"));
        control_port->flush();
      }
      #endif // DEBUG_ROTATOR

    } // rotator

    void initialize_interrupts() {

      #ifdef DEBUG_LOOP
      debug.print("initialize_interrupts()\n");
      Serial.flush();
      #endif // DEBUG_LOOP
    } // initialize_interrupts

    void initialize_pins() {

      #ifdef DEBUG_LOOP
      debug.print("initialize_pins()\n");
      Serial.flush();
      #endif // DEBUG_LOOP

      #ifdef reset_pin
      pinMode(reset_pin, OUTPUT);
      digitalWrite(reset_pin, LOW);
      #endif //reset_pin

      if (serial_led) {
        pinModeEnhanced(serial_led, OUTPUT);
      }

      if (overlap_led) {
        pinModeEnhanced(overlap_led, OUTPUT);
      }

      if (brake_az) {
        pinModeEnhanced(brake_az, OUTPUT);
        digitalWriteEnhanced(brake_az, BRAKE_INACTIVE_STATE);
      }

      if (az_speed_pot) {
        pinModeEnhanced(az_speed_pot, INPUT);
        digitalWriteEnhanced(az_speed_pot, LOW);
      }

      if (az_preset_pot) {
        pinModeEnhanced(az_preset_pot, INPUT);
        digitalWriteEnhanced(az_preset_pot, LOW);
      }

      if (preset_start_button) {
        pinModeEnhanced(preset_start_button, INPUT);
        digitalWriteEnhanced(preset_start_button, HIGH);
      }

      if (button_stop) {
        pinModeEnhanced(button_stop, INPUT);
        digitalWriteEnhanced(button_stop, HIGH);
      }

      #ifdef FEATURE_ELEVATION_CONTROL
      if (brake_el) {
        pinModeEnhanced(brake_el, OUTPUT);
        digitalWriteEnhanced(brake_el, BRAKE_INACTIVE_STATE);
      }
      #endif // FEATURE_ELEVATION_CONTROL

      if (rotate_cw) {
        pinModeEnhanced(rotate_cw, OUTPUT);
      }
      if (rotate_ccw) {
        pinModeEnhanced(rotate_ccw, OUTPUT);
      }
      if (rotate_cw_pwm) {
        pinModeEnhanced(rotate_cw_pwm, OUTPUT);
      }
      if (rotate_ccw_pwm) {
        pinModeEnhanced(rotate_ccw_pwm, OUTPUT);
      }
      if (rotate_cw_ccw_pwm) {
        pinModeEnhanced(rotate_cw_ccw_pwm, OUTPUT);
      }
      if (rotate_cw_freq) {
        pinModeEnhanced(rotate_cw_freq, OUTPUT);
      }
      if (rotate_ccw_freq) {
        pinModeEnhanced(rotate_ccw_freq, OUTPUT);
      }

      if (rotate_cw_ccw) {
        pinModeEnhanced(rotate_cw_ccw, OUTPUT);
      }

      #if defined(pin_led_cw)
      pinModeEnhanced(pin_led_cw, OUTPUT);
      digitalWriteEnhanced(pin_led_cw, PIN_LED_INACTIVE_STATE);
      #endif

      #if defined(pin_led_ccw)
      pinModeEnhanced(pin_led_ccw, OUTPUT);
      digitalWriteEnhanced(pin_led_ccw, PIN_LED_INACTIVE_STATE);
      #endif

      #if defined(pin_led_up)
      pinModeEnhanced(pin_led_up, OUTPUT);
      digitalWriteEnhanced(pin_led_up, PIN_LED_INACTIVE_STATE);
      #endif

      #if defined(pin_led_down)
      pinModeEnhanced(pin_led_down, OUTPUT);
      digitalWriteEnhanced(pin_led_down, PIN_LED_INACTIVE_STATE);
      #endif

      rotator(DEACTIVATE, CW);
      rotator(DEACTIVATE, CCW);

      #if defined(FEATURE_AZ_POSITION_POTENTIOMETER)
      pinModeEnhanced(rotator_analog_az, INPUT);
      #endif

      if (button_cw) {
        pinModeEnhanced(button_cw, INPUT);
        digitalWriteEnhanced(button_cw, HIGH);
      }
      if (button_ccw) {
        pinModeEnhanced(button_ccw, INPUT);
        digitalWriteEnhanced(button_ccw, HIGH);
      }

      normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
      current_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;

      #ifdef FEATURE_ELEVATION_CONTROL
      normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
      current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
      #endif // FEATURE_ELEVATION_CONTROL

      if (azimuth_speed_voltage) {                 // if azimuth_speed_voltage pin is configured, set it up for PWM output
        analogWriteEnhanced(azimuth_speed_voltage, PWM_SPEED_VOLTAGE_X4);
      }


      #ifdef FEATURE_ELEVATION_CONTROL
      pinModeEnhanced(rotate_up, OUTPUT);
      pinModeEnhanced(rotate_down, OUTPUT);
      if (rotate_up_or_down) {
        pinModeEnhanced(rotate_up_or_down, OUTPUT);
      }
      if (rotate_up_pwm) {
        pinModeEnhanced(rotate_up_pwm, OUTPUT);
      }
      if (rotate_down_pwm) {
        pinModeEnhanced(rotate_down_pwm, OUTPUT);
      }
      if (rotate_up_down_pwm) {
        pinModeEnhanced(rotate_up_down_pwm, OUTPUT);
      }
      if (rotate_up_freq) {
        pinModeEnhanced(rotate_up_freq, OUTPUT);
      }
      if (rotate_down_freq) {
        pinModeEnhanced(rotate_down_freq, OUTPUT);
      }
      rotator(DEACTIVATE, UP);
      rotator(DEACTIVATE, DOWN);
      #ifdef FEATURE_EL_POSITION_POTENTIOMETER
      pinModeEnhanced(rotator_analog_el, INPUT);
      #endif // FEATURE_EL_POSITION_POTENTIOMETER
      if (button_up) {
        pinModeEnhanced(button_up, INPUT);
        digitalWriteEnhanced(button_up, HIGH);
      }
      if (button_down) {
        pinModeEnhanced(button_down, INPUT);
        digitalWriteEnhanced(button_down, HIGH);
      }

      if (elevation_speed_voltage) {                 // if elevation_speed_voltage pin is configured, set it up for PWM output
        analogWriteEnhanced(elevation_speed_voltage, PWM_SPEED_VOLTAGE_X4);
        normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
        current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
      }

      read_elevation(0);
      #endif // FEATURE_ELEVATION_CONTROL

      #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
      if (az_position_pulse_pin) {
        pinModeEnhanced(az_position_pulse_pin, INPUT);
        #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
        digitalWriteEnhanced(az_position_pulse_pin, HIGH);
        #endif // OPTION_POSITION_PULSE_INPUT_PULLUPS
      }
      #endif // FEATURE_AZ_POSITION_PULSE_INPUT


      #ifdef FEATURE_EL_POSITION_PULSE_INPUT
      if (el_position_pulse_pin) {
        pinModeEnhanced(el_position_pulse_pin, INPUT);
        #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
        digitalWriteEnhanced(el_position_pulse_pin, HIGH);
        #endif // OPTION_POSITION_PULSE_INPUT_PULLUPS
      }
      #endif // FEATURE_EL_POSITION_PULSE_INPUT

      #ifdef FEATURE_ROTATION_INDICATOR_PIN
      if (rotation_indication_pin) {
        pinModeEnhanced(rotation_indication_pin, OUTPUT);
        digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_INACTIVE_STATE);
      }
      #endif // FEATURE_ROTATION_INDICATOR_PIN


      if (blink_led) {
        pinModeEnhanced(blink_led, OUTPUT);
      }

      if (heading_reading_inhibit_pin) {
        pinModeEnhanced(heading_reading_inhibit_pin, INPUT);
      }

      #ifdef FEATURE_LIMIT_SENSE
      if (az_limit_sense_pin) {
        pinModeEnhanced(az_limit_sense_pin, INPUT);
        digitalWriteEnhanced(az_limit_sense_pin, HIGH);
      }
      #ifdef FEATURE_ELEVATION_CONTROL
      if (el_limit_sense_pin) {
        pinModeEnhanced(el_limit_sense_pin, INPUT);
        digitalWriteEnhanced(el_limit_sense_pin, HIGH);
      }
      #endif // FEATURE_ELEVATION_CONTROL
      #endif // FEATURE_LIMIT_SENSE

      #ifdef FEATURE_MOON_TRACKING
      if (moon_tracking_active_pin) {
        pinModeEnhanced(moon_tracking_active_pin, OUTPUT);
        digitalWriteEnhanced(moon_tracking_active_pin, LOW);
      }
      if (moon_tracking_activate_line) {
        pinModeEnhanced(moon_tracking_activate_line, INPUT);
        digitalWriteEnhanced(moon_tracking_activate_line, HIGH);
      }
      if (moon_tracking_button) {
        pinModeEnhanced(moon_tracking_button, INPUT);
        digitalWriteEnhanced(moon_tracking_button, HIGH);
      }
      #endif // FEATURE_MOON_TRACKING


      #ifdef FEATURE_SUN_TRACKING
      if (sun_tracking_active_pin) {
        pinModeEnhanced(sun_tracking_active_pin, OUTPUT);
        digitalWriteEnhanced(sun_tracking_active_pin, LOW);
      }
      if (sun_tracking_activate_line) {
        pinModeEnhanced(sun_tracking_activate_line, INPUT);
        digitalWriteEnhanced(sun_tracking_activate_line, HIGH);
      }
      if (sun_tracking_button) {
        pinModeEnhanced(sun_tracking_button, INPUT);
        digitalWriteEnhanced(sun_tracking_button, HIGH);
      }
      #endif // FEATURE_SUN_TRACKING


      #ifdef FEATURE_GPS
      if (gps_sync) {
        pinModeEnhanced(gps_sync, OUTPUT);
        digitalWriteEnhanced(gps_sync, LOW);
      }
      #endif //FEATURE_GPS

      #ifdef FEATURE_POWER_SWITCH
      pinModeEnhanced(power_switch, OUTPUT);
      digitalWriteEnhanced(power_switch, HIGH);
      #endif //FEATURE_POWER_SWITCH

      #ifdef FEATURE_STEPPER_MOTOR
      if (az_stepper_motor_pulse){
        pinModeEnhanced(az_stepper_motor_pulse, OUTPUT);
        digitalWriteEnhanced(az_stepper_motor_pulse, HIGH);
      }
      /*
      if (az_stepper_motor_direction){
      pinModeEnhanced(az_stepper_motor_direction, OUTPUT);
      digitalWriteEnhanced(az_stepper_motor_direction, configuration.az_stepper_motor_last_pin_state);
    }
    */

    #ifdef FEATURE_ELEVATION_CONTROL
    if (el_stepper_motor_pulse){
      pinModeEnhanced(el_stepper_motor_pulse, OUTPUT);
      digitalWriteEnhanced(el_stepper_motor_pulse, HIGH);
    }
    /*
    if (el_stepper_motor_direction){
    pinModeEnhanced(el_stepper_motor_direction, OUTPUT);
    digitalWriteEnhanced(el_stepper_motor_direction, configuration.el_stepper_motor_last_pin_state);
  }
  */
  #endif //FEATURE_ELEVATION_CONTROL
  #endif //FEATURE_STEPPER_MOTOR

  #ifdef FEATURE_EL_POSITION_MEMSIC_2125
  pinModeEnhanced(pin_memsic_2125_x, INPUT);
  pinModeEnhanced(pin_memsic_2125_y, INPUT);
  #endif //FEATURE_EL_POSITION_MEMSIC_2125

  #ifdef FEATURE_ANALOG_OUTPUT_PINS
  pinModeEnhanced(pin_analog_az_out, OUTPUT);
  digitalWriteEnhanced(pin_analog_az_out, LOW);
  #ifdef FEATURE_ELEVATION_CONTROL
  pinModeEnhanced(pin_analog_el_out, OUTPUT);
  digitalWriteEnhanced(pin_analog_el_out, LOW);
  #endif //FEATURE_ELEVATION_CONTROL
  #endif //FEATURE_ANALOG_OUTPUT_PINS

  #ifdef FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION
  pinModeEnhanced(pin_sun_pushbutton_calibration, INPUT);
  digitalWriteEnhanced(pin_sun_pushbutton_calibration, HIGH);
  #endif //FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION

  #ifdef FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION
  pinModeEnhanced(pin_moon_pushbutton_calibration, INPUT);
  digitalWriteEnhanced(pin_moon_pushbutton_calibration, HIGH);
  #endif //FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION



} /* initialize_pins */

void initialize_serial() {

  #if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION) || defined(FEATURE_CLOCK) || defined(UNDER_DEVELOPMENT_REMOTE_UNIT_COMMANDS)
  control_port = CONTROL_PORT_MAPPED_TO;
  control_port->begin(CONTROL_PORT_BAUD_RATE);
  #if defined(OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING)
  control_port->print OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING_STRING;
  #endif
  #endif

  #ifdef FEATURE_REMOTE_UNIT_SLAVE
  control_port->print(F("CS"));
  control_port->println(CODE_VERSION);
  #endif // FEATURE_REMOTE_UNIT_SLAVE

  #if defined(FEATURE_MASTER_WITH_SERIAL_SLAVE)
  remote_unit_port = REMOTE_PORT_MAPPED_TO;
  remote_unit_port->begin(REMOTE_UNIT_PORT_BAUD_RATE);
  #endif

  #ifdef FEATURE_GPS
  gps_port = GPS_PORT_MAPPED_TO;
  gps_port->begin(GPS_PORT_BAUD_RATE);
  #ifdef GPS_MIRROR_PORT
  gps_mirror_port = GPS_MIRROR_PORT;
  gps_mirror_port->begin(GPS_MIRROR_PORT_BAUD_RATE);
  #endif //GPS_MIRROR_PORT
  #endif //FEATURE_GPS

} /* initialize_serial */

void initialize_display() {

  #ifdef DEBUG_LOOP
  debug.print("initialize_display()\n");
  Serial.flush();
  #endif // DEBUG_LOOP

  k3ngdisplay.initialize();

  k3ngdisplay.print_center_timed_message("\x4B\x57\x34\x42\x51","\x52\x6F\x74\x6F\x72\x20\x43\x6F\x6E\x74\x72\x6F\x6C\x6C\x65\x72",CODE_VERSION,SPLASH_SCREEN_TIME);

  k3ngdisplay.service(0);

  #ifdef DEBUG_LOOP
  debug.print("exiting initialize_display()\n");
  Serial.flush();
  #endif // DEBUG_LOOP
} // initialize_display

void initialize_peripherals() {

  #ifdef DEBUG_LOOP
  debug.print("initialize_peripherals()\n");
  Serial.flush();
  #endif // DEBUG_LOOP

  #ifdef FEATURE_WIRE_SUPPORT
  Wire.begin();
  #endif

  #ifdef FEATURE_JOYSTICK_CONTROL
  pinModeEnhanced(pin_joystick_x, INPUT);
  pinModeEnhanced(pin_joystick_y, INPUT);
  #endif // FEATURE_JOYSTICK_CONTROL

  #ifdef FEATURE_RTC_DS1307
  rtc.begin();
  #endif // FEATURE_RTC_DS1307

  #ifdef SET_I2C_BUS_SPEED
  TWBR = ((F_CPU / SET_I2C_BUS_SPEED) - 16) / 2;
  #endif

} // initialize_peripherals

void submit_request(byte axis, byte request, int parm, byte called_by) {

  #ifdef DEBUG_SUBMIT_REQUEST
  debug.print("submit_request: ");
  debug.print(called_by);
  debug.print(" ");
  #endif // DEBUG_SUBMIT_REQUEST

  if (axis == AZ) {
    #ifdef DEBUG_SUBMIT_REQUEST
    debug.print("AZ ");
    #endif // DEBUG_SUBMIT_REQUEST
    az_request = request;
    az_request_parm = parm;
    az_request_queue_state = IN_QUEUE;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  if (axis == EL) {
    #ifdef DEBUG_SUBMIT_REQUEST
    debug.print("EL ");
    #endif // DEBUG_SUBMIT_REQUEST
    el_request = request;
    el_request_parm = parm;
    el_request_queue_state = IN_QUEUE;
  }
  #endif // FEATURE_ELEVATION_CONTROL

  #ifdef DEBUG_SUBMIT_REQUEST
  switch(request){
    case 0: debug.print("REQUEST_STOP");break;
    case 1: debug.print("REQUEST_AZIMUTH");break;
    case 2: debug.print("REQUEST_AZIMUTH_RAW");break;
    case 3: debug.print("REQUEST_CW");break;
    case 4: debug.print("REQUEST_CCW");break;
    case 5: debug.print("REQUEST_UP");break;
    case 6: debug.print("REQUEST_DOWN");break;
    case 7: debug.print("REQUEST_ELEVATION");break;
    case 8: debug.print("REQUEST_KILL");break;
  }
  debug.print(" ");
  debug.print(parm);
  debug.println("");
  #endif // DEBUG_SUBMIT_REQUEST

} // submit_request

void service_rotation() {

  static byte az_direction_change_flag = 0;
  static byte az_initial_slow_down_voltage = 0;

  #ifdef FEATURE_ELEVATION_CONTROL
  static byte el_direction_change_flag = 0;
  static byte el_initial_slow_down_voltage = 0;
  #endif // FEATURE_ELEVATION_CONTROL

  if (az_state == INITIALIZE_NORMAL_CW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CW);
    az_state = NORMAL_CW;
  }

  if (az_state == INITIALIZE_NORMAL_CCW) {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CCW);
    az_state = NORMAL_CCW;
  }

  if (az_state == INITIALIZE_SLOW_START_CW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CW);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_CW -> SLOW_START_CW");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (az_state == INITIALIZE_SLOW_START_CCW) {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE, CCW);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CCW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: INITIALIZE_SLOW_START_CCW -> SLOW_START_CCW");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CCW) {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }

  if (az_state == INITIALIZE_DIR_CHANGE_TO_CCW) {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
    az_state = TIMED_SLOW_DOWN_CW;
  }

  // slow start-------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW)) {
    if ((millis() - az_slowstart_start_time) >= AZ_SLOW_START_UP_TIME) {  // is it time to end slow start?
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: NORMAL_C");
      #endif // DEBUG_SERVICE_ROTATION
      if (az_state == SLOW_START_CW) {
        az_state = NORMAL_CW;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("W");
        #endif // DEBUG_SERVICE_ROTATION
      } else {
        az_state = NORMAL_CCW;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("CW");
        #endif // DEBUG_SERVICE_ROTATION
      }
      update_az_variable_outputs(normal_az_speed_voltage);
    } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
    if (((millis() - az_last_step_time) > (AZ_SLOW_START_UP_TIME / AZ_SLOW_START_STEPS)) && (normal_az_speed_voltage > AZ_SLOW_START_STARTING_PWM)) {
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("service_rotation: step up: ");
      debug.print(az_slow_start_step);
      debug.print(" pwm: ");
      debug.print((int)(AZ_SLOW_START_STARTING_PWM + ((normal_az_speed_voltage - AZ_SLOW_START_STARTING_PWM) * ((float)az_slow_start_step / (float)(AZ_SLOW_START_STEPS - 1)))));
      debug.println("");
      #endif // DEBUG_SERVICE_ROTATION
      update_az_variable_outputs((AZ_SLOW_START_STARTING_PWM + ((normal_az_speed_voltage - AZ_SLOW_START_STARTING_PWM) * ((float)az_slow_start_step / (float)(AZ_SLOW_START_STEPS - 1)))));
      az_last_step_time = millis();
      az_slow_start_step++;
    }
  }
} // ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW))

// timed slow down ------------------------------------------------------------------------------------------------------
if (((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW)) && ((millis() - az_last_step_time) >= (TIMED_SLOW_DOWN_TIME / AZ_SLOW_DOWN_STEPS))) {
  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: TIMED_SLOW_DOWN step down: ");
  debug.print(az_slow_down_step);
  debug.print(" pwm: ");
  debug.print((int)(normal_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
  debug.println("");
  #endif // DEBUG_SERVICE_ROTATION
  //updated 2016-05-15
  //update_az_variable_outputs((int)(normal_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
  update_az_variable_outputs((int)(current_az_speed_voltage * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)));
  az_last_step_time = millis();
  if (az_slow_down_step > 0) {az_slow_down_step--;}

  if (az_slow_down_step == 0) { // is it time to exit timed slow down?
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: TIMED_SLOW_DOWN->IDLE");
    #endif // DEBUG_SERVICE_ROTATION
    rotator(DEACTIVATE, CW);
    rotator(DEACTIVATE, CCW);
    if (az_direction_change_flag) {
      if (az_state == TIMED_SLOW_DOWN_CW) {
        //rotator(ACTIVATE, CCW);
        if (az_slowstart_active) {
          az_state = INITIALIZE_SLOW_START_CCW;
        } else { az_state = NORMAL_CCW; };
        az_direction_change_flag = 0;
      }
      if (az_state == TIMED_SLOW_DOWN_CCW) {
        //rotator(ACTIVATE, CW);
        if (az_slowstart_active) {
          az_state = INITIALIZE_SLOW_START_CW;
        } else { az_state = NORMAL_CW; };
        az_direction_change_flag = 0;
      }
    } else {
      az_state = IDLE;
      az_request_queue_state = NONE;

    }
  }

}  // ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW))

// slow down ---------------------------------------------------------------------------------------------------------------
if ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {

  // is it time to do another step down?
  if (abs((target_raw_azimuth - raw_azimuth) / HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_AZ * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS)))) {
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: step down: ");
    debug.print(az_slow_down_step);
    debug.print(" pwm: ");
    debug.print((int)(AZ_SLOW_DOWN_PWM_STOP + ((az_initial_slow_down_voltage - AZ_SLOW_DOWN_PWM_STOP) * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS))));
    debug.println("");
    #endif // DEBUG_SERVICE_ROTATION
    update_az_variable_outputs((AZ_SLOW_DOWN_PWM_STOP + ((az_initial_slow_down_voltage - AZ_SLOW_DOWN_PWM_STOP) * ((float)az_slow_down_step / (float)AZ_SLOW_DOWN_STEPS))));
    if (az_slow_down_step > 0) {az_slow_down_step--;}
  }
}  // ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW))

// normal -------------------------------------------------------------------------------------------------------------------
// if slow down is enabled, see if we're ready to go into slowdown
if (((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == NORMAL_CCW) || (az_state == SLOW_START_CCW)) &&
(az_request_queue_state == IN_PROGRESS_TO_TARGET) && az_slowdown_active && (abs((target_raw_azimuth - raw_azimuth) / HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_AZ)) {

  byte az_state_was = az_state;

  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: SLOW_DOWN_C");
  #endif // DEBUG_SERVICE_ROTATION
  az_slow_down_step = AZ_SLOW_DOWN_STEPS - 1;
  if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW)) {
    az_state = SLOW_DOWN_CW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("W");
    #endif // DEBUG_SERVICE_ROTATION
  } else {
    az_state = SLOW_DOWN_CCW;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("CW");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if ((az_state_was == SLOW_START_CW) || (az_state_was == SLOW_START_CCW)){
    az_initial_slow_down_voltage = (AZ_INITIALLY_IN_SLOW_DOWN_PWM);
    update_az_variable_outputs(az_initial_slow_down_voltage);
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print(" SLOW_START -> SLOW_DOWN az_initial_slow_down_voltage:");
    debug.print(az_initial_slow_down_voltage);
    debug.print(" ");
    #endif // DEBUG_SERVICE_ROTATION
  } else {
    if (AZ_SLOW_DOWN_PWM_START < current_az_speed_voltage) {
      update_az_variable_outputs(AZ_SLOW_DOWN_PWM_START);
      az_initial_slow_down_voltage = AZ_SLOW_DOWN_PWM_START;
    } else {
      az_initial_slow_down_voltage = current_az_speed_voltage;
    }
  }

}

// check rotation target --------------------------------------------------------------------------------------------------------
if ((az_state != IDLE) && (az_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
  if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW) || (az_state == SLOW_DOWN_CW)) {
    if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
      delay(50);
      read_azimuth(0);
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth > target_raw_azimuth) && ((raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        rotator(DEACTIVATE, CW);
        rotator(DEACTIVATE, CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: IDLE");
        #endif // DEBUG_SERVICE_ROTATION
      }
    }
  } else {
    if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
      delay(50);
      read_azimuth(0);
      if ((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)) || ((raw_azimuth < target_raw_azimuth) && ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        rotator(DEACTIVATE, CW);
        rotator(DEACTIVATE, CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: IDLE");
        #endif // DEBUG_SERVICE_ROTATION
      }
    }
  }
}

#ifdef FEATURE_ELEVATION_CONTROL
if (el_state == INITIALIZE_NORMAL_UP) {
  update_el_variable_outputs(normal_el_speed_voltage);
  rotator(ACTIVATE, UP);
  el_state = NORMAL_UP;
}

if (el_state == INITIALIZE_NORMAL_DOWN) {
  update_el_variable_outputs(normal_el_speed_voltage);
  rotator(ACTIVATE, DOWN);
  el_state = NORMAL_DOWN;
}

if (el_state == INITIALIZE_SLOW_START_UP) {
  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
  rotator(ACTIVATE, UP);
  el_slowstart_start_time = millis();
  el_last_step_time = 0;
  el_slow_start_step = 0;
  el_state = SLOW_START_UP;
  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: INITIALIZE_SLOW_START_UP -> SLOW_START_UP");
  #endif // DEBUG_SERVICE_ROTATION
}

if (el_state == INITIALIZE_SLOW_START_DOWN) {
  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
  rotator(ACTIVATE, DOWN);
  el_slowstart_start_time = millis();
  el_last_step_time = 0;
  el_slow_start_step = 0;
  el_state = SLOW_START_DOWN;
  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: INITIALIZE_SLOW_START_DOWN -> SLOW_START_DOWN");
  #endif // DEBUG_SERVICE_ROTATION
}

if (el_state == INITIALIZE_TIMED_SLOW_DOWN_UP) {
  el_direction_change_flag = 0;
  el_timed_slow_down_start_time = millis();
  el_last_step_time = millis();
  el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
  el_state = TIMED_SLOW_DOWN_UP;
}

if (el_state == INITIALIZE_TIMED_SLOW_DOWN_DOWN) {
  el_direction_change_flag = 0;
  el_timed_slow_down_start_time = millis();
  el_last_step_time = millis();
  el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
  el_state = TIMED_SLOW_DOWN_DOWN;
}

if (el_state == INITIALIZE_DIR_CHANGE_TO_UP) {
  el_direction_change_flag = 1;
  el_timed_slow_down_start_time = millis();
  el_last_step_time = millis();
  el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
  el_state = TIMED_SLOW_DOWN_DOWN;
}

if (el_state == INITIALIZE_DIR_CHANGE_TO_DOWN) {
  el_direction_change_flag = 1;
  el_timed_slow_down_start_time = millis();
  el_last_step_time = millis();
  el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
  el_state = TIMED_SLOW_DOWN_UP;
}

// slow start-------------------------------------------------------------------------------------------------
if ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN)) {
  if ((millis() - el_slowstart_start_time) >= EL_SLOW_START_UP_TIME) {  // is it time to end slow start?
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: NORMAL_");
    #endif // DEBUG_SERVICE_ROTATION
    if (el_state == SLOW_START_UP) {
      el_state = NORMAL_UP;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("UP");
      #endif // DEBUG_SERVICE_ROTATION
    } else {
      el_state = NORMAL_DOWN;
      #ifdef DEBUG_SERVICE_ROTATION
      debug.print("DOWN");
      #endif // DEBUG_SERVICE_ROTATION
    }
    update_el_variable_outputs(normal_el_speed_voltage);
  } else {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
  if (((millis() - el_last_step_time) > (EL_SLOW_START_UP_TIME / EL_SLOW_START_STEPS)) && (normal_el_speed_voltage > EL_SLOW_START_STARTING_PWM)) {
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: step up: ");
    debug.print(el_slow_start_step);
    debug.print(" pwm: ");
    debug.print((int)(EL_SLOW_START_STARTING_PWM + ((normal_el_speed_voltage - EL_SLOW_START_STARTING_PWM) * ((float)el_slow_start_step / (float)(EL_SLOW_START_STEPS - 1)))));
    debug.println("");
    #endif // DEBUG_SERVICE_ROTATION
    update_el_variable_outputs((EL_SLOW_START_STARTING_PWM + ((normal_el_speed_voltage - EL_SLOW_START_STARTING_PWM) * ((float)el_slow_start_step / (float)(EL_SLOW_START_STEPS - 1)))));
    el_last_step_time = millis();
    el_slow_start_step++;
  }
}
} // ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))


// timed slow down ------------------------------------------------------------------------------------------------------
if (((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) && ((millis() - el_last_step_time) >= (TIMED_SLOW_DOWN_TIME / EL_SLOW_DOWN_STEPS))) {
  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: TIMED_SLOW_DOWN step down: ");
  debug.print(el_slow_down_step);
  debug.print(" pwm: ");
  debug.print((int)(normal_el_speed_voltage * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)));
  debug.println("");
  #endif // DEBUG_SERVICE_ROTATION
  update_el_variable_outputs((int)(normal_el_speed_voltage * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)));
  el_last_step_time = millis();
  if (el_slow_down_step > 0) {el_slow_down_step--;}

  if (el_slow_down_step == 0) { // is it time to exit timed slow down?
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: TIMED_SLOW_DOWN->IDLE");
    #endif // DEBUG_SERVICE_ROTATION
    rotator(DEACTIVATE, UP);
    rotator(DEACTIVATE, DOWN);
    if (el_direction_change_flag) {
      if (el_state == TIMED_SLOW_DOWN_UP) {
        if (el_slowstart_active) {
          el_state = INITIALIZE_SLOW_START_DOWN;
        } else { el_state = NORMAL_DOWN; };
        el_direction_change_flag = 0;
      }
      if (el_state == TIMED_SLOW_DOWN_DOWN) {
        if (el_slowstart_active) {
          el_state = INITIALIZE_SLOW_START_UP;
        } else { el_state = NORMAL_UP; };
        el_direction_change_flag = 0;
      }
    } else {
      el_state = IDLE;
      el_request_queue_state = NONE;

    }
  }

}  // ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN))



// slow down ---------------------------------------------------------------------------------------------------------------
if ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {
  // is it time to do another step down?
  if (abs((target_elevation - elevation) / HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_EL * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS)))) {
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("service_rotation: step down: ");
    debug.print(el_slow_down_step);
    debug.print(" pwm: ");
    debug.print((int)(EL_SLOW_DOWN_PWM_STOP + ((el_initial_slow_down_voltage - EL_SLOW_DOWN_PWM_STOP) * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS))));
    debug.println("");
    #endif // DEBUG_SERVICE_ROTATION
    update_el_variable_outputs((EL_SLOW_DOWN_PWM_STOP + ((el_initial_slow_down_voltage - EL_SLOW_DOWN_PWM_STOP) * ((float)el_slow_down_step / (float)EL_SLOW_DOWN_STEPS))));
    if (el_slow_down_step > 0) {el_slow_down_step--;}
  }
}  // ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))

// normal -------------------------------------------------------------------------------------------------------------------
// if slow down is enabled, see if we're ready to go into slowdown
if (((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == NORMAL_DOWN) || (el_state == SLOW_START_DOWN)) &&
(el_request_queue_state == IN_PROGRESS_TO_TARGET) && el_slowdown_active && (abs((target_elevation - elevation) / HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_EL)) {

  byte el_state_was = el_state;


  #ifdef DEBUG_SERVICE_ROTATION
  debug.print("service_rotation: SLOW_DOWN_");
  #endif // DEBUG_SERVICE_ROTATION
  el_slow_down_step = EL_SLOW_DOWN_STEPS - 1;
  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP)) {
    el_state = SLOW_DOWN_UP;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("UP");
    #endif // DEBUG_SERVICE_ROTATION
  } else {
    el_state = SLOW_DOWN_DOWN;
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print("DOWN");
    #endif // DEBUG_SERVICE_ROTATION
  }

  if ((el_state_was == SLOW_START_UP) || (el_state_was == SLOW_START_DOWN)){
    el_initial_slow_down_voltage = EL_INITIALLY_IN_SLOW_DOWN_PWM;
    update_el_variable_outputs(el_initial_slow_down_voltage);
    #ifdef DEBUG_SERVICE_ROTATION
    debug.print(" SLOW_START -> SLOW_DOWN el_initial_slow_down_voltage:");
    debug.print(el_initial_slow_down_voltage);
    debug.print(" ");
    #endif // DEBUG_SERVICE_ROTATION

  } else {
    if (EL_SLOW_DOWN_PWM_START < current_el_speed_voltage) {
      update_el_variable_outputs(EL_SLOW_DOWN_PWM_START);
      el_initial_slow_down_voltage = EL_SLOW_DOWN_PWM_START;
    } else {
      el_initial_slow_down_voltage = current_el_speed_voltage;
    }
  }
}

// check rotation target --------------------------------------------------------------------------------------------------------
if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
  read_elevation(0);
  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == SLOW_DOWN_UP)) {
    if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
      #ifndef OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
      delay(50);
      #endif //OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
      read_elevation(0);
      if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        rotator(DEACTIVATE, UP);
        rotator(DEACTIVATE, DOWN);
        el_state = IDLE;
        el_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: IDLE");
        #endif // DEBUG_SERVICE_ROTATION
      }
    }
  } else {
    read_elevation(0);
    if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
      #ifndef OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
      delay(50);
      #endif //OPTION_NO_ELEVATION_CHECK_TARGET_DELAY
      read_elevation(0);
      if ((abs(elevation - target_elevation) <= (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) || ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE + 5) * HEADING_MULTIPLIER)))) {
        rotator(DEACTIVATE, UP);
        rotator(DEACTIVATE, DOWN);
        el_state = IDLE;
        el_request_queue_state = NONE;
        #ifdef DEBUG_SERVICE_ROTATION
        debug.print("service_rotation: IDLE");
        #endif // DEBUG_SERVICE_ROTATION
      }
    }
  }
}




#endif // FEATURE_ELEVATION_CONTROL

} /* service_rotation */

void stop_all_tracking() {

  #ifdef FEATURE_MOON_TRACKING
  moon_tracking_active = 0;
  #endif // FEATURE_MOON_TRACKING

  #ifdef FEATURE_SUN_TRACKING
  sun_tracking_active = 0;
  #endif // FEATURE_SUN_TRACKING
} // stop_all_tracking

void service_request_queue() {

  int work_target_raw_azimuth = 0;
  byte direction_to_go = 0;
  byte within_tolerance_flag = 0;

  if (az_request_queue_state == IN_QUEUE) {

    #ifdef FEATURE_POWER_SWITCH
    last_activity_time = millis();
    #endif //FEATURE_POWER_SWITCH

    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("service_request_queue: AZ ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE

    switch (az_request) {
      case (REQUEST_STOP):
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      debug.print("REQUEST_STOP");
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      stop_all_tracking();
      if (az_state != IDLE) {
        if (az_slowdown_active) {
          if ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW) || (az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) {  // if we're already in timed slow down and we get another stop, do a hard stop
          rotator(DEACTIVATE, CW);
          rotator(DEACTIVATE, CCW);
          az_state = IDLE;
          az_request_queue_state = NONE;
        }
        if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW)) {
          az_state = INITIALIZE_TIMED_SLOW_DOWN_CW;
          az_request_queue_state = IN_PROGRESS_TIMED;
          az_last_rotate_initiation = millis();
        }
        if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW)) {
          az_state = INITIALIZE_TIMED_SLOW_DOWN_CCW;
          az_request_queue_state = IN_PROGRESS_TIMED;
          az_last_rotate_initiation = millis();
        }

      } else {
        rotator(DEACTIVATE, CW);
        rotator(DEACTIVATE, CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;
      }
    } else {
      az_request_queue_state = NONE; // nothing to do - we clear the queue
    }
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    if (debug_mode) {
      control_port->println();
    }
    #endif // DEBUG_SERVICE_REQUEST_QUEUE
    break; // REQUEST_STOP

    case (REQUEST_AZIMUTH):
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("REQUEST_AZIMUTH");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE
    if ((az_request_parm >= 0) && (az_request_parm <= (360 * HEADING_MULTIPLIER))) {
      target_azimuth = az_request_parm;
      target_raw_azimuth = az_request_parm;
      if (target_azimuth == (360 * HEADING_MULTIPLIER)) {
        target_azimuth = 0;
      }
      if ((target_azimuth > (azimuth - (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))) && (target_azimuth < (azimuth + (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER)))) {
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print(" request within tolerance");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        within_tolerance_flag = 1;
        // az_request_queue_state = NONE;
        if (az_state != IDLE){
          submit_request(AZ, REQUEST_STOP, 0, 137);
        } else {
          az_request_queue_state = NONE;
        }
      } else {  // target azimuth is not within tolerance, we need to rotate
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print(" ->A");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        work_target_raw_azimuth = target_azimuth;
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print(" work_target_raw_azimuth:");
        debug.print(work_target_raw_azimuth / HEADING_MULTIPLIER);
        debug.print(" azimuth_starting_point:");
        debug.print(azimuth_starting_point);
        debug.print(" ");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE

        if (work_target_raw_azimuth < (azimuth_starting_point * HEADING_MULTIPLIER)) {
          work_target_raw_azimuth = work_target_raw_azimuth + (360 * HEADING_MULTIPLIER);
          target_raw_azimuth = work_target_raw_azimuth;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          debug.print("->B");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        }
        if ((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) < ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER)) { // is there a second possible heading in overlap?
          if (abs(raw_azimuth - work_target_raw_azimuth) < abs((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) - raw_azimuth)) { // is second possible heading closer?
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print("->C");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
            if (work_target_raw_azimuth  > raw_azimuth) { // not closer, use position in non-overlap
              direction_to_go = CW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->CW!");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              direction_to_go = CCW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->CCW!");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            }
          } else { // go to position in overlap
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            debug.print("->D");
            #endif // DEBUG_SERVICE_REQUEST_QUEUE
            target_raw_azimuth = work_target_raw_azimuth + (360 * HEADING_MULTIPLIER);
            if ((work_target_raw_azimuth + (360 * HEADING_MULTIPLIER)) > raw_azimuth) {
              direction_to_go = CW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->CW!");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            } else {
              direction_to_go = CCW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              debug.print("->CCW!");
              #endif // DEBUG_SERVICE_REQUEST_QUEUE
            }
          }
        } else {  // no possible second heading in overlap
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          debug.print("->E");
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
          if (work_target_raw_azimuth  > raw_azimuth) {
            direction_to_go = CW;
          } else {
            direction_to_go = CCW;
          }
        }
      }
    } else {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      debug.print("->F");
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      if ((az_request_parm > (360 * HEADING_MULTIPLIER)) && (az_request_parm <= ((azimuth_starting_point + azimuth_rotation_capability) * HEADING_MULTIPLIER))) {
        target_azimuth = az_request_parm - (360 * HEADING_MULTIPLIER);
        target_raw_azimuth = az_request_parm;
        if (az_request_parm > raw_azimuth) {
          direction_to_go = CW;
        } else {
          direction_to_go = CCW;
        }
      } else {
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print(" error: bogus azimuth request:");
        debug.print(az_request_parm);
        debug.println("");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        rotator(DEACTIVATE, CW);
        rotator(DEACTIVATE, CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;
        return;
      }
    }
    if (direction_to_go == CW) {
      if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
        az_state = INITIALIZE_DIR_CHANGE_TO_CW;
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
      } else {
        if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
        // rotator(ACTIVATE,CW);
        if (az_slowstart_active) {
          az_state = INITIALIZE_SLOW_START_CW;
        } else { az_state = INITIALIZE_NORMAL_CW; };
      }
    }
  }
  if (direction_to_go == CCW) {
    if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
      az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
    } else {
      if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
      // rotator(ACTIVATE,CCW);
      if (az_slowstart_active) {
        az_state = INITIALIZE_SLOW_START_CCW;
      } else { az_state = INITIALIZE_NORMAL_CCW; };
    }
  }
}
if (!within_tolerance_flag) {
  az_request_queue_state = IN_PROGRESS_TO_TARGET;
  az_last_rotate_initiation = millis();
}
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_AZIMUTH

case (REQUEST_AZIMUTH_RAW):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
debug.print("REQUEST_AZIMUTH_RAW");
#endif // DEBUG_SERVICE_REQUEST_QUEUE
target_raw_azimuth = az_request_parm;
target_azimuth = target_raw_azimuth;
if (target_azimuth >= (360 * HEADING_MULTIPLIER)) {
  target_azimuth = target_azimuth - (360 * HEADING_MULTIPLIER);
}

if (((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))) && (az_state == IDLE)) {
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print(" request within tolerance");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  if (az_state != IDLE){
    submit_request(AZ, REQUEST_STOP, 0, 138);
  } else {
    az_request_queue_state = NONE;
  }
  within_tolerance_flag = 1;
} else {
  if (target_raw_azimuth > raw_azimuth) {
    if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
      az_state = INITIALIZE_DIR_CHANGE_TO_CW;
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
    } else {
      if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) { // if we're already rotating CW, don't do anything
      if (az_slowstart_active) {
        az_state = INITIALIZE_SLOW_START_CW;
      } else { az_state = INITIALIZE_NORMAL_CW; };
    }
  }
}
if (target_raw_azimuth < raw_azimuth) {
  if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
    az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE
  } else {
    if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) { // if we're already rotating CCW, don't do anything
    if (az_slowstart_active) {
      az_state = INITIALIZE_SLOW_START_CCW;
    } else { az_state = INITIALIZE_NORMAL_CCW; };
  }
}
}
if (!within_tolerance_flag) {
  az_request_queue_state = IN_PROGRESS_TO_TARGET;
  az_last_rotate_initiation = millis();
}
}
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_AZIMUTH_RAW

case (REQUEST_CW):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
debug.print("REQUEST_CW");
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)) {
  az_state = INITIALIZE_DIR_CHANGE_TO_CW;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print(" INITIALIZE_DIR_CHANGE_TO_CW");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
} else {
  if ((az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) {
    // rotator(ACTIVATE,CW);
    if (az_slowstart_active) {
      az_state = INITIALIZE_SLOW_START_CW;
    } else {
      az_state = INITIALIZE_NORMAL_CW;
    };
  }
}
az_request_queue_state = NONE;
az_last_rotate_initiation = millis();
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_CW

case (REQUEST_CCW):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
debug.print("REQUEST_CCW");
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active)) {
  az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print(" INITIALIZE_DIR_CHANGE_TO_CCW");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
} else {
  if ((az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) {
    // rotator(ACTIVATE,CCW);
    if (az_slowstart_active) {
      az_state = INITIALIZE_SLOW_START_CCW;
    } else { az_state = INITIALIZE_NORMAL_CCW; };
  }
}
az_request_queue_state = NONE;
az_last_rotate_initiation = millis();
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_CCW

case (REQUEST_KILL):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
debug.print("REQUEST_KILL");
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
rotator(DEACTIVATE, CW);
rotator(DEACTIVATE, CCW);
az_state = IDLE;
az_request_queue_state = NONE;
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
debug.println("");
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_KILL
}

#ifdef FEATURE_LCD_DISPLAY
if (az_request_queue_state != IN_QUEUE) {push_lcd_update = 1;}
#endif //FEATURE_LCD_DISPLAY
}
#ifdef FEATURE_ELEVATION_CONTROL
if (el_request_queue_state == IN_QUEUE) {

  #ifdef FEATURE_POWER_SWITCH
  last_activity_time = millis();
  #endif //FEATURE_POWER_SWITCH

  within_tolerance_flag = 0;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print("service_request_queue: EL ");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  switch (el_request) {
    case (REQUEST_ELEVATION):
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("REQUEST_ELEVATION ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE
    target_elevation = el_request_parm;

    if (target_elevation > (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER)) {
      target_elevation = ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER;
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_ELEVATION: target_elevation > ELEVATION_MAXIMUM_DEGREES"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
    }

    #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
    if (target_elevation < (EL_MANUAL_ROTATE_DOWN_LIMIT * HEADING_MULTIPLIER)) {
      target_elevation = EL_MANUAL_ROTATE_DOWN_LIMIT * HEADING_MULTIPLIER;
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_ELEVATION: target_elevation < EL_MANUAL_ROTATE_DOWN_LIMIT"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
    }
    if (target_elevation > (EL_MANUAL_ROTATE_UP_LIMIT * HEADING_MULTIPLIER)) {
      target_elevation = EL_MANUAL_ROTATE_UP_LIMIT * HEADING_MULTIPLIER;
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_ELEVATION: target_elevation > EL_MANUAL_ROTATE_UP_LIMIT"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
    }
    #endif // OPTION_EL_MANUAL_ROTATE_LIMITS

    if (abs(target_elevation - elevation) < (ELEVATION_TOLERANCE * HEADING_MULTIPLIER)) {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("requested elevation within tolerance\n"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      within_tolerance_flag = 1;
      el_request_queue_state = NONE;
    } else {
      if (target_elevation > elevation) {
        if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
          el_state = INITIALIZE_DIR_CHANGE_TO_UP;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {
            debug.print(F(" INITIALIZE_DIR_CHANGE_TO_UP\n"));
          }
          #endif // DEBUG_SERVICE_REQUEST_QUEUE
        } else {
          if ((el_state != INITIALIZE_SLOW_START_UP) && (el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) { // if we're already rotating UP, don't do anything
          if (el_slowstart_active) {
            el_state = INITIALIZE_SLOW_START_UP;
          } else { el_state = INITIALIZE_NORMAL_UP; };
        }
      }
    } // (target_elevation > elevation)
    if (target_elevation < elevation) {
      if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
        el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {
          debug.print(F(" INITIALIZE_DIR_CHANGE_TO_DOWN\n"));
        }
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
      } else {
        if ((el_state != INITIALIZE_SLOW_START_DOWN) && (el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) { // if we're already rotating DOWN, don't do anything
        if (el_slowstart_active) {
          el_state = INITIALIZE_SLOW_START_DOWN;
        } else { el_state = INITIALIZE_NORMAL_DOWN; };
      }
    }
  }  // (target_elevation < elevation)
}  // (abs(target_elevation - elevation) < ELEVATION_TOLERANCE)
if (!within_tolerance_flag) {
  el_request_queue_state = IN_PROGRESS_TO_TARGET;
  el_last_rotate_initiation = millis();
}
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_ELEVATION

case (REQUEST_UP):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  debug.print(F("REQUEST_UP\n"));
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active)) {
  el_state = INITIALIZE_DIR_CHANGE_TO_UP;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  if (debug_mode) {
    debug.print(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_UP\n"));
  }
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
} else {
  if ((el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) {
    if (el_slowstart_active) {
      el_state = INITIALIZE_SLOW_START_UP;
    } else { el_state = INITIALIZE_NORMAL_UP; };
  }
}
el_request_queue_state = NONE;
el_last_rotate_initiation = millis();
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_UP

case (REQUEST_DOWN):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  debug.print(F("REQUEST_DOWN\n"));
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active)) {
  el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  if (debug_mode) {
    debug.print(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_DOWN\n"));
  }
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
} else {
  if ((el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) {
    if (el_slowstart_active) {
      el_state = INITIALIZE_SLOW_START_DOWN;
    } else { el_state = INITIALIZE_NORMAL_DOWN; };
  }
}
el_request_queue_state = NONE;
el_last_rotate_initiation = millis();
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_DOWN

case (REQUEST_STOP):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  debug.print(F("REQUEST_STOP\n"));
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
if (el_state != IDLE) {
  if (el_slowdown_active) {
    if ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN) || (el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN)) {  // if we're already in timed slow down and we get another stop, do a hard stop
    rotator(DEACTIVATE, UP);
    rotator(DEACTIVATE, DOWN);
    el_state = IDLE;
    el_request_queue_state = NONE;
  }
  if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP)) {
    el_state = INITIALIZE_TIMED_SLOW_DOWN_UP;
    el_request_queue_state = IN_PROGRESS_TIMED;
    el_last_rotate_initiation = millis();
  }
  if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN)) {
    el_state = INITIALIZE_TIMED_SLOW_DOWN_DOWN;
    el_request_queue_state = IN_PROGRESS_TIMED;
    el_last_rotate_initiation = millis();
  }
} else {
  rotator(DEACTIVATE, UP);
  rotator(DEACTIVATE, DOWN);
  el_state = IDLE;
  el_request_queue_state = NONE;
}
} else {
  el_request_queue_state = NONE; // nothing to do, we're already in IDLE state
}
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_STOP

case (REQUEST_KILL):
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  debug.print(F("REQUEST_KILL\n"));
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
stop_all_tracking();
rotator(DEACTIVATE, UP);
rotator(DEACTIVATE, DOWN);
el_state = IDLE;
el_request_queue_state = NONE;
#ifdef DEBUG_SERVICE_REQUEST_QUEUE
if (debug_mode) {
  control_port->println();
}
#endif // DEBUG_SERVICE_REQUEST_QUEUE
break; // REQUEST_KILL
} /* switch */

#ifdef FEATURE_LCD_DISPLAY
if (el_request_queue_state != IN_QUEUE) {push_lcd_update = 1;}
#endif //FEATURE_LCD_DISPLAY

} // (el_request_queue_state == IN_QUEUE)
#endif // FEATURE_ELEVATION_CONTROL
} /* service_request_queue */

void check_for_dirty_configuration() {

  static unsigned long last_config_write_time = 0;

  if ((configuration_dirty) && ((millis() - last_config_write_time) > ((unsigned long)EEPROM_WRITE_DIRTY_CONFIG_TIME * 1000))) {
    write_settings_to_eeprom();
    last_config_write_time = millis();
  }

} // check_for_dirty_configuration

byte current_az_state() {

  if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) {
    return ROTATING_CW;
  }
  if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) {
    return ROTATING_CCW;
  }
  return NOT_DOING_ANYTHING;

} // current_az_state

byte current_el_state() {
  if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) {
    return ROTATING_UP;
  }
  if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) {
    return ROTATING_DOWN;
  }
  return NOT_DOING_ANYTHING;
} // current_el_state

#ifdef FEATURE_AZIMUTH_CORRECTION
float correct_azimuth(float azimuth_in){

  if (sizeof(azimuth_calibration_from) != sizeof(azimuth_calibration_to)) {
    return azimuth_in;
  }
  for (unsigned int x = 0; x < (sizeof(azimuth_calibration_from) - 2); x++) {
    if ((azimuth_in >= azimuth_calibration_from[x]) && (azimuth_in <= azimuth_calibration_from[x + 1])) {
      //return (map(azimuth_in * 10, azimuth_calibration_from[x] * 10, azimuth_calibration_from[x + 1] * 10, azimuth_calibration_to[x] * 10, azimuth_calibration_to[x + 1] * 10)) / 10.0;
      return (azimuth_in - azimuth_calibration_from[x]) * (azimuth_calibration_to[x+1] - azimuth_calibration_to[x]) / (azimuth_calibration_from[x + 1] - azimuth_calibration_from[x]) + azimuth_calibration_to[x];
    }
  }
  return(azimuth_in);

}
#endif // FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
float correct_elevation(float elevation_in) {
  if (sizeof(elevation_calibration_from) != sizeof(elevation_calibration_to)) {
    return elevation_in;
  }
  for (int x = 0; x < (sizeof(elevation_calibration_from) - 2); x++) {
    if ((elevation_in >= elevation_calibration_from[x]) && (elevation_in <= elevation_calibration_from[x + 1])) {
      // changed this from map() 2015-03-28 due to it blowing up at compile time in Arduino 1.6.1
      return (elevation_in - elevation_calibration_from[x]) * (elevation_calibration_to[x+1] - elevation_calibration_to[x]) / (elevation_calibration_from[x + 1] - elevation_calibration_from[x]) + elevation_calibration_to[x];
    }
  }

  return(elevation_in);
}
#endif // FEATURE_ELEVATION_CORRECTION

#ifdef FEATURE_JOYSTICK_CONTROL
void check_joystick() {

  int joystick_x = 0;
  int joystick_y = 0;

  static int joystick_resting_x = 0;
  static int joystick_resting_y = 0;

  static unsigned long last_joystick_az_action_time = 0;

  static byte joystick_azimuth_rotation = NOT_DOING_ANYTHING;

  static byte joystick_elevation_rotation = NOT_DOING_ANYTHING;
  static unsigned long last_joystick_el_action_time = 0;

  if ((joystick_resting_x == 0) || (joystick_resting_y == 0)) {
    // initialize the resting readings if this is our first time here
    joystick_resting_x = analogReadEnhanced(pin_joystick_x);
    joystick_resting_y = analogReadEnhanced(pin_joystick_y);

  } else {

    joystick_x = analogReadEnhanced(pin_joystick_x);
    joystick_y = analogReadEnhanced(pin_joystick_y);

    if ((millis() - last_joystick_az_action_time) > JOYSTICK_WAIT_TIME_MS) {
      #ifdef DEBUG_JOYSTICK
      static unsigned long last_debug_joystick_status = 0;

      if ((debug_mode) && ((millis() - last_debug_joystick_status) > 1000)) {
        debug.print("check_joystick: x: ");
        debug.print(joystick_x);
        debug.print("\ty: ");
        control_port->println(joystick_y);
        last_debug_joystick_status = millis();
      }
      #endif // DEBUG_JOYSTICK

      #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
      if ((joystick_resting_x - joystick_x) < (joystick_resting_x * -0.2)) {   // left
        #else
        if ((joystick_resting_x - joystick_x) > (joystick_resting_x * 0.2)) {
          #endif
          #ifdef DEBUG_JOYSTICK
          if (debug_mode) {
            control_port->println("check_joystick: L");
          }
          #endif // DEBUG_JOYSTICK
          if (current_az_state() != ROTATING_CCW) {
            submit_request(AZ, REQUEST_CCW, 0, 1);
          }
          joystick_azimuth_rotation = ROTATING_CCW;
          last_joystick_az_action_time = millis();

        } else {
          #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
          if ((joystick_resting_x - joystick_x) > (joystick_resting_x * 0.2)) {  // right
            #else
            if ((joystick_resting_x - joystick_x) < (joystick_resting_x * -0.2)) {
              #endif
              #ifdef DEBUG_JOYSTICK
              if (debug_mode) {
                control_port->println("check_joystick: R");
              }
              #endif // DEBUG_JOYSTICK
              if (current_az_state() != ROTATING_CW) {
                submit_request(AZ, REQUEST_CW, 0, 2);
              }
              joystick_azimuth_rotation = ROTATING_CW;
              last_joystick_az_action_time = millis();

            } else { // joystick is in X axis resting position
              if (joystick_azimuth_rotation != NOT_DOING_ANYTHING) {
                if (current_az_state() != NOT_DOING_ANYTHING) {
                  submit_request(AZ, REQUEST_STOP, 0, 3);
                  last_joystick_az_action_time = millis();
                }
                joystick_azimuth_rotation = NOT_DOING_ANYTHING;
              }
            }

          }

        }

        #ifdef FEATURE_ELEVATION_CONTROL
        if ((millis() - last_joystick_el_action_time) > JOYSTICK_WAIT_TIME_MS) {
          #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
          if ((joystick_resting_y - joystick_y) > (joystick_resting_y * 0.2)) {  // down
            #else
            if ((joystick_resting_y - joystick_y) < (joystick_resting_y * -0.2)) {
              #endif
              #ifdef DEBUG_JOYSTICK
              if (debug_mode) {
                control_port->println("check_joystick: D");
              }
              #endif // DEBUG_JOYSTICK
              if (current_el_state() != ROTATING_DOWN) {
                submit_request(EL, REQUEST_DOWN, 0, 4);
              }
              joystick_elevation_rotation = ROTATING_DOWN;
              last_joystick_el_action_time = millis();
            } else {
              #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
              if ((joystick_resting_y - joystick_y) < (joystick_resting_y * -0.2)) { // up
                #else
                if ((joystick_resting_y - joystick_y) > (joystick_resting_y * 0.2)) {
                  #endif
                  #ifdef DEBUG_JOYSTICK
                  if (debug_mode) {
                    control_port->println("check_joystick: U");
                  }
                  #endif // DEBUG_JOYSTICK
                  if (current_el_state() != ROTATING_UP) {
                    submit_request(EL, REQUEST_UP, 0, 5);
                  }
                  joystick_elevation_rotation = ROTATING_UP;
                  last_joystick_el_action_time = millis();

                } else {  // Y axis is in resting position
                  if (joystick_elevation_rotation != NOT_DOING_ANYTHING) {
                    if (current_el_state() != NOT_DOING_ANYTHING) {
                      submit_request(EL, REQUEST_STOP, 0, 6);
                      last_joystick_el_action_time = millis();
                    }
                    joystick_elevation_rotation = NOT_DOING_ANYTHING;
                  }
                }
              }


            }
            #endif // FEATURE_ELEVATION_CONTROL

          }


        } /* check_joystick */
        #endif // FEATURE_JOYSTICK_CONTROL

        #ifdef FEATURE_ROTATION_INDICATOR_PIN
        void service_rotation_indicator_pin(){


          static byte rotation_indication_pin_state = 0;
          static unsigned long time_rotation_went_inactive = 0;

          #ifdef FEATURE_ELEVATION_CONTROL
          if ((!rotation_indication_pin_state) && ((az_state != IDLE) || (el_state != IDLE))) {
            #else
            if ((!rotation_indication_pin_state) && ((az_state != IDLE))) {
              #endif
              if (rotation_indication_pin) {
                digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_ACTIVE_STATE);
              }
              rotation_indication_pin_state = 1;
              #ifdef DEBUG_ROTATION_INDICATION_PIN
              if (debug_mode) {
                debug.print(F("service_rotation_indicator_pin: active\n"));
              }
              #endif
            }

            #ifdef FEATURE_ELEVATION_CONTROL
            if ((rotation_indication_pin_state) && (az_state == IDLE) && (el_state == IDLE)) {
              #else
              if ((rotation_indication_pin_state) && (az_state == IDLE)) {
                #endif
                if (time_rotation_went_inactive == 0) {
                  time_rotation_went_inactive = millis();
                } else {
                  if ((millis() - time_rotation_went_inactive) >= (((unsigned long)ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS * 1000) + ((unsigned long)ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES * 60 * 1000))) {
                    if (rotation_indication_pin) {
                      digitalWriteEnhanced(rotation_indication_pin, ROTATION_INDICATOR_PIN_INACTIVE_STATE);
                    }
                    rotation_indication_pin_state = 0;
                    time_rotation_went_inactive = 0;
                    #ifdef DEBUG_ROTATION_INDICATION_PIN
                    if (debug_mode) {
                      debug.print(F("service_rotation_indicator_pin: inactive\n"));
                    }
                    #endif
                  }
                }
              }


            } /* service_rotation_indicator_pin */
            #endif // FEATURE_ROTATION_INDICATOR_PIN


            #ifdef FEATURE_LIMIT_SENSE
            void check_limit_sense(){

              static byte az_limit_tripped = 0;

              #ifdef FEATURE_ELEVATION_CONTROL
              static byte el_limit_tripped = 0;
              #endif // FEATURE_ELEVATION_CONTROL

              if (az_limit_sense_pin) {
                if (digitalReadEnhanced(az_limit_sense_pin) == 0) {
                  if (!az_limit_tripped) {
                    submit_request(AZ, REQUEST_KILL, 0, 9);
                    az_limit_tripped = 1;
                    #ifdef DEBUG_LIMIT_SENSE
                    debug.print(F("check_limit_sense: az limit tripped\n"));
                    #endif // DEBUG_LIMIT_SENSE
                  }
                } else {
                  az_limit_tripped = 0;
                }
              }

              #ifdef FEATURE_ELEVATION_CONTROL
              if (el_limit_sense_pin) {
                if (digitalReadEnhanced(el_limit_sense_pin) == 0) {
                  if (!el_limit_tripped) {
                    submit_request(EL, REQUEST_KILL, 0, 10);
                    el_limit_tripped = 1;
                    #ifdef DEBUG_LIMIT_SENSE
                    debug.print(F("check_limit_sense: el limit tripped\n"));
                    #endif // DEBUG_LIMIT_SENSE
                  }
                } else {
                  el_limit_tripped = 0;
                }
              }
              #endif // FEATURE_ELEVATION_CONTROL


            } /* check_limit_sense */
            #endif // FEATURE_LIMIT_SENSE

            #ifdef FEATURE_AZ_POSITION_INCREMENTAL_ENCODER
            void az_position_incremental_encoder_interrupt_handler(){

              byte rotation_result = 0;
              byte current_phase_a = digitalReadEnhanced(az_incremental_encoder_pin_phase_a);
              byte current_phase_b = digitalReadEnhanced(az_incremental_encoder_pin_phase_b);
              byte current_phase_z = digitalReadEnhanced(az_incremental_encoder_pin_phase_z);

              #ifdef DEBUG_AZ_POSITION_INCREMENTAL_ENCODER
              az_position_incremental_encoder_interrupt++;
              #endif // DEBUG_AZ_POSITION_INCREMENTAL_ENCODER

              if ((az_3_phase_encoder_last_phase_a_state != current_phase_a) || (az_3_phase_encoder_last_phase_b_state != current_phase_b)) {
                if (az_3_phase_encoder_last_phase_a_state == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (az_3_phase_encoder_last_phase_b_state == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (current_phase_a == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (current_phase_b == LOW) {
                  rotation_result++;
                }
                switch (rotation_result) {
                  case B0010: //az_incremental_encoder_position++; break;
                  case B1011: //az_incremental_encoder_position++; break;
                  case B1101: //az_incremental_encoder_position++; break;
                  case B0100: az_incremental_encoder_position++; break;

                  case B0001: //az_incremental_encoder_position--; break;
                  case B0111: //az_incremental_encoder_position--; break;
                  case B1110: //az_incremental_encoder_position--; break;
                  case B1000: az_incremental_encoder_position--; break;
                }


                if (az_incremental_encoder_position > ((long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1) * 2)) {
                  az_incremental_encoder_position = 0;
                }
                if (az_incremental_encoder_position < 0) {
                  az_incremental_encoder_position = ((long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1) * 2);
                }

                #ifndef OPTION_SCANCON_2RMHF3600_INC_ENCODER
                if ((current_phase_a == LOW) && (current_phase_b == LOW) && (current_phase_z == LOW)) {
                  if ((az_incremental_encoder_position < long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) / 2)) || (az_incremental_encoder_position > long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) * 1.5))) {
                    az_incremental_encoder_position = AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
                  } else {
                    az_incremental_encoder_position = long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.);
                  }
                }
                #else
                if ((current_phase_a == HIGH) && (current_phase_b == HIGH) && (current_phase_z == HIGH)) {
                  if ((az_incremental_encoder_position < long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) / 2)) || (az_incremental_encoder_position > long((AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) * 1.5))) {
                    az_incremental_encoder_position = AZ_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
                  } else {
                    az_incremental_encoder_position = long(AZ_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.);
                  }
                }
                #endif //OPTION_SCANCON_2RMHF3600_INC_ENCODER
                az_3_phase_encoder_last_phase_a_state = current_phase_a;
                az_3_phase_encoder_last_phase_b_state = current_phase_b;

              }

              if (!read_azimuth_lock){
                read_azimuth(1);
                if(!service_rotation_lock){
                  service_rotation();
                }
              }



            } /* az_position_incremental_encoder_interrupt_handler */
            #endif // FEATURE_AZ_POSITION_INCREMENTAL_ENCODER


            #if defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)
            void el_position_incremental_encoder_interrupt_handler(){

              byte rotation_result = 0;
              byte current_phase_a = digitalReadEnhanced(el_incremental_encoder_pin_phase_a);
              byte current_phase_b = digitalReadEnhanced(el_incremental_encoder_pin_phase_b);
              byte current_phase_z = digitalReadEnhanced(el_incremental_encoder_pin_phase_z);

              #ifdef DEBUG_EL_POSITION_INCREMENTAL_ENCODER
              el_position_incremental_encoder_interrupt++;
              #endif // DEBUG_EL_POSITION_INCREMENTAL_ENCODER

              if ((el_3_phase_encoder_last_phase_a_state != current_phase_a) || (el_3_phase_encoder_last_phase_b_state != current_phase_b)) {
                if (el_3_phase_encoder_last_phase_a_state == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (el_3_phase_encoder_last_phase_b_state == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (current_phase_a == LOW) {
                  rotation_result++;
                }
                rotation_result = rotation_result << 1;
                if (current_phase_b == LOW) {
                  rotation_result++;
                }
                switch (rotation_result) {
                  case B0010: //el_incremental_encoder_position++; break;
                  case B1011: //el_incremental_encoder_position++; break;
                  case B1101: //el_incremental_encoder_position++; break;
                  case B0100: el_incremental_encoder_position++; break;

                  case B0001: //el_incremental_encoder_position--; break;
                  case B0111: //el_incremental_encoder_position--; break;
                  case B1110: //el_incremental_encoder_position--; break;
                  case B1000: el_incremental_encoder_position--; break;
                }


                #ifndef OPTION_SCANCON_2RMHF3600_INC_ENCODER
                if ((current_phase_a == LOW) && (current_phase_b == LOW) && (current_phase_z == LOW)) {
                  el_incremental_encoder_position = EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
                } else {

                  if (el_incremental_encoder_position < 0) {
                    el_incremental_encoder_position = int((EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1);
                  }

                  if (el_incremental_encoder_position >= int(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.)) {
                    el_incremental_encoder_position = 0;
                  }

                }
                #else
                if ((current_phase_a == HIGH) && (current_phase_b == HIGH) && (current_phase_z == HIGH)) {
                  el_incremental_encoder_position = EL_INCREMENTAL_ENCODER_ZERO_PULSE_POSITION;
                } else {
                  if (el_incremental_encoder_position < 0) {
                    el_incremental_encoder_position = int((EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.) - 1);
                  }
                  if (el_incremental_encoder_position >= int(EL_POSITION_INCREMENTAL_ENCODER_PULSES_PER_REV*4.)) {
                    el_incremental_encoder_position = 0;
                  }
                }
                #endif //OPTION_SCANCON_2RMHF3600_INC_ENCODER

                el_3_phase_encoder_last_phase_a_state = current_phase_a;
                el_3_phase_encoder_last_phase_b_state = current_phase_b;

              }

              if (!read_elevation_lock){
                read_elevation(1);
                if(!service_rotation_lock){
                  service_rotation();
                }
              }


            } /* el_position_incremental_encoder_interrupt_handler */
            #endif // defined(FEATURE_EL_POSITION_INCREMENTAL_ENCODER) && defined(FEATURE_ELEVATION_CONTROL)



            void pinModeEnhanced(uint8_t pin, uint8_t mode) {
              #if !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
              pinMode(pin, mode);
              #else
              if (pin < 100) {
                pinMode(pin, mode);
              } else {
                submit_remote_command(REMOTE_UNIT_DHL_COMMAND, pin, mode);
              }
              #endif // !defined(FEATURE_MASTER_WITH_SERIAL_SLAVE) && !defined(FEATURE_MASTER_WITH_ETHERNET_SLAVE)
            }

            void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue) {
              digitalWrite(pin, writevalue);
            }


            int digitalReadEnhanced(uint8_t pin) {

              return digitalRead(pin);

            }

            int analogReadEnhanced(uint8_t pin) {

              #ifdef OPTION_EXTERNAL_ANALOG_REFERENCE
              analogReference(EXTERNAL);
              #endif //OPTION_EXTERNAL_ANALOG_REFERENCE
              return analogRead(pin);

            }

            void analogWriteEnhanced(uint8_t pin, int writevalue) {
              analogWrite(pin, writevalue);
            }

            void port_flush() {
              #if defined(CONTROL_PORT_MAPPED_TO) && (defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_YAESU_EMULATION) || defined(FEATURE_EASYCOM_EMULATION))
              control_port->flush();
              #endif //CONTROL_PORT_MAPPED_TO

              #if defined(GPS_PORT_MAPPED_TO) && defined(FEATURE_GPS)
              gps_port->flush();
              #endif //defined(GPS_PORT_MAPPED_TO) && defined(FEATURE_GPS)
            }

            #ifdef FEATURE_POWER_SWITCH
            void service_power_switch(){

              static byte power_switch_state = 1;

              #ifdef FEATURE_ELEVATION_CONTROL
              if ((az_state != IDLE) || (el_state != IDLE)){
                last_activity_time = millis();
              }
              #else //FEATURE_ELEVATION_CONTROL
              if (az_state != IDLE){
                last_activity_time = millis();
              }
              #endif //FEATURE_ELEVATION_CONTROL


              if ((millis()-last_activity_time) > ((unsigned long)60000 * (unsigned long)POWER_SWITCH_IDLE_TIMEOUT)) {
                if (power_switch_state){
                  digitalWriteEnhanced(power_switch, LOW);
                  power_switch_state = 0;
                }
              } else {
                if (!power_switch_state){
                  digitalWriteEnhanced(power_switch, HIGH);
                  power_switch_state = 1;
                }
              }


            }
            #endif //FEATURE_POWER_SWITCH

            char *coordinates_to_maidenhead(float latitude_degrees,float longitude_degrees) {
              static char temp_string[8] = "";  // I had to declare this static in Arduino 1.6, otherwise this won't work (it worked before)

              latitude_degrees += 90.0;
              longitude_degrees += 180.0;

              temp_string[0] = (int(longitude_degrees/20)) + 65;
              temp_string[1] = (int(latitude_degrees/10)) + 65;
              temp_string[2] = (int((longitude_degrees - int(longitude_degrees/20)*20)/2)) + 48;
              temp_string[3] = (int(latitude_degrees - int(latitude_degrees/10)*10)) + 48;
              temp_string[4] = (int((longitude_degrees - (int(longitude_degrees/2)*2)) / (5.0/60.0))) + 97;
              temp_string[5] = (int((latitude_degrees - (int(latitude_degrees/1)*1)) / (2.5/60.0))) + 97;
              temp_string[6] = 0;

              return temp_string;
            }

            #ifdef FEATURE_ANALOG_OUTPUT_PINS
            void service_analog_output_pins() {
              static int last_azimith_voltage_out = 0;
              int azimuth_voltage_out = map(azimuth/HEADING_MULTIPLIER,0,360,0,255);
              if (last_azimith_voltage_out != azimuth_voltage_out){
                analogWriteEnhanced(pin_analog_az_out,azimuth_voltage_out);
                last_azimith_voltage_out = azimuth_voltage_out;
              }
              #ifdef FEATURE_ELEVATION_CONTROL
              static int last_elevation_voltage_out = 0;
              int elevation_voltage_out = map(elevation/HEADING_MULTIPLIER,0,ANALOG_OUTPUT_MAX_EL_DEGREES,0,255);
              if (last_elevation_voltage_out != elevation_voltage_out){
                analogWriteEnhanced(pin_analog_el_out,elevation_voltage_out);
                last_elevation_voltage_out = elevation_voltage_out;
              }
              #endif //FEATURE_ELEVATION_CONTROL
            }
            #endif //FEATURE_ANALOG_OUTPUT_PINS

            #ifdef FEATURE_AUTOCORRECT
            void submit_autocorrect(byte axis,float heading){

              #ifdef DEBUG_AUTOCORRECT
              debug.print("submit_autocorrect: ");
              #endif //DEBUG_AUTOCORRECT

              if (axis == AZ){
                autocorrect_state_az = AUTOCORRECT_WATCHING_AZ;
                autocorrect_az = heading;
                autocorrect_az_submit_time = millis();

                #ifdef DEBUG_AUTOCORRECT
                debug.print("AZ: ");
                #endif //DEBUG_AUTOCORRECT

              }


              #ifdef FEATURE_ELEVATION_CONTROL
              if (axis == EL){
                autocorrect_state_el = AUTOCORRECT_WATCHING_EL;
                autocorrect_el = heading;
                autocorrect_el_submit_time = millis();

                #ifdef DEBUG_AUTOCORRECT
                debug.print("EL: ");
                #endif //DEBUG_AUTOCORRECT

              }
              #endif //FEATURE_ELEVATION_CONTROL

              #ifdef DEBUG_AUTOCORRECT
              debug.print(heading,2);
              debug.println("");
              #endif //DEBUG_AUTOCORRECT

            }
            #endif //FEATURE_AUTOCORRECT

            byte get_analog_pin(byte pin_number){

              byte return_output = 0;
              switch (pin_number) {
                case 0: return_output = A0; break;
                case 1: return_output = A1; break;
                case 2: return_output = A2; break;
                case 3: return_output = A3; break;
                case 4: return_output = A4; break;
                case 5: return_output = A5; break;
                case 6: return_output = A6; break;
              }
              return return_output;
            }

            #if defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

            void update_sun_position() {

              update_time();
              c_time.iYear = clock_years;
              c_time.iMonth = clock_months;
              c_time.iDay = clock_days;

              c_time.dHours = clock_hours;
              c_time.dMinutes = clock_minutes;
              c_time.dSeconds = clock_seconds;

              c_loc.dLongitude = longitude;
              c_loc.dLatitude  = latitude;

              c_sposn.dZenithAngle = 0;
              c_sposn.dAzimuth = 0;

              sunpos(c_time, c_loc, &c_sposn);

              // Convert Zenith angle to elevation
              sun_elevation = 90. - c_sposn.dZenithAngle;
              sun_azimuth = c_sposn.dAzimuth;

            } /* update_sun_position */

            void update_moon_position() {
              update_time();
              double RA, Dec, topRA, topDec, LST, HA, dist;
              update_time();
              moon2(clock_years, clock_months, clock_days, (clock_hours + (clock_minutes / 60.0) + (clock_seconds / 3600.0)), longitude, latitude, &RA, &Dec, &topRA, &topDec, &LST, &HA, &moon_azimuth, &moon_elevation, &dist);
            }

            byte calibrate_az_el(float new_az, float new_el) {
              #ifdef DEBUG_OFFSET
              debug.print("calibrate_az_el: new_az:");
              debug.print(new_az, 2);
              debug.print(" new_el:");
              control_port->println(new_el, 2);
              #endif // DEBUG_OFFSET

              if ((new_az >= 0 ) && (new_az <= 360) && (new_el >= 0) && (new_el <= 90)) {
                configuration.azimuth_offset = 0;
                configuration.elevation_offset = 0;
                read_azimuth(1);
                read_elevation(1);

                #ifdef DEBUG_OFFSET
                debug.print("calibrate_az_el: az:");
                debug.print(azimuth / LCD_HEADING_MULTIPLIER, 2);
                debug.print(" el:");
                control_port->println(elevation / LCD_HEADING_MULTIPLIER, 2);
                #endif // DEBUG_OFFSET


                configuration.azimuth_offset = new_az - (float(raw_azimuth) / float(HEADING_MULTIPLIER));
                #if defined(FEATURE_ELEVATION_CONTROL)
                configuration.elevation_offset = new_el - (float(elevation) / float(HEADING_MULTIPLIER));
                #endif
                configuration_dirty = 1;
                return 1;
              } else {
                return 0;
              }

            } /* calibrate_az_el */

            char * az_el_calibrated_string(){

              char return_string[48] = "";
              char tempstring[16] = "";

              read_azimuth(1);
              read_elevation(1);
              strcpy(return_string, "Heading calibrated.  Az: ");
              dtostrf((azimuth / LCD_HEADING_MULTIPLIER), 0, LCD_DECIMAL_PLACES, tempstring);
              strcat(return_string, tempstring);
              #ifdef FEATURE_ELEVATION_CONTROL
              strcat(return_string, " El: ");
              dtostrf((elevation / LCD_HEADING_MULTIPLIER), 0, LCD_DECIMAL_PLACES, tempstring);
              strcat(return_string, tempstring);
              #endif //FEATURE_ELEVATION_CONTROL
              return return_string;

            }
            #endif // defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)

            #ifdef FEATURE_CLOCK
            char * clock_status_string(){

              switch (clock_status) {
                case FREE_RUNNING: return("FREE_RUNNING"); break;
                case GPS_SYNC: return("GPS_SYNC"); break;
                case RTC_SYNC: return("RTC_SYNC"); break;
                case SLAVE_SYNC: return("SLAVE_SYNC"); break;
                case SLAVE_SYNC_GPS: return("SLAVE_SYNC_GPS"); break;
              }
            }
            char * timezone_modified_clock_string() {

              static char return_string[32] = "";
              char temp_string[16] = "";

              dtostrf(local_clock_years, 0, 0, temp_string);
              strcpy(return_string, temp_string);
              strcat(return_string, "-");
              if (local_clock_months < 10) {
                strcat(return_string, "0");
              }
              dtostrf(local_clock_months, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, "-");
              if (local_clock_days < 10) {
                strcat(return_string, "0");
              }
              dtostrf(local_clock_days, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, " ");

              if (local_clock_hours < 10) {
                strcat(return_string, "0");
              }
              dtostrf(local_clock_hours, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, ":");
              if (local_clock_minutes < 10) {
                strcat(return_string, "0");
              }
              dtostrf(local_clock_minutes, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, ":");
              if (local_clock_seconds < 10) {
                strcat(return_string, "0");
              }
              dtostrf(local_clock_seconds, 0, 0, temp_string);
              strcat(return_string, temp_string);
              if (configuration.clock_timezone_offset == 0){
                strcat(return_string,"Z");
              }
              return return_string;

            } /* clock_string */

            char * zulu_clock_string() {

              static char return_string[32] = "";
              char temp_string[16] = "";

              dtostrf(clock_years, 0, 0, temp_string);
              strcpy(return_string, temp_string);
              strcat(return_string, "-");
              if (clock_months < 10) {
                strcat(return_string, "0");
              }
              dtostrf(clock_months, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, "-");
              if (clock_days < 10) {
                strcat(return_string, "0");
              }
              dtostrf(clock_days, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, " ");

              if (clock_hours < 10) {
                strcat(return_string, "0");
              }
              dtostrf(clock_hours, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, ":");
              if (clock_minutes < 10) {
                strcat(return_string, "0");
              }
              dtostrf(clock_minutes, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string, ":");
              if (clock_seconds < 10) {
                strcat(return_string, "0");
              }
              dtostrf(clock_seconds, 0, 0, temp_string);
              strcat(return_string, temp_string);
              strcat(return_string,"Z");
              return return_string;

            } /* zulu_clock_string */

            void update_time() {
              unsigned long runtime = millis() - millis_at_last_calibration;
              // calculate UTC

              unsigned long time = (3600L * clock_hour_set) + (60L * clock_min_set) + clock_sec_set + ((runtime + (runtime * INTERNAL_CLOCK_CORRECTION)) / 1000.0);

              clock_years = clock_year_set;
              clock_months = clock_month_set;
              clock_days = time / 86400L;
              time -= clock_days * 86400L;
              clock_days += clock_day_set;
              clock_hours = time / 3600L;

              switch (clock_months) {

                case 1:
                case 3:
                case 5:
                case 7:
                case 8:
                case 10:
                case 12:
                if (clock_days > 31) {
                  clock_days = 1; clock_months++;
                }
                break;

                case 2:
                if ((float(clock_years) / 4.0) == 0.0) {  // do we have a leap year?
                  if (clock_days > 29) {
                    clock_days = 1; clock_months++;
                  }
                } else {
                  if (clock_days > 28) {
                    clock_days = 1; clock_months++;
                  }
                }
                break;

                case 4:
                case 6:
                case 9:
                case 11:
                if (clock_days > 30) {
                  clock_days = 1; clock_months++;
                }
                break;
              } /* switch */

              if (clock_months > 12) {
                clock_months = 1; clock_years++;
              }

              time -= clock_hours * 3600L;
              clock_minutes  = time / 60L;
              time -= clock_minutes * 60L;
              clock_seconds = time;


              // calculate local time

              long local_time = (configuration.clock_timezone_offset * 60L * 60L) + (3600L * clock_hour_set) + (60L * clock_min_set) + clock_sec_set + ((runtime + (runtime * INTERNAL_CLOCK_CORRECTION)) / 1000.0);

              local_clock_years = clock_year_set;
              local_clock_months = clock_month_set;
              local_clock_days = clock_day_set;

              if (local_time < 0){
                local_time = local_time + (24L * 60L * 60L) - 1;
                local_clock_days--;
                if (local_clock_days < 1){
                  local_clock_months--;
                  switch (local_clock_months) {
                    case 0:
                    local_clock_months = 12;
                    local_clock_days = 31;
                    local_clock_years--;
                    break;
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                    case 8:
                    case 10:
                    case 12:
                    local_clock_days = 31;
                    break;
                    case 2: //February
                    if ((float(local_clock_years) / 4.0) == 0.0) {  // do we have a leap year?
                      local_clock_days = 29;
                    } else {
                      local_clock_days = 28;
                    }
                    break;
                    case 4:
                    case 6:
                    case 9:
                    case 11:
                    local_clock_days = 30;
                    break;
                  } /* switch */
                }
                local_clock_hours = local_time / 3600L;
                local_time -= local_clock_hours * 3600L;
                local_clock_minutes  = local_time / 60L;
                local_time -= local_clock_minutes * 60L;
                local_clock_seconds = local_time;

              } else {  //(local_time < 0)

                local_clock_days = local_time / 86400L;
                local_time -= local_clock_days * 86400L;
                local_clock_days += clock_day_set;
                local_clock_hours = local_time / 3600L;

                switch (local_clock_months) {

                  case 1:
                  case 3:
                  case 5:
                  case 7:
                  case 8:
                  case 10:
                  case 12:
                  if (local_clock_days > 31) {
                    local_clock_days = 1;
                    local_clock_months++;
                  }
                  break;

                  case 2:
                  if ((float(local_clock_years) / 4.0) == 0.0) {  // do we have a leap year?
                    if (local_clock_days > 29) {
                      local_clock_days = 1;
                      local_clock_months++;
                    }
                  } else {
                    if (local_clock_days > 28) {
                      local_clock_days = 1;
                      local_clock_months++;
                    }
                  }
                  break;

                  case 4:
                  case 6:
                  case 9:
                  case 11:
                  if (local_clock_days > 30) {
                    local_clock_days = 1;
                    local_clock_months++;
                  }
                  break;
                } /* switch */

                if (local_clock_months > 12) {
                  local_clock_months = 1;
                  local_clock_years++;
                }

                local_time -= local_clock_hours * 3600L;
                local_clock_minutes  = local_time / 60L;
                local_time -= local_clock_minutes * 60L;
                local_clock_seconds = local_time;


              }  //(local_time < 0)

            } /* update_time */
            #endif // FEATURE_CLOCK

            #ifdef FEATURE_GPS
            void service_gps(){

              long gps_lat, gps_lon;
              unsigned long fix_age;
              int gps_year;
              byte gps_month, gps_day, gps_hours, gps_minutes, gps_seconds, gps_hundredths;
              static byte gps_sync_pin_active = 0;
              #ifdef DEBUG_GPS
              char tempstring[10] = "";
              #endif //#ifdef DEBUG_GPS

              static unsigned long last_sync = 0;

              if (gps_data_available) {
                // retrieves +/- lat/long in 100000ths of a degree
                gps.get_position(&gps_lat, &gps_lon, &fix_age);
                gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hours, &gps_minutes, &gps_seconds, &gps_hundredths, &fix_age);
                #ifdef DEBUG_GPS
                #if defined(DEBUG_GPS_SERIAL)
                debug.println("");
                #endif //DEBUG_GPS_SERIAL
                debug.print("service_gps: fix_age:");
                debug.print(fix_age);
                debug.print(" lat:");
                debug.print(gps_lat,4);
                debug.print(" long:");
                debug.print(gps_lon,4);
                debug.print(" ");
                debug.print(gps_year);
                debug.print("-");
                debug.print(gps_month);
                debug.print("-");
                debug.print(gps_day);
                debug.print(" ");
                debug.print(gps_hours);
                debug.print(":");
                debug.print(gps_minutes);
                debug.println("");
                #endif // DEBUG_GPS

                if (fix_age < GPS_VALID_FIX_AGE_MS) {

                  if (SYNC_TIME_WITH_GPS) {
                    clock_year_set = gps_year;
                    clock_month_set = gps_month;
                    clock_day_set = gps_day;
                    clock_hour_set = gps_hours;
                    clock_min_set = gps_minutes;
                    clock_sec_set = gps_seconds;
                    millis_at_last_calibration = millis() - GPS_UPDATE_LATENCY_COMPENSATION_MS;
                    update_time();
                    #ifdef DEBUG_GPS
                    #ifdef DEBUG_GPS_SERIAL
                    debug.println("");
                    #endif //DEBUG_GPS_SERIAL
                    debug.print("service_gps: clock sync:");
                    sprintf(tempstring,"%s",timezone_modified_clock_string());
                    debug.print(tempstring);
                    debug.println("");
                    #endif // DEBUG_GPS
                  }

                  #if defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_DS1307)
                  static unsigned long last_rtc_gps_sync_time;
                  if ((millis() - last_rtc_gps_sync_time) >= ((unsigned long)SYNC_RTC_TO_GPS_SECONDS * 1000)) {
                    rtc.adjust(DateTime(gps_year, gps_month, gps_day, gps_hours, gps_minutes, gps_seconds));
                    #ifdef DEBUG_RTC
                    debug.println("service_gps: synced RTC");
                    #endif // DEBUG_RTC
                    last_rtc_gps_sync_time = millis();
                  }
                  #endif // defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_DS1307)

                  #if defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_PCF8583)
                  static unsigned long last_rtc_gps_sync_time;
                  if ((millis() - last_rtc_gps_sync_time) >= ((unsigned long)SYNC_RTC_TO_GPS_SECONDS * 1000)) {
                    rtc.year = gps_year;
                    rtc.month = gps_month;
                    rtc.day = gps_day;
                    rtc.hour  = gps_hours;
                    rtc.minute = gps_minutes;
                    rtc.second = gps_seconds;
                    rtc.set_time();
                    #ifdef DEBUG_RTC
                    debug.println("service_gps: synced RTC");
                    #endif // DEBUG_RTC
                    last_rtc_gps_sync_time = millis();
                  }
                  #endif // defined(OPTION_SYNC_RTC_TO_GPS) && defined(FEATURE_RTC_PCF8583)


                  if (SYNC_COORDINATES_WITH_GPS) {
                    latitude = float(gps_lat) / 1000000.0;
                    longitude = float(gps_lon) / 1000000.0;
                    #ifdef DEBUG_GPS
                    debug.print("service_gps: coord sync:");
                    debug.print(latitude,2);
                    debug.print(" ");
                    debug.print(longitude,2);
                    debug.println("");
                    #endif // DEBUG_GPS
                  }

                  last_sync = millis();
                }

                gps_data_available = 0;
              }

              if ((millis() > ((unsigned long)GPS_SYNC_PERIOD_SECONDS * 1000)) && ((millis() - last_sync) < ((unsigned long)GPS_SYNC_PERIOD_SECONDS * 1000)) && (SYNC_TIME_WITH_GPS)) {
                clock_status = GPS_SYNC;
              } else {
                clock_status = FREE_RUNNING;
              }

              if (gps_sync){
                if (clock_status == GPS_SYNC){
                  if (!gps_sync_pin_active){
                    digitalWriteEnhanced(gps_sync,HIGH);
                    gps_sync_pin_active = 1;
                  }
                } else {
                  if (gps_sync_pin_active){
                    digitalWriteEnhanced(gps_sync,LOW);
                    gps_sync_pin_active = 0;
                  }
                }
              }


            } /* service_gps */
            #endif // FEATURE_GPS

            #ifdef FEATURE_RTC
            void service_rtc() {

              static unsigned long last_rtc_sync_time = 0;

              if (((millis() - last_rtc_sync_time) >= ((unsigned long)SYNC_WITH_RTC_SECONDS * 1000)) || ((clock_status == FREE_RUNNING) && (millis() - last_rtc_sync_time) > 1000)) {
                last_rtc_sync_time = millis();
                #ifdef FEATURE_GPS
                if (clock_status == GPS_SYNC) { // if we're also equipped with GPS and we're synced to it, don't sync to realtime clock
                #ifdef DEBUG_RTC
                debug.println("service_rtc: synced to GPS already.  Exiting.");
                #endif // DEBUG_RTC
                return;
              }
              #endif // FEATURE_GPS


              #ifdef FEATURE_RTC_DS1307
              if (rtc.isrunning()) {
                DateTime now = rtc.now();
                #ifdef DEBUG_RTC
                debug.print("service_rtc: syncing: ");
                debug.print(now.year());
                debug.print("/");
                debug.print(now.month());
                debug.print("/");
                debug.print(now.day());
                debug.print(" ");
                debug.print(now.hour());
                debug.print(":");
                debug.print(now.minute());
                debug.print(":");
                debug.print(now.second());
                debug.println("");
                #endif // DEBUG_RTC
                clock_year_set = now.year();
                clock_month_set = now.month();
                clock_day_set = now.day();
                clock_hour_set = now.hour();
                clock_min_set = now.minute();
                clock_sec_set = now.second();
                millis_at_last_calibration = millis();
                update_time();
                clock_status = RTC_SYNC;
              } else {
                clock_status = FREE_RUNNING;
                #ifdef DEBUG_RTC
                debug.println("service_rtc: error: RTC not running");
                #endif // DEBUG_RTC
              }
              #endif //#FEATURE_RTC_DS1307



              #ifdef FEATURE_RTC_PCF8583
              rtc.get_time();
              if ((rtc.year > 2000) && (rtc.month > 0) && (rtc.month < 13)){  // do we have a halfway reasonable date?
                #ifdef DEBUG_RTC
                control_port->print("service_rtc: syncing: ");
                control_port->print(rtc.year, DEC);
                control_port->print('/');
                control_port->print(rtc.month, DEC);
                control_port->print('/');
                control_port->print(rtc.day, DEC);
                control_port->print(' ');
                control_port->print(rtc.hour, DEC);
                control_port->print(':');
                control_port->print(rtc.minute, DEC);
                control_port->print(':');
                control_port->println(rtc.second, DEC);
                #endif // DEBUG_RTC
                clock_year_set = rtc.year;
                clock_month_set = rtc.month;
                clock_day_set = rtc.day;
                clock_hour_set = rtc.hour;
                clock_min_set = rtc.minute;
                clock_sec_set = rtc.second;
                millis_at_last_calibration = millis();
                update_time();
                clock_status = RTC_SYNC;
              } else {
                clock_status = FREE_RUNNING;
                #ifdef DEBUG_RTC
                control_port->print("service_rtc: error: RTC not returning valid date or time: ");
                control_port->print(rtc.year, DEC);
                control_port->print('/');
                control_port->print(rtc.month, DEC);
                control_port->print('/');
                control_port->print(rtc.day, DEC);
                control_port->print(' ');
                control_port->print(rtc.hour, DEC);
                control_port->print(':');
                control_port->print(rtc.minute, DEC);
                control_port->print(':');
                control_port->println(rtc.second, DEC);
                #endif // DEBUG_RTC
              }
              #endif //#FEATURE_RTC_PCF8583



            }
          } /* service_rtc */
          #endif // FEATURE_RTC

          #ifdef FEATURE_YAESU_EMULATION
          void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string) {

            char tempstring[11] = "";
            int parsed_value = 0;
            int parsed_elevation = 0;

            #ifdef FEATURE_TIMED_BUFFER
            int parsed_value2 = 0;
            #endif //FEATURE_TIMED_BUFFER

            strcpy(return_string,"");

            switch (yaesu_command_buffer[0]) {          // look at the first character of the command
              case 'C':                                // C - return current azimuth
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: C\n");
              }
              #endif // DEBUG_PROCESS_YAESU
              #ifdef OPTION_DELAY_C_CMD_OUTPUT
              delay(400);
              #endif
              //strcpy(return_string,"");
              #ifndef OPTION_GS_232B_EMULATION
              strcat(return_string,"+0");
              #else
              strcat(return_string,"AZ=");
              #endif
              dtostrf(int(azimuth / HEADING_MULTIPLIER),0,0,tempstring);
              if (int(azimuth / HEADING_MULTIPLIER) < 10) {
                strcat(return_string,"0");
              }
              if (int(azimuth / HEADING_MULTIPLIER) < 100) {
                strcat(return_string,"0");
              }
              strcat(return_string,tempstring);

              #ifdef FEATURE_ELEVATION_CONTROL
              #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
              if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
                #endif


                #ifndef OPTION_GS_232B_EMULATION
                if (elevation < 0) {
                  strcat(return_string,"-0");
                } else {
                  strcat(return_string,"+0");
                }
                #endif
                #ifdef OPTION_GS_232B_EMULATION
                strcat(return_string,"EL=");
                #endif
                dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
                if (int(elevation / HEADING_MULTIPLIER) < 10) {
                  strcat(return_string,("0"));
                }
                if (int(elevation / HEADING_MULTIPLIER) < 100) {
                  strcat(return_string,"0");
                }
                strcat(return_string,tempstring);

                #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
              } else {
                //strcat(return_string,"\n");
              }
              #endif // OPTION_C_COMMAND_SENDS_AZ_AND_EL
              #endif // FEATURE_ELEVATION_CONTROL

              #ifndef FEATURE_ELEVATION_CONTROL
              if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
                #ifndef OPTION_GS_232B_EMULATION
                strcat(return_string,"+0000");    // return a dummy elevation since we don't have the elevation feature turned on
                #else
                strcat(return_string,"EL=000");
                #endif
              } else {
                //strcat(return_string,"\n");
              }
              #endif // FEATURE_ELEVATION_CONTROL
              break;


              //-----------------end of C command-----------------

              #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
              case 'F': // F - full scale calibration
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: F\n");
              }
              #endif // DEBUG_PROCESS_YAESU


              #ifdef FEATURE_ELEVATION_CONTROL
              if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the F2 command?

                clear_serial_buffer();
                if (source_port == CONTROL_PORT0){
                  control_port->println(F("Elevate to 180 (or max elevation) and send keystroke..."));
                }
                get_keystroke();
                read_elevation(1);
                configuration.analog_el_max_elevation = analog_el;
                write_settings_to_eeprom();
                strcpy(return_string,"Wrote to memory");
                return;
              }
              #endif

              clear_serial_buffer();
              if (source_port == CONTROL_PORT0){
                control_port->println(F("Rotate to full CW and send keystroke..."));
                get_keystroke();
              }
              read_azimuth(1);
              configuration.analog_az_full_cw = analog_az;
              write_settings_to_eeprom();
              strcpy(return_string,"Wrote to memory");
              break;
              #endif // FEATURE_AZ_POSITION_POTENTIOMETER
              case 'H': print_help(source_port); break;                     // H - print help - depricated
              case 'L':  // L - manual left (CCW) rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: L\n");
              }
              #endif // DEBUG_PROCESS_YAESU
              submit_request(AZ, REQUEST_CCW, 0, 21);
              //strcpy(return_string,"\n");
              break;

              #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
              case 'O':  // O - offset calibration
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: O\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              #ifdef FEATURE_ELEVATION_CONTROL
              if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the O2 command?
                clear_serial_buffer();
                if (source_port == CONTROL_PORT0){
                  control_port->println(F("Elevate to 0 degrees and send keystroke..."));
                }
                get_keystroke();
                read_elevation(1);
                configuration.analog_el_0_degrees = analog_el;
                write_settings_to_eeprom();
                strcpy(return_string,"Wrote to memory");
                return;
              }
              #endif

              clear_serial_buffer();
              if (source_port == CONTROL_PORT0){
                control_port->println(F("Rotate to full CCW and send keystroke..."));
              }
              get_keystroke();
              read_azimuth(1);
              configuration.analog_az_full_ccw = analog_az;
              write_settings_to_eeprom();
              strcpy(return_string,"Wrote to memory");
              break;
              #endif // FEATURE_AZ_POSITION_POTENTIOMETER

              case 'R':  // R - manual right (CW) rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: R\n");
              }
              #endif // DEBUG_PROCESS_YAESU
              submit_request(AZ, REQUEST_CW, 0, 22);
              strcpy(return_string,"\n");
              break;

              case 'A':  // A - CW/CCW rotation stop
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: A\n");
              }
              #endif // DEBUG_PROCESS_YAESU
              submit_request(AZ, REQUEST_STOP, 0, 23);
              //strcpy(return_string,"\n");
              break;

              case 'S':         // S - all stop
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: S\n");
              }
              #endif // DEBUG_PROCESS_YAESU
              submit_request(AZ, REQUEST_STOP, 0, 24);
              #ifdef FEATURE_ELEVATION_CONTROL
              submit_request(EL, REQUEST_STOP, 0, 25);
              #endif
              #ifdef FEATURE_TIMED_BUFFER
              clear_timed_buffer();
              #endif // FEATURE_TIMED_BUFFER
              //strcpy(return_string,"");
              break;

              case 'M': // M - auto azimuth rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: M\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              if (yaesu_command_buffer_index > 4) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
                #ifdef FEATURE_TIMED_BUFFER
                clear_timed_buffer();
                parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
                if ((parsed_value > 0) && (parsed_value < 1000)) {
                  timed_buffer_interval_value_seconds = parsed_value;
                  for (int x = 5; x < yaesu_command_buffer_index; x = x + 4) {
                    parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
                    if ((parsed_value >= 0) && (parsed_value <= (azimuth_starting_point + azimuth_rotation_capability))) {  // is it a valid azimuth?
                      timed_buffer_azimuths[timed_buffer_number_entries_loaded] = parsed_value * HEADING_MULTIPLIER;
                      timed_buffer_number_entries_loaded++;
                      timed_buffer_status = LOADED_AZIMUTHS;
                      if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                        submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 26);  // array is full, go to the first azimuth
                        timed_buffer_entry_pointer = 1;
                        return;
                      }
                    } else {   // we hit an invalid bearing
                      timed_buffer_status = EMPTY;
                      timed_buffer_number_entries_loaded = 0;
                      strcpy(return_string,"?>");  // error
                      return;
                    }
                  }
                  submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[0], 27);   // go to the first azimuth
                  timed_buffer_entry_pointer = 1;
                } else {
                  strcpy(return_string,"?>");  // error
                }
                #else
                strcpy(return_string,"?>");
                #endif // FEATURE_TIMED_BUFFER
                return;
              } else {                         // if there are four characters, this is just a single direction setting
                if (yaesu_command_buffer_index == 4) {
                  parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
                  #ifdef FEATURE_TIMED_BUFFER
                  clear_timed_buffer();
                  #endif // FEATURE_TIMED_BUFFER
                  if ((parsed_value >= 0) && (parsed_value <= (azimuth_starting_point + azimuth_rotation_capability))) {
                    submit_request(AZ, REQUEST_AZIMUTH, (parsed_value * HEADING_MULTIPLIER), 28);
                    return;
                  }
                }
              }
              strcpy(return_string,"?>");
              break;

              #ifdef FEATURE_TIMED_BUFFER
              case 'N': // N - number of loaded timed interval entries
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: N\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              sprintf(return_string,"%d",timed_buffer_number_entries_loaded);
              break;
              #endif // FEATURE_TIMED_BUFFER

              #ifdef FEATURE_TIMED_BUFFER
              case 'T': // T - initiate timed tracking
              initiate_timed_buffer(source_port);
              break;
              #endif // FEATURE_TIMED_BUFFER

              case 'X':  // X - azimuth speed change
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: X\n");
              }
              #endif // DEBUG_PROCESS_YAESU


              if (yaesu_command_buffer_index > 1) {
                switch (yaesu_command_buffer[1]) {
                  case '4':
                  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
                  update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
                  #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
                  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
                  update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);
                  #endif
                  strcpy(return_string,"Speed X4");
                  break;
                  case '3':
                  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;
                  update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
                  #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
                  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;
                  update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);
                  #endif
                  strcpy(return_string,"Speed X3");
                  break;
                  case '2':
                  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;
                  update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);
                  #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
                  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;
                  update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);
                  #endif
                  strcpy(return_string,"Speed X2");
                  break;
                  case '1':
                  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
                  update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1);
                  #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
                  normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;
                  update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);
                  #endif
                  strcpy(return_string,"Speed X1");
                  break;
                  default: strcpy(return_string,"?>"); break;
                } /* switch */
              } else {
                strcpy(return_string,"?>");
              }
              break;

              #ifdef FEATURE_ELEVATION_CONTROL
              case 'U':  // U - manual up rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: U\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              submit_request(EL, REQUEST_UP, 0, 29);
              //strcpy(return_string,"\n");
              break;

              case 'D':  // D - manual down rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: D\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              submit_request(EL, REQUEST_DOWN, 0, 30);
              //strcpy(return_string,"\n");
              break;

              case 'E':  // E - stop elevation rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: E\n");
              }
              #endif // DEBUG_PROCESS_YAESU

              submit_request(EL, REQUEST_STOP, 0, 31);
              //strcpy(return_string,"\n");
              break;

              case 'B': // B - return current elevation
              #ifndef OPTION_GS_232B_EMULATION
              if (elevation < 0) {
                strcat(return_string,"-0");
              } else {
                strcat(return_string,"+0");
              }
              #else
              strcat(return_string,"EL=");
              #endif //OPTION_GS_232B_EMULATION
              dtostrf(int(elevation / HEADING_MULTIPLIER),0,0,tempstring);
              if (int(elevation / HEADING_MULTIPLIER) < 10) {
                strcat(return_string,("0"));
              }
              if (int(elevation / HEADING_MULTIPLIER) < 100) {
                strcat(return_string,"0");
              }
              strcat(return_string,tempstring);
              break;

              #endif /* ifdef FEATURE_ELEVATION_CONTROL */

              case 'W':  // W - auto elevation rotation
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("yaesu_serial_command: W\n");
              }
              #endif // DEBUG_PROCESS_YAESU


              // parse out W command
              // Short Format: WXXX YYY            XXX = azimuth YYY = elevation
              // Long Format : WSSS XXX YYY        SSS = timed interval   XXX = azimuth    YYY = elevation

              if (yaesu_command_buffer_index > 8) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
                #if defined(FEATURE_TIMED_BUFFER) && defined(FEATURE_ELEVATION_CONTROL)
                parsed_value = ((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48);
                if ((parsed_value > 0) && (parsed_value < 1000)) {
                  timed_buffer_interval_value_seconds = parsed_value;
                  for (int x = 5; x < yaesu_command_buffer_index; x = x + 8) {
                    parsed_value = ((int(yaesu_command_buffer[x]) - 48) * 100) + ((int(yaesu_command_buffer[x + 1]) - 48) * 10) + (int(yaesu_command_buffer[x + 2]) - 48);
                    parsed_value2 = ((int(yaesu_command_buffer[x + 4]) - 48) * 100) + ((int(yaesu_command_buffer[x + 5]) - 48) * 10) + (int(yaesu_command_buffer[x + 6]) - 48);
                    if ((parsed_value > -1) && (parsed_value < 361) && (parsed_value2 > -1) && (parsed_value2 < 181)) {  // is it a valid azimuth?
                      timed_buffer_azimuths[timed_buffer_number_entries_loaded] = (parsed_value * HEADING_MULTIPLIER);
                      timed_buffer_elevations[timed_buffer_number_entries_loaded] = (parsed_value2 * HEADING_MULTIPLIER);
                      timed_buffer_number_entries_loaded++;
                      timed_buffer_status = LOADED_AZIMUTHS_ELEVATIONS;
                      if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
                        x = yaesu_command_buffer_index;  // array is full, go to the first azimuth and elevation

                      }
                    } else {   // we hit an invalid bearing
                      timed_buffer_status = EMPTY;
                      timed_buffer_number_entries_loaded = 0;
                      strcpy(return_string,"?>");  // error
                      return;
                    }
                  }
                }
                timed_buffer_entry_pointer = 1;             // go to the first bearings
                parsed_value = timed_buffer_azimuths[0];
                parsed_elevation = timed_buffer_elevations[0];
                #else /* ifdef FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL*/
                strcpy(return_string,"?>");
                #endif // FEATURE_TIMED_BUFFER FEATURE_ELEVATION_CONTROL
              } else {
                // this is a short form W command, just parse the azimuth and elevation and initiate rotation
                parsed_value = (((int(yaesu_command_buffer[1]) - 48) * 100) + ((int(yaesu_command_buffer[2]) - 48) * 10) + (int(yaesu_command_buffer[3]) - 48)) * HEADING_MULTIPLIER;
                parsed_elevation = (((int(yaesu_command_buffer[5]) - 48) * 100) + ((int(yaesu_command_buffer[6]) - 48) * 10) + (int(yaesu_command_buffer[7]) - 48)) * HEADING_MULTIPLIER;
              }

              #ifndef FEATURE_ELEVATION_CONTROL

              if ((parsed_value >= 0) && (parsed_value <= ((azimuth_starting_point + azimuth_rotation_capability)* HEADING_MULTIPLIER))) {
                //if ((parsed_value >= 0) && (parsed_value <= (360 * HEADING_MULTIPLIER))) {
                submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 32);
              } else {
                #ifdef DEBUG_PROCESS_YAESU
                if (debug_mode) {
                  debug.print("process_yaesu_command: W cmd az error");
                }
                #endif // DEBUG_PROCESS_YAESU
                strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
              }

              #else
              if ((parsed_value >= 0) && (parsed_value <= ((azimuth_starting_point + azimuth_rotation_capability)* HEADING_MULTIPLIER)) && (parsed_elevation >= 0) && (parsed_elevation <= (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER))) {

                //if ((parsed_value >= 0) && (parsed_value <= (360 * HEADING_MULTIPLIER)) && (parsed_elevation >= 0) && (parsed_elevation <= (180 * HEADING_MULTIPLIER))) {
                submit_request(AZ, REQUEST_AZIMUTH, parsed_value, 33);
                submit_request(EL, REQUEST_ELEVATION, parsed_elevation, 34);
              } else {
                #ifdef DEBUG_PROCESS_YAESU
                if (debug_mode) {
                  debug.print("process_yaesu_command: W cmd az/el error");
                }
                #endif // DEBUG_PROCESS_YAESU
                strcpy(return_string,"?>");      // bogus elevation - return and error and don't do anything
              }
              #endif // FEATURE_ELEVATION_CONTROL


              break;

              #ifdef OPTION_GS_232B_EMULATION
              case 'P':  // P - switch between 360 and 450 degree mode

              if ((yaesu_command_buffer[1] == '3') && (yaesu_command_buffer_index > 2)) {  // P36 command
                azimuth_rotation_capability = 360;
                strcpy(return_string,"Mode 360 degree");
                // write_settings_to_eeprom();
              } else {
                if ((yaesu_command_buffer[1] == '4') && (yaesu_command_buffer_index > 2)) { // P45 command
                  azimuth_rotation_capability = 450;
                  strcpy(return_string,"Mode 450 degree");
                  // write_settings_to_eeprom();
                } else {
                  strcpy(return_string,"?>");
                }
              }


              break;
              case 'Z':                                           // Z - Starting point toggle

              if (azimuth_starting_point == 180) {
                azimuth_starting_point = 0;
                strcpy(return_string,"N");
              } else {
                azimuth_starting_point = 180;
                strcpy(return_string,"S");
              }
              strcat(return_string," Center");
              // write_settings_to_eeprom();
              break;
              #endif

              default:
              strcpy(return_string,"?>");
              #ifdef DEBUG_PROCESS_YAESU
              if (debug_mode) {
                debug.print("process_yaesu_command: yaesu_command_buffer_index: ");
                debug.print(yaesu_command_buffer_index);
                for (int debug_x = 0; debug_x < yaesu_command_buffer_index; debug_x++) {
                  debug.print("process_yaesu_command: yaesu_command_buffer[");
                  debug.print(debug_x);
                  debug.print("]: ");
                  debug.print(yaesu_command_buffer[debug_x]);
                  debug.print(" ");
                  debug.write(yaesu_command_buffer[debug_x]);
                  debug.print("\n");;
                }
              }
              #endif // DEBUG_PROCESS_YAESU
            } /* switch */

          } /* yaesu_serial_command */
          #endif // FEATURE_YAESU_EMULATION

          #ifdef FEATURE_MOON_TRACKING
          void service_moon_tracking() {

            static unsigned long last_check = 0;
            static byte moon_tracking_activated_by_activate_line = 0;

            static byte moon_tracking_pin_state = 0;

            if (moon_tracking_active_pin) {
              if ((moon_tracking_active) && (!moon_tracking_pin_state)) {
                digitalWriteEnhanced(moon_tracking_active_pin, HIGH);
                moon_tracking_pin_state = 1;
              }
              if ((!moon_tracking_active) && (moon_tracking_pin_state)) {
                digitalWriteEnhanced(moon_tracking_active_pin, LOW);
                moon_tracking_pin_state = 0;
              }
            }

            if (moon_tracking_activate_line) {
              if ((!moon_tracking_active) && (!digitalReadEnhanced(moon_tracking_activate_line))) {
                moon_tracking_active = 1;
                moon_tracking_activated_by_activate_line = 1;
              }
              if ((moon_tracking_active) && (digitalReadEnhanced(moon_tracking_activate_line)) && (moon_tracking_activated_by_activate_line)) {
                moon_tracking_active = 0;
                moon_tracking_activated_by_activate_line = 0;
              }
            }

            if ((moon_tracking_active) && ((millis() - last_check) > MOON_TRACKING_CHECK_INTERVAL)) {

              update_time();
              update_moon_position();

              #ifdef DEBUG_MOON_TRACKING
              debug.print(F("service_moon_tracking: AZ: "));
              debug.print(moon_azimuth);
              debug.print(" EL: ");
              debug.print(moon_elevation);
              debug.print(" lat: ");
              debug.print(latitude);
              debug.print(" long: ");
              debug.println(longitude);
              #endif // DEBUG_MOON_TRACKING

              if ((moon_azimuth >= MOON_AOS_AZIMUTH_MIN) && (moon_azimuth <= MOON_AOS_AZIMUTH_MAX) && (moon_elevation >= MOON_AOS_ELEVATION_MIN) && (moon_elevation <= MOON_AOS_ELEVATION_MAX)) {
                submit_request(AZ, REQUEST_AZIMUTH, moon_azimuth * HEADING_MULTIPLIER, 11);
                submit_request(EL, REQUEST_ELEVATION, moon_elevation * HEADING_MULTIPLIER, 12);
                if (!moon_visible) {
                  moon_visible = 1;
                  #ifdef DEBUG_MOON_TRACKING
                  debug.println("service_moon_tracking: moon AOS");
                  #endif // DEBUG_MOON_TRACKING
                }
              } else {
                if (moon_visible) {
                  moon_visible = 0;
                  #ifdef DEBUG_MOON_TRACKING
                  debug.println("service_moon_tracking: moon loss of AOS");
                  #endif // DEBUG_MOON_TRACKING
                } else {
                  #ifdef DEBUG_MOON_TRACKING
                  debug.println("service_moon_tracking: moon out of AOS limits");
                  #endif // DEBUG_MOON_TRACKING
                }
              }

              last_check = millis();
            }

          } /* service_moon_tracking */

          char * moon_status_string() {

            char returnstring[128] = "";
            char tempstring[16] = "";

            strcpy(returnstring,"\tmoon: AZ:");
            dtostrf(moon_azimuth,0,2,tempstring);
            strcat(returnstring,tempstring);
            strcat(returnstring," EL:");
            dtostrf(moon_elevation,0,2,tempstring);
            strcat(returnstring,tempstring);
            strcat(returnstring,"  TRACKING_");
            if (!moon_tracking_active) {
              strcat(returnstring,"IN");
            }
            strcat(returnstring,"ACTIVE ");
            if (moon_tracking_active) {
              if (!moon_visible) {
                strcat(returnstring,"NOT_");
              }
              strcat(returnstring,"VISIBLE");
            }
            return returnstring;
          }

          #endif // FEATURE_MOON_TRACKING

          #ifdef FEATURE_SUN_TRACKING
          void service_sun_tracking(){

            static unsigned long last_check = 0;
            static byte sun_tracking_pin_state = 0;
            static byte sun_tracking_activated_by_activate_line = 0;

            if (sun_tracking_active_pin) {
              if ((sun_tracking_active) && (!sun_tracking_pin_state)) {
                digitalWriteEnhanced(sun_tracking_active_pin, HIGH);
                sun_tracking_pin_state = 1;
              }
              if ((!sun_tracking_active) && (sun_tracking_pin_state)) {
                digitalWriteEnhanced(sun_tracking_active_pin, LOW);
                sun_tracking_pin_state = 0;
              }
            }

            if (sun_tracking_activate_line) {
              if ((!sun_tracking_active) && (!digitalReadEnhanced(sun_tracking_activate_line))) {
                sun_tracking_active = 1;
                sun_tracking_activated_by_activate_line = 1;
              }
              if ((sun_tracking_active) && (digitalReadEnhanced(sun_tracking_activate_line)) && (sun_tracking_activated_by_activate_line)) {
                sun_tracking_active = 0;
                sun_tracking_activated_by_activate_line = 0;
              }
            }

            if ((sun_tracking_active) && ((millis() - last_check) > SUN_TRACKING_CHECK_INTERVAL)) {

              update_time();
              update_sun_position();


              #ifdef DEBUG_SUN_TRACKING
              debug.print(F("service_sun_tracking: AZ: "));
              debug.print(sun_azimuth);
              debug.print(" EL: ");
              debug.print(sun_elevation);
              debug.print(" lat: ");
              debug.print(latitude);
              debug.print(" long: ");
              debug.println(longitude);
              #endif // DEBUG_SUN_TRACKING

              if ((sun_azimuth >= SUN_AOS_AZIMUTH_MIN) && (sun_azimuth <= SUN_AOS_AZIMUTH_MAX) && (sun_elevation >= SUN_AOS_ELEVATION_MIN) && (sun_elevation <= SUN_AOS_ELEVATION_MAX)) {
                submit_request(AZ, REQUEST_AZIMUTH, sun_azimuth * HEADING_MULTIPLIER, 13);
                submit_request(EL, REQUEST_ELEVATION, sun_elevation * HEADING_MULTIPLIER, 14);
                if (!sun_visible) {
                  sun_visible = 1;
                  #ifdef DEBUG_SUN_TRACKING
                  debug.println("service_sun_tracking: sun AOS");
                  #endif // DEBUG_SUN_TRACKING
                }
              } else {
                if (sun_visible) {
                  sun_visible = 0;
                  #ifdef DEBUG_SUN_TRACKING
                  debug.println("service_sun_tracking: sun loss of AOS");
                  #endif // DEBUG_SUN_TRACKING
                } else {
                  #ifdef DEBUG_SUN_TRACKING
                  debug.println("service_sun_tracking: sun out of AOS limits");
                  #endif // DEBUG_SUN_TRACKING
                }
              }

              last_check = millis();
            }

          } /* service_sun_tracking */

          char * sun_status_string() {

            char returnstring[128] = "";
            char tempstring[16] = "";

            strcpy(returnstring,"\tsun: AZ:");
            dtostrf(sun_azimuth,0,2,tempstring);
            strcat(returnstring,tempstring);
            strcat(returnstring," EL:");
            dtostrf(sun_elevation,0,2,tempstring);
            strcat(returnstring,tempstring);
            strcat(returnstring,"  TRACKING_");
            if (!sun_tracking_active) {
              strcat(returnstring,"IN");
            }
            strcat(returnstring,"ACTIVE ");
            if (sun_tracking_active) {
              if (!sun_visible) {
                strcat(returnstring,"NOT_");
              }
              strcat(returnstring,"VISIBLE");
            }
            return returnstring;
          }
          #endif // FEATURE_SUN_TRACKING

          #if defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)
          void check_moon_pushbutton_calibration(){

            static unsigned long last_update_time = 0;

            if ((digitalReadEnhanced(pin_moon_pushbutton_calibration) == LOW) && ((millis() - last_update_time) > 500)){
              update_moon_position();
              if (calibrate_az_el(moon_azimuth, moon_elevation)) {
              } else {
              }
              last_update_time = millis();
            }

          }
          #endif //defined(FEATURE_MOON_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_MOON_TRACKING)

          #if defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)
          void check_sun_pushbutton_calibration(){

            static unsigned long last_update_time = 0;

            if ((digitalReadEnhanced(pin_sun_pushbutton_calibration) == LOW) && ((millis() - last_update_time) > 500)){
              update_sun_position();
              if (calibrate_az_el(sun_azimuth, sun_elevation)) {
              } else {
              }
              last_update_time = millis();
            }

          }
          #endif //defined(FEATURE_SUN_PUSHBUTTON_AZ_EL_CALIBRATION) && defined(FEATURE_SUN_TRACKING)

          #if defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)
          char * coordinate_string(){

            char returnstring[32] = "";
            char tempstring[12] = "";

            dtostrf(latitude,0,4,returnstring);
            strcat(returnstring," ");
            dtostrf(longitude,0,4,tempstring);
            strcat(returnstring,tempstring);
            return returnstring;

          }
          #endif //defined(FEATURE_SUN_TRACKING) || defined(FEATURE_MOON_TRACKING)

          #ifdef FEATURE_TIMED_BUFFER
          void clear_timed_buffer() {
            timed_buffer_status = EMPTY;
            timed_buffer_number_entries_loaded = 0;
            timed_buffer_entry_pointer = 0;
          } // clear_timed_buffer

          void initiate_timed_buffer(byte source_port) {

            if (timed_buffer_status == LOADED_AZIMUTHS) {

              timed_buffer_status = RUNNING_AZIMUTHS;

              submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[1], 79);

              last_timed_buffer_action_time = millis();
              timed_buffer_entry_pointer = 2;

              #ifdef DEBUG_TIMED_BUFFER
              debug.println("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS");
              #endif // DEBUG_TIMED_BUFFER

            } else {
              #ifdef FEATURE_ELEVATION_CONTROL
              if (timed_buffer_status == LOADED_AZIMUTHS_ELEVATIONS) {
                timed_buffer_status = RUNNING_AZIMUTHS_ELEVATIONS;
                submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[1], 80);
                submit_request(EL, REQUEST_ELEVATION, timed_buffer_elevations[1], 81);
                last_timed_buffer_action_time = millis();
                timed_buffer_entry_pointer = 2;

                #ifdef DEBUG_TIMED_BUFFER
                debug.println("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS_ELEVATIONS");
                #endif // DEBUG_TIMED_BUFFER

              } else {
                print_to_port(">",source_port);  // error
              }
              #endif
            }
          } // initiate_timed_buffer

          void print_timed_buffer_empty_message() {

            #ifdef DEBUG_TIMED_BUFFER
            debug.println("check_timed_interval: completed timed buffer; changing state to EMPTY");
            #endif // DEBUG_TIMED_BUFFER

          } // print_timed_buffer_empty_message

          void check_timed_interval(){

            if ((timed_buffer_status == RUNNING_AZIMUTHS) && (((millis() - last_timed_buffer_action_time) / 1000) > timed_buffer_interval_value_seconds)) {
              timed_buffer_entry_pointer++;
              #ifdef DEBUG_TIMED_BUFFER
              debug.println("check_timed_interval: executing next timed interval step - azimuths");
              #endif // DEBUG_TIMED_BUFFER
              submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[timed_buffer_entry_pointer - 1], 82);
              last_timed_buffer_action_time = millis();
              if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) {
                clear_timed_buffer();
                print_timed_buffer_empty_message();
              }
            }
            #ifdef FEATURE_ELEVATION_CONTROL

            if ((timed_buffer_status == RUNNING_AZIMUTHS_ELEVATIONS) && (((millis() - last_timed_buffer_action_time) / 1000) > timed_buffer_interval_value_seconds)) {
              timed_buffer_entry_pointer++;
              #ifdef DEBUG_TIMED_BUFFER
              debug.println("check_timed_interval: executing next timed interval step - az and el");
              #endif // DEBUG_TIMED_BUFFER
              submit_request(AZ, REQUEST_AZIMUTH, timed_buffer_azimuths[timed_buffer_entry_pointer - 1], 83);
              submit_request(EL, REQUEST_ELEVATION, timed_buffer_elevations[timed_buffer_entry_pointer - 1], 84);
              last_timed_buffer_action_time = millis();
              if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) {
                clear_timed_buffer();
                print_timed_buffer_empty_message();

              }
            }

            #endif
          } /* check_timed_interval */

          #endif // FEATURE_TIMED_BUFFER
