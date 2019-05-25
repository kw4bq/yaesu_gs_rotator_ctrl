#include "rotator_controller.h"

void setup() {
  delay(1000);
  initialize_serial();
  initialize_peripherals();
  read_settings_from_eeprom();
  initialize_pins();
  read_azimuth(0);
  initialize_display();
} // setup

void loop() {

  #ifdef DEBUG_LOOP
  debug.print("loop()\n");
  Serial.flush();
  #endif // DEBUG_LOOP

  check_serial();

  read_headings();

  service_request_queue();

  service_rotation();

  az_check_operation_timeout();

  #ifdef FEATURE_TIMED_BUFFER
  check_timed_interval();
  #endif // FEATURE_TIMED_BUFFER

  read_headings();

  check_overlap();

  check_brake_release();

  #ifdef FEATURE_ELEVATION_CONTROL
  el_check_operation_timeout();
  #endif

  update_display();

  output_debug();

  read_headings();

  service_rotation();

  check_for_dirty_configuration();

  read_headings();

  service_rotation();

  service_blink_led();

  check_for_reset_flag();

} // loop

void read_headings() {

  #ifdef DEBUG_LOOP
  debug.print("read_headings()\n");
  #endif // DEBUG_LOOP

  read_azimuth(0);
  read_elevation(0);

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

void check_brake_release() {


  static byte in_az_brake_release_delay = 0;
  static unsigned long az_brake_delay_start_time = 0;
  static byte in_el_brake_release_delay = 0;
  static unsigned long el_brake_delay_start_time = 0;

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
  char return_string[100] = "";


  #if defined(CONTROL_PROTOCOL_EMULATION)

  if ((serial_led) && (serial_led_time != 0) && ((millis() - serial_led_time) > SERIAL_LED_TIME_MS)) {
    digitalWriteEnhanced(serial_led, LOW);
    serial_led_time = 0;
  }

  if (control_port->available()) {

    if (serial_led) {
      digitalWriteEnhanced(serial_led, HIGH);                      // blink the LED just to say we got something
      serial_led_time = millis();
    }

    #ifdef DEBUG_SERIAL
    int control_port_available = control_port->available();
    #endif // DEBUG_SERIAL

    incoming_serial_byte = control_port->read();
    last_serial_receive_time = millis();

    #ifdef DEBUG_SERIAL
    debug.print("check_serial: control_port: ");
    debug.print(control_port_available);
    debug.print(":");
    debug.print(incoming_serial_byte);
    debug.println("");
    #endif // DEBUG_SERIAL


    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {  // uppercase it
      incoming_serial_byte = incoming_serial_byte - 32;
    }

    #if defined(FEATURE_YAESU_EMULATION)

    if ((incoming_serial_byte != 10) && (incoming_serial_byte != 13)) {
      // add it to the buffer if it's not a line feed or carriage return
      control_port_buffer[control_port_buffer_index] = incoming_serial_byte;
      control_port_buffer_index++;
    }

    if (incoming_serial_byte == 13) {  // do we have a carriage return?
      if ((control_port_buffer[0] == '\\') || (control_port_buffer[0] == '/')) {
        process_backslash_command(control_port_buffer, control_port_buffer_index, CONTROL_PORT0, return_string);
      } else {
        process_yaesu_command(control_port_buffer,control_port_buffer_index,CONTROL_PORT0,return_string);
        control_port->println(return_string);
        clear_command_buffer();
      }
    }
    #endif

  } // if (control_port->available())
  #endif // CONTROL_PROTOCOL_EMULATION

} /* check_serial */

char * idle_status() {
  return azimuth_direction(azimuth);
} // idle_status

char * azimuth_direction(int azimuth_in) {
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
} // azimuth_direction

void update_display() {
  byte force_display_update_now = 0;
  char workstring[32] = "";
  char workstring2[32] = "";
  byte row_override[LCD_ROWS+1];

  for (int x = 0;x < (LCD_ROWS+1);x++){row_override[x] = 0;}

  k3ngdisplay.clear_pending_buffer();

  #if defined(OPTION_DISPLAY_DIRECTION_STATUS)
  strcpy(workstring,azimuth_direction(azimuth));  // TODO - add left/right/center
  k3ngdisplay.print_center_fixed_field_size(workstring,LCD_DIRECTION_ROW-1,LCD_STATUS_FIELD_SIZE);
  #endif //defined(OPTION_DISPLAY_DIRECTION_STATUS)

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

  #if defined(OPTION_DISPLAY_STATUS)

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

  #endif //defined(OPTION_DISPLAY_STATUS)

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

} // update_display

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

  if (((millis() - last_measurement_time) > AZIMUTH_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

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

    last_measurement_time = millis();
  }
} // read_azimuth

void output_debug() {

  #ifdef DEBUG_DUMP
  char tempstring[32] = "";
  #if defined(CONTROL_PROTOCOL_EMULATION)

  if (((millis() - last_debug_output_time) >= 3000) && (debug_mode)) {

    //port_flush();

    debug.print("debug: \t");
    debug.print(CODE_VERSION);
    debug.print("\t\t");


    debug.print("\t\t");

    debug.print("GS-232B");


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

  #ifdef DEBUG_HEADING_READING_TIME
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;
  static float average_read_time = 0;
  #endif // DEBUG_HEADING_READING_TIME

  if (((millis() - last_measurement_time) > ELEVATION_MEASUREMENT_FREQUENCY_MS) || (force_read)) {

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
    case CW: {
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
        digitalWriteEnhanced(rotate_cw, ROTATE_PIN_ACTIVE_VALUE);
        digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
      } else {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("DEACTIVATE\n"));
        }
        #endif // DEBUG_ROTATOR
        digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
      }
      break;
    }
    case CCW: {
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
        digitalWriteEnhanced(rotate_cw, ROTATE_PIN_INACTIVE_VALUE);
        digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_ACTIVE_VALUE);
      } else {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("DEACTIVATE\n"));
        }
        #endif // DEBUG_ROTATOR
        digitalWriteEnhanced(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE);
      }
      break;
    }
    case UP: {
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
        digitalWriteEnhanced(rotate_up, ROTATE_PIN_ACTIVE_VALUE);
        digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
      } else {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("DEACTIVATE\n"));
        }
        #endif // DEBUG_ROTATOR
        digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
      }
      break;
    }
    case DOWN: {
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
        digitalWriteEnhanced(rotate_up, ROTATE_PIN_INACTIVE_VALUE);
        digitalWriteEnhanced(rotate_down, ROTATE_PIN_ACTIVE_VALUE);
      } else {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {
          debug.print(F("DEACTIVATE\n"));
        }
        #endif // DEBUG_ROTATOR
        digitalWriteEnhanced(rotate_down, ROTATE_PIN_INACTIVE_VALUE);
      }
      break;
    }
    default: {
      break;
    }
  }

  #ifdef DEBUG_ROTATOR
  if (debug_mode) {
    debug.print(F("rotator: exiting\n"));
    control_port->flush();
  }
  #endif // DEBUG_ROTATOR

} // rotator

void initialize_pins() {

  debug.print("initialize_pins()\n");

  #ifdef DEBUG_LOOP
  Serial.flush();
  #endif // DEBUG_LOOP

  pinModeEnhanced(rotate_cw, OUTPUT);
  pinModeEnhanced(rotate_ccw, OUTPUT);

  pinModeEnhanced(rotate_up, OUTPUT);
  pinModeEnhanced(rotate_down, OUTPUT);

  pinModeEnhanced(rotator_analog_az, INPUT);
  pinModeEnhanced(rotator_analog_el, INPUT);

  rotator(DEACTIVATE, CW);
  rotator(DEACTIVATE, CCW);

  rotator(DEACTIVATE, UP);
  rotator(DEACTIVATE, DOWN);

  read_azimuth(0);
  read_elevation(0);

} /* initialize_pins */

void initialize_serial() {

  control_port = CONTROL_PORT_MAPPED_TO;
  control_port->begin(CONTROL_PORT_BAUD_RATE);
  #if defined(OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING)
  control_port->print OPTION_SEND_STRING_OUT_CONTROL_PORT_WHEN_INITIALIZING_STRING;
  #endif

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

  if (axis == EL) {
    #ifdef DEBUG_SUBMIT_REQUEST
    debug.print("EL ");
    #endif // DEBUG_SUBMIT_REQUEST
    el_request = request;
    el_request_parm = parm;
    el_request_queue_state = IN_QUEUE;
  }

  #ifdef DEBUG_SUBMIT_REQUEST
  switch(request) {
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

void service_request_queue() {

  int work_target_raw_azimuth = 0;
  byte direction_to_go = 0;
  byte within_tolerance_flag = 0;

  if (az_request_queue_state == IN_QUEUE) {

    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("service_request_queue: AZ ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE

    switch (az_request) {
      case (REQUEST_STOP): {
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        debug.print("REQUEST_STOP");
        #endif // DEBUG_SERVICE_REQUEST_QUEUE
        #ifdef FEATURE_PARK
        deactivate_park();
        #endif // FEATURE_PARK
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
    }
      case (REQUEST_AZIMUTH): {
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
}
      case (REQUEST_AZIMUTH_RAW): {
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
}
      case (REQUEST_CW): {
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print("REQUEST_CW");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  #ifdef FEATURE_PARK
  deactivate_park();
  #endif // FEATURE_PARK
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
}
      case (REQUEST_CCW): {
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print("REQUEST_CCW");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  #ifdef FEATURE_PARK
  deactivate_park();
  #endif // FEATURE_PARK
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
}
      case (REQUEST_KILL): {
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.print("REQUEST_KILL");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  #ifdef FEATURE_PARK
  deactivate_park();
  #endif // FEATURE_PARK
  rotator(DEACTIVATE, CW);
  rotator(DEACTIVATE, CCW);
  az_state = IDLE;
  az_request_queue_state = NONE;
  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
  debug.println("");
  #endif // DEBUG_SERVICE_REQUEST_QUEUE
  break; // REQUEST_KILL
}
      default: break;
    } /* switch */

    if (az_request_queue_state != IN_QUEUE) {
      push_lcd_update = 1;
    }
  }

  if (el_request_queue_state == IN_QUEUE) {

    within_tolerance_flag = 0;

    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    debug.print("service_request_queue: EL ");
    #endif // DEBUG_SERVICE_REQUEST_QUEUE

    switch (el_request) {
      case (REQUEST_ELEVATION): {
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
  }
      case (REQUEST_UP): {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_UP\n"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      #ifdef FEATURE_PARK
      deactivate_park();
      #endif // FEATURE_PARK
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
      }
      case (REQUEST_DOWN): {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_DOWN\n"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      #ifdef FEATURE_PARK
      deactivate_park();
      #endif // FEATURE_PARK
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
      }
      case (REQUEST_STOP): {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_STOP\n"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      #ifdef FEATURE_PARK
      deactivate_park();
      #endif // FEATURE_PARK
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
      }
      case (REQUEST_KILL): {
      #ifdef DEBUG_SERVICE_REQUEST_QUEUE
      if (debug_mode) {
        debug.print(F("REQUEST_KILL\n"));
      }
      #endif // DEBUG_SERVICE_REQUEST_QUEUE
      #ifdef FEATURE_PARK
      deactivate_park();
      #endif // FEATURE_PARK
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
      }
      default: break;
    } /* switch */

    if (el_request_queue_state != IN_QUEUE) {
      push_lcd_update = 1;
    }
  }

} /* service_request_queue */


void service_rotation() {

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

  if ((az_state != IDLE) && (az_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    read_azimuth(0);
    if (az_state == NORMAL_CW) {
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
      read_azimuth(0);
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

  if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) ) {
    read_elevation(0);
    if (el_state == NORMAL_UP) {
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

} /* service_rotation */

void check_for_dirty_configuration() {
  static unsigned long last_config_write_time = 0;
  if ((configuration_dirty) && ((millis() - last_config_write_time) > ((unsigned long)EEPROM_WRITE_DIRTY_CONFIG_TIME * 1000))) {
    write_settings_to_eeprom();
    last_config_write_time = millis();
  }
} // check_for_dirty_configuration

byte current_az_state() {
  switch (az_state) {
    case NORMAL_CW: {
      return ROTATING_CW;
      break;
    }
    case NORMAL_CCW: {
      return ROTATING_CCW;
      break;
    }
    default: {
      return NOT_DOING_ANYTHING;
      break;
    }
  }
} // current_az_state

byte current_el_state() {
  switch (az_state) {
    case NORMAL_UP: {
      return ROTATING_UP;
      break;
    }
    case NORMAL_DOWN: {
      return ROTATING_DOWN;
      break;
    }
    default: {
      return NOT_DOING_ANYTHING;
      break;
    }
  }
} // current_el_state

byte get_analog_pin(byte pin_number) {
  byte return_output = 0;
  switch (pin_number) {
    case 0: { return_output = A0; break; }
    case 1: { return_output = A1; break; }
    case 2: { return_output = A2; break; }
    case 3: { return_output = A3; break; }
    case 4: { return_output = A4; break; }
    case 5: { return_output = A5; break; }
    case 6: { return_output = A6; break; }
    default: { break; }
  }
  return return_output;
}

void pinModeEnhanced(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
}

void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue) {
  digitalWrite(pin, writevalue);
}

int digitalReadEnhanced(uint8_t pin) {
  return digitalRead(pin);
}

int analogReadEnhanced(uint8_t pin) {
  return analogRead(pin);
}

void analogWriteEnhanced(uint8_t pin, int writevalue) {
  analogWrite(pin, writevalue);
}

void port_flush() {
  control_port->flush();
}

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

void get_keystroke() {
  while (control_port->available() > 0) {
    incoming_serial_byte = control_port->read();
  }
}

void print_wrote_to_memory() {
  control_port->println(F("Wrote to memory"));
}

void clear_serial_buffer() {
  delay(200);
  while (control_port->available()) {
    incoming_serial_byte = control_port->read();
  }
} // clear_serial_buffer

byte process_backslash_command(byte input_buffer[], int input_buffer_index, byte source_port, char * return_string) {

  strcpy(return_string,"");
  float tempfloat = 0;
  float heading = 0;
  long place_multiplier = 0;
  byte decimalplace = 0;
  int new_azimuth_starting_point;
  int new_azimuth_rotation_capability;
  byte brake_az_disabled;
  char temp_string[20] = "";

  switch (input_buffer[1]) {
    case 'A': {
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 360)) {
        configuration.azimuth_offset = 0;
        read_azimuth(1);
        configuration.azimuth_offset = tempfloat - float(raw_azimuth / HEADING_MULTIPLIER);
        configuration_dirty = 1;
        strcpy(return_string, "Azimuth calibrated to ");
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }
      break;
    } // \Ax[xxx][.][xxxx] - manually set azimuth
    case 'I': {
      new_azimuth_starting_point = 9999;
      switch (input_buffer_index) {
        case 2: {
          new_azimuth_starting_point = configuration.azimuth_starting_point;
          break;
        }
        case 3: {
          new_azimuth_starting_point = (input_buffer[2] - 48);
          break;
        }
        case 4: {
          new_azimuth_starting_point = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        }
        case 5: {
          new_azimuth_starting_point = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
        }
        default: {
          break;
        }
      } // switch
      if ((new_azimuth_starting_point  >= 0) && (new_azimuth_starting_point  < 360)) {
        if (input_buffer_index > 2) {
          azimuth_starting_point = configuration.azimuth_starting_point = new_azimuth_starting_point;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Azimuth starting point set to ");
        dtostrf(new_azimuth_starting_point, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Ix[x][x]");
      }
      break;
    } // \Ix[x][x] - set az starting point
    case 'J': {
      new_azimuth_rotation_capability = 9999;
      switch (input_buffer_index) {
        case 2: {
          new_azimuth_rotation_capability = configuration.azimuth_rotation_capability;
          break;
        }
        case 3: {
          new_azimuth_rotation_capability = (input_buffer[2] - 48);
          break;
        }
        case 4: {
          new_azimuth_rotation_capability = ((input_buffer[2] - 48) * 10) + (input_buffer[3] - 48);
          break;
        }
        case 5: {
          new_azimuth_rotation_capability = ((input_buffer[2] - 48) * 100) + ((input_buffer[3] - 48) * 10) + (input_buffer[4] - 48);
          break;
        }
      }
      if ((new_azimuth_rotation_capability >= 0) && (new_azimuth_rotation_capability <= 450)) {
        if (input_buffer_index > 2) {
          azimuth_rotation_capability = configuration.azimuth_rotation_capability = new_azimuth_rotation_capability;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Azimuth rotation capability set to ");
        dtostrf(new_azimuth_rotation_capability, 0, 0, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.  Format: \\Jx[x][x]");
      }
      break;
    } // \Jx[x][x] - set az rotation capability
    case 'K': {
      brake_az_disabled = 2;
      if (input_buffer_index == 2) {
        brake_az_disabled = configuration.brake_az_disabled;
      } else {
        switch (input_buffer[2]) {
          case '0': brake_az_disabled = 0; break;
          case '1': brake_az_disabled = 1; break;
        }
      }
      if ((brake_az_disabled >=0) && (brake_az_disabled <= 1)) {
        if (input_buffer_index > 2) {
          configuration.brake_az_disabled = brake_az_disabled;
          configuration_dirty = 1;
        }
        strcpy(return_string, "Az brake ");
        strcat(return_string, (brake_az_disabled ? "disabled." : "enabled."));
      } else {
        strcpy(return_string, "Error.");
      }
      break;
    } // \Kx   - Force disable the az brake even if a pin is defined (x: 0 = enable, 1 = disable)
    case 'B': {
      place_multiplier = 1;
      for (int x = input_buffer_index - 1; x > 1; x--) {
        if (char(input_buffer[x]) != '.') {
          tempfloat += (input_buffer[x] - 48) * place_multiplier;
          place_multiplier = place_multiplier * 10;
        } else {
          decimalplace = x;
        }
      }
      if (decimalplace) {
        tempfloat = tempfloat / pow(10, (input_buffer_index - decimalplace - 1));
      }
      if ((tempfloat >= 0) && (tempfloat <= 180)) {
        configuration.elevation_offset = 0;
        read_elevation(1);
        configuration.elevation_offset = tempfloat - float(elevation / HEADING_MULTIPLIER);
        configuration_dirty = 1;
        strcpy(return_string, "Elevation calibrated to ");
        dtostrf(tempfloat, 0, 2, temp_string);
        strcat(return_string, temp_string);
      } else {
        strcpy(return_string, "Error.");
      }
      break;
    } // \Bx[xxx][.][xxxx] - manually set elevation
    case 'D': {
      if (debug_mode & source_port) {
        debug_mode = debug_mode & (~source_port);
      } else {
        debug_mode = debug_mode | source_port;
      }
      break;
    } // \D - Debug
    case 'E': {
      initialize_eeprom_with_defaults();
      strcpy(return_string, "Initialized eeprom, resetting unit in 5 seconds...");
      reset_the_unit = 1;
      break;
    } // \E - Initialize eeprom
    case 'Q': {
      write_settings_to_eeprom();
      strcpy(return_string, "Settings saved in EEPROM, resetting unit in 5 seconds...");
      reset_the_unit = 1;
      break;
    } // \Q - Save settings in the EEPROM and restart
    case 'L': {
      if (azimuth < (180 * HEADING_MULTIPLIER)) {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth + (180 * HEADING_MULTIPLIER)), 15);
      } else {
        submit_request(AZ, REQUEST_AZIMUTH, (azimuth - (180 * HEADING_MULTIPLIER)), 16);
      }
      break;
    } // \L - rotate to long path
    case '+': {
      if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_OVERLAP_PLUS){
        configuration.azimuth_display_mode = AZ_DISPLAY_MODE_NORMAL;
        strcpy(return_string, "Azimuth Display Mode: Normal");
      } else {
        if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_RAW){
          configuration.azimuth_display_mode = AZ_DISPLAY_MODE_OVERLAP_PLUS;
          strcpy(return_string, "Azimuth Display Mode: +Overlap");
        } else {
          if (configuration.azimuth_display_mode == AZ_DISPLAY_MODE_NORMAL){
            configuration.azimuth_display_mode = AZ_DISPLAY_MODE_RAW;
            strcpy(return_string, "Azimuth Display Mode: Raw Degrees");
          }
        }
      }
      configuration_dirty = 1;
      break;
    }
    case '?': {
      strcpy(return_string, "\\!??");  //  \\??xxyy - failed response back
      if (input_buffer_index == 4){
        if ((input_buffer[2] == 'F') && (input_buffer[3] == 'S')) {  // \?FS - Full Status
          strcpy(return_string, "\\!OKFS");
          // AZ
          if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,temp_string);
          strcat(return_string,temp_string);
          strcat(return_string,",");
          // EL
          if ((elevation/HEADING_MULTIPLIER) >= 0) {
            strcat(return_string,"+");
          } else {
            strcat(return_string,"-");
          }
          if (abs(elevation/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if (abs(elevation/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,temp_string);
          strcat(return_string,temp_string);
          strcat(return_string,",");
          // AS
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string);
          strcat(return_string,",");
          // ES
          dtostrf(el_state, 0, 0, temp_string);
          strcat(return_string, temp_string);
          strcat(return_string,",");

        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'Z')) {  // \?AZ - query AZ
          strcpy(return_string, "\\!OKAZ");
          if ((raw_azimuth/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if ((raw_azimuth/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(raw_azimuth/(float)HEADING_MULTIPLIER),0,6,temp_string);
          strcat(return_string,temp_string);
        }
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'L')) {  // \?EL - query EL
          strcpy(return_string, "\\!OKEL");
          if ((elevation/HEADING_MULTIPLIER) >= 0) {
            strcat(return_string,"+");
          } else {
            strcat(return_string,"-");
          }
          if (abs(elevation/HEADING_MULTIPLIER) < 100) {
            strcat(return_string,"0");
          }
          if (abs(elevation/HEADING_MULTIPLIER) < 10) {
            strcat(return_string,"0");
          }
          dtostrf(float(abs(elevation/(float)HEADING_MULTIPLIER)),0,6,temp_string);
          strcat(return_string,temp_string);
        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'S')) {  // \?AS - AZ status
          strcpy(return_string, "\\!OKAS");
          dtostrf(az_state, 0, 0, temp_string);
          strcat(return_string, temp_string);
        }
        if ((input_buffer[2] == 'E') && (input_buffer[3] == 'S')) {  // \?ES - EL Status
          strcpy(return_string, "\\!OKES");
          dtostrf(el_state, 0, 0, temp_string);
          strcat(return_string, temp_string);
        }
        if ((input_buffer[2] == 'P') && (input_buffer[3] == 'G')) {  // \?PG - Ping
          strcpy(return_string, "\\!OKPG");
        }
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'L')) {  // \?RL - rotate left
          submit_request(AZ, REQUEST_CCW, 0, 121);
          strcpy(return_string, "\\!OKRL");
        }
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'R')) {  // \?RR - rotate right
          submit_request(AZ, REQUEST_CW, 0, 122);
          strcpy(return_string, "\\!OKRR");
        }
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'U')) {  //  \?RU - elevate up
          submit_request(EL, REQUEST_UP, 0, 129);
          strcpy(return_string, "\\!OKRU");
        }
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'D')) {  // \?RD - elevate down
          submit_request(EL, REQUEST_DOWN, 0, 130);
          strcpy(return_string, "\\!OKRD");
        }
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'A')) {  // \?SA - stop azimuth rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
          strcpy(return_string,"\\!OKSA");
        }
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'E')) {  // \?SE - stop elevation rotation
          submit_request(EL, REQUEST_STOP, 0, 125);
          strcpy(return_string,"\\!OKSE");
        }
        if ((input_buffer[2] == 'S') && (input_buffer[3] == 'S')) {  // \?SS - stop all rotation
          submit_request(AZ, REQUEST_STOP, 0, 124);
          submit_request(EL, REQUEST_STOP, 0, 125);
          strcpy(return_string,"\\!OKSS");
        }
        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'L')) {  // \?CL - read the clock
          strcpy(return_string,"\\!??CL");
        }
        if ((input_buffer[2] == 'R') && (input_buffer[3] == 'B')) {  // \?RB - reboot
          wdt_enable(WDTO_30MS); while (1) {}  //ZZZZZZ - TODO - change to reboot flag
        }
        if ((input_buffer[2] == 'C') && (input_buffer[3] == 'V')) {  // \?CV Code Verson
          strcpy(return_string,"\\!OKCV");
          strcat(return_string,CODE_VERSION);
        }

      } //if (input_buffer_index == 4)

      if (input_buffer_index == 6){
        if ((input_buffer[2] == 'D') && (input_buffer[3] == 'O')) {  // \?DOxx - digital pin initialize as output; xx = pin # (01, 02, A0,etc.)
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[4] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            strcpy(return_string,"\\!OKDO");
            pinModeEnhanced(pin_value, OUTPUT);
          }
        }

        if ((input_buffer[2] == 'D') && ((input_buffer[3] == 'H') || (input_buffer[3] == 'L'))) { // \?DLxx - digital pin write low; xx = pin #   \?DHxx - digital pin write high; xx = pin #
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            if (input_buffer[3] == 'H') {
              digitalWriteEnhanced(pin_value, HIGH);
              strcpy(return_string,"\\!OKDH");
            } else {
              digitalWriteEnhanced(pin_value, LOW);
              strcpy(return_string,"\\!OKDL");
            }
          }
        }

        if ((input_buffer[2] == 'D') && (input_buffer[3] == 'I')) {  // \?DIxx - digital pin initialize as input; xx = pin #
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            pinModeEnhanced(pin_value, INPUT);
            strcpy(return_string,"\\!OKDI");
          }
        }

        if ((input_buffer[2] == 'D') && (input_buffer[3] == 'P')) {  // \?DPxx - digital pin initialize as input with pullup; xx = pin #
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            pinModeEnhanced(pin_value, INPUT);
            digitalWriteEnhanced(pin_value, HIGH);
            strcpy(return_string,"\\!OKDP");
          }
        }

        if ((input_buffer[2] == 'D') && (input_buffer[3] == 'R')) {  // \?DRxx - digital pin read; xx = pin #
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            byte pin_read = digitalReadEnhanced(pin_value);
            strcpy(return_string,"\\!OKDR");
            dtostrf((input_buffer[4]-48),0,0,temp_string);
            strcat(return_string,temp_string);
            dtostrf((input_buffer[5]-48),0,0,temp_string);
            strcat(return_string,temp_string);
            if (pin_read) {
              strcat(return_string,"1");
            } else {
              strcat(return_string,"0");
            }
          }
        }
        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'R')) {  //  \?ARxx - analog pin read; xx = pin #
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            int pin_read = analogReadEnhanced(pin_value);
            strcpy(return_string,"\\!OKAR");
            if (toupper(input_buffer[4]) == 'A') {
              strcat(return_string,"A");
            } else {
              dtostrf((input_buffer[4]-48),0,0,temp_string);
              strcat(return_string,temp_string);
            }

            dtostrf((input_buffer[5]-48),0,0,temp_string);
            strcat(return_string,temp_string);
            if (pin_read < 1000) {
              strcat(return_string,"0");
            }
            if (pin_read < 100) {
              strcat(return_string,"0");
            }
            if (pin_read < 10) {
              strcat(return_string,"0");
            }
            dtostrf(pin_read,0,0,temp_string);
            strcat(return_string,temp_string);
          }
        }

        if ((input_buffer[2] == 'N') && (input_buffer[3] == 'T')) { // \?NTxx - no tone; xx = pin #
          byte pin_value = 0;
          if (toupper(input_buffer[4]) == 'A') {
            pin_value = get_analog_pin(input_buffer[5] - 48);
          } else {
            pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
          }
          noTone(pin_value);
          strcpy(return_string,"\\!OKNT");
        }
      }  //if ((input_buffer_index == 6)

      if (input_buffer_index == 9) {

        if ((input_buffer[2] == 'G') && (input_buffer[3] == 'A')) {  // \?GAxxx.x - go to AZ xxx.x
          heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[6] - 48.) + ((input_buffer[8] - 48) / 10.);
          if (((heading >= 0) && (heading < 451))  && (input_buffer[7] == '.')) {
            submit_request(AZ, REQUEST_AZIMUTH, (heading * HEADING_MULTIPLIER), 136);
            strcpy(return_string,"\\!OKGA");
          } else {
            strcpy(return_string,"\\!??GA");
          }
        }
        if ((input_buffer[2] == 'G') && (input_buffer[3] == 'E')) {  // \?GExxx.x - go to EL
          #ifdef FEATURE_ELEVATION_CONTROL
          heading = ((input_buffer[4] - 48) * 100.) + ((input_buffer[5] - 48) * 10.) + (input_buffer[5] - 48) + ((input_buffer[8] - 48) / 10.);
          if (((heading >= 0) && (heading < 181)) && (input_buffer[7] == '.')) {
            submit_request(EL, REQUEST_ELEVATION, (heading * HEADING_MULTIPLIER), 37);
            strcpy(return_string,"\\!OKGE");
          } else {
            strcpy(return_string,"\\!??GE");
          }
          #else
          strcpy(return_string,"\\!OKGE");
          #endif // #FEATURE_ELEVATION_CONTROL
        }


        if ((input_buffer[2] == 'A') && (input_buffer[3] == 'W')) {  // \?AWxxyyy - analog pin write; xx = pin #, yyy = value to write (0 - 255)
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            int write_value = ((input_buffer[6] - 48) * 100) + ((input_buffer[7] - 48) * 10) + (input_buffer[8] - 48);
            if ((write_value >= 0) && (write_value < 256)) {
              analogWriteEnhanced(pin_value, write_value);
              strcpy(return_string,"\\!OKAW");
            }
          }
        }
      } //if (input_buffer_index == 9)

      if (input_buffer_index == 10) {
        if ((input_buffer[2] == 'D') && (input_buffer[3] == 'T')) { // \?DTxxyyyy - digital pin tone output; xx = pin #, yyyy = frequency
          if ((((input_buffer[4] > 47) && (input_buffer[4] < 58)) || (toupper(input_buffer[4]) == 'A')) && (input_buffer[5] > 47) && (input_buffer[5] < 58)) {
            byte pin_value = 0;
            if (toupper(input_buffer[4]) == 'A') {
              pin_value = get_analog_pin(input_buffer[5] - 48);
            } else {
              pin_value = ((input_buffer[4] - 48) * 10) + (input_buffer[5] - 48);
            }
            int write_value = ((input_buffer[6] - 48) * 1000) + ((input_buffer[7] - 48) * 100) + ((input_buffer[8] - 48) * 10) + (input_buffer[9] - 48);
            if ((write_value >= 0) && (write_value <= 9999)) {
              tone(pin_value, write_value);
              strcpy(return_string,"\\!OKDT");

            }
          }
        }
      }  //if (input_buffer_index == 10)

      break;
    } // \?
    default: {
      break;
    }
  } // switch
  return(0);
} // process_backslash_command

void process_yaesu_command(byte * yaesu_command_buffer, int yaesu_command_buffer_index, byte source_port, char * return_string) {

  char tempstring[11] = "";
  int parsed_value = 0;
  int parsed_elevation = 0;

  #ifdef FEATURE_TIMED_BUFFER
  int parsed_value2 = 0;
  #endif //FEATURE_TIMED_BUFFER

  strcpy(return_string,"");

  // look at the first character of the command
  switch (yaesu_command_buffer[0]) {
    case 'C': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: C\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      #ifdef OPTION_DELAY_C_CMD_OUTPUT
      delay(400);
      #endif
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

      if ((yaesu_command_buffer[1] == '2') && (yaesu_command_buffer_index > 1)) {     // did we get the C2 command?
        #ifndef OPTION_GS_232B_EMULATION
        strcat(return_string,"+0000");    // return a dummy elevation since we don't have the elevation feature turned on
        #else
        strcat(return_string,"EL=000");
        #endif
      } else {
        //strcat(return_string,"\n");
      }
      break;
    } // C - return current azimuth
    case 'F': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: F\n");
      }
      #endif // DEBUG_PROCESS_YAESU
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
    } // F - full scale calibration
    case 'H': {
      print_help(source_port);
      break;
    } // H - print help - depricated
    case 'L': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: L\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(AZ, REQUEST_CCW, 0, 21);
      //strcpy(return_string,"\n");
      break;
    } // L - manual left (CCW) rotation
    case 'O': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: O\n");
      }
      #endif // DEBUG_PROCESS_YAESU
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
    } // O - offset calibration
    case 'R': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: R\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(AZ, REQUEST_CW, 0, 22);
      strcpy(return_string,"\n");
      break;
    } // R - manual right (CW) rotation
    case 'A': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: A\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(AZ, REQUEST_STOP, 0, 23);
      //strcpy(return_string,"\n");
      break;
    } // A - CW/CCW rotation stop
    case 'S': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: S\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(AZ, REQUEST_STOP, 0, 24);
      submit_request(EL, REQUEST_STOP, 0, 25);
      #ifdef FEATURE_TIMED_BUFFER
      clear_timed_buffer();
      #endif // FEATURE_TIMED_BUFFER
      break;
    } // S - all stop
    case 'M': {
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
    } // M - auto azimuth rotation
    case 'X': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: X\n");
      }
      #endif // DEBUG_PROCESS_YAESU

      if (yaesu_command_buffer_index > 1) {
        switch (yaesu_command_buffer[1]) {
          case '4':{
            normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
            update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
            #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
            normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
            update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);
            #endif
            strcpy(return_string,"Speed X4");
            break;
          }
          case '3': {
            normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;
            update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
            #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
            normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;
            update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);
            #endif
            strcpy(return_string,"Speed X3");
            break;
          }
          case '2': {
            normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;
            update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);
            #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
            normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;
            update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);
            #endif
            strcpy(return_string,"Speed X2");
            break;
          }
          case '1': {
            normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
            update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1);
            #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
            normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;
            update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);
            #endif
            strcpy(return_string,"Speed X1");
            break;
          }
          default: {
            strcpy(return_string,"?>");
            break;
          }
        } /* switch */
      } else {
        strcpy(return_string,"?>");
      }
      break;
    } // X - azimuth speed change
    case 'U': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: U\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(EL, REQUEST_UP, 0, 29);
      //strcpy(return_string,"\n");
      break;
    } // U - manual up rotation
    case 'D':  {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: D\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      submit_request(EL, REQUEST_DOWN, 0, 30);
      //strcpy(return_string,"\n");
      break;
    } // D - manual down rotation
    case 'E': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: E\n");
      }
      #endif // DEBUG_PROCESS_YAESU

      submit_request(EL, REQUEST_STOP, 0, 31);
      //strcpy(return_string,"\n");
      break;
    } // E - stop elevation rotation
    case 'B': {
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
    } // B - return current elevation
    case 'W': {
      #ifdef DEBUG_PROCESS_YAESU
      if (debug_mode) {
        debug.print("yaesu_serial_command: W\n");
      }
      #endif // DEBUG_PROCESS_YAESU
      // parse out W command
      // Short Format: WXXX YYY            XXX = azimuth YYY = elevation
      // Long Format : WSSS XXX YYY        SSS = timed interval   XXX = azimuth    YYY = elevation
      if (yaesu_command_buffer_index > 8) {
        // if there are more than 4 characters in the command buffer, we got a timed interval command
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

      if ((parsed_value >= 0) && (parsed_value <= ((azimuth_starting_point + azimuth_rotation_capability)* HEADING_MULTIPLIER)) && (parsed_elevation >= 0) && (parsed_elevation <= (ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER))) {
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
      break;
    } // W - auto elevation rotation
    case 'P':  {
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
    } // P - switch between 360 and 450 degree mode
    case 'Z': {
      if (azimuth_starting_point == 180) {
        azimuth_starting_point = 0;
        strcpy(return_string,"N");
      } else {
        azimuth_starting_point = 180;
        strcpy(return_string,"S");
      }
      strcat(return_string," Center");
      break;
    } // Z - Starting point toggle
    default: {
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
      break;
    }
  } /* switch */
} /* yaesu_serial_command */

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

  if (axis == EL){
    autocorrect_state_el = AUTOCORRECT_WATCHING_EL;
    autocorrect_el = heading;
    autocorrect_el_submit_time = millis();

    #ifdef DEBUG_AUTOCORRECT
    debug.print("EL: ");
    #endif //DEBUG_AUTOCORRECT

  }

  #ifdef DEBUG_AUTOCORRECT
  debug.print(heading,2);
  debug.println("");
  #endif //DEBUG_AUTOCORRECT

}
#endif //FEATURE_AUTOCORRECT

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
