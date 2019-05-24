#define rotate_cw 6              // goes high to activate rotator R (CW) rotation - pin 1 on Yaesu connector
#define rotate_ccw 7             // goes high to activate rotator L (CCW) rotation - pin 2 on Yaesu connector

#define rotate_up 8               // goes high to activate rotator elevation up
#define rotate_down 9             // goes high to activate rotator elevation down

#define rotator_analog_el A1      // reads analog elevation voltage from rotator
#define rotator_analog_az A0     // reads analog azimuth voltage from rotator - pin 4 on Yaesu connector


#define rotate_cw_ccw  0         // goes high for both CW and CCW rotation
#define rotate_cw_pwm 0          // optional - PWM CW output - set to 0 to disable (must be PWM capable pin)
#define rotate_ccw_pwm 0         // optional - PWM CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_ccw_pwm 0      // optional - PWM on CW and CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_freq 0         // optional - CW variable frequency output
#define rotate_ccw_freq 0        // optional - CCW variable frequency output
#define button_cw 0              // normally open button to ground for manual CW rotation (schematic pin: A2)
#define button_ccw 0             // normally open button to ground for manual CCW rotation (schematic pin: A3)
#define serial_led 0             // LED blinks when command is received on serial port (set to 0 to disable)
#define azimuth_speed_voltage 0  // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_cw_pwm and rotate_ccw_pwm)
#define overlap_led 0            // line goes active when azimuth rotator is in overlap (> 360 rotators)
#define brake_az 0               // goes high to disengage azimuth brake (set to 0 to disable)
#define az_speed_pot 0           // connect to wiper of 1K to 10K potentiometer for speed control (set to 0 to disable)
#define az_preset_pot 0          // connect to wiper of 1K to 10K potentiometer for preset control (set to 0 to disable)
#define preset_start_button 0    // connect to momentary switch (ground on button press) for preset start (set to 0 to disable or for preset automatic start)
#define button_stop 0            // connect to momentary switch (ground on button press) for preset stop (set to 0 to disable or for preset automatic start)
#define rotation_indication_pin 0
#define blink_led 0
#define az_stepper_motor_pulse 0 //0
#define az_stepper_motor_direction 0



/*----------- elevation pins --------------*/
#ifdef FEATURE_ELEVATION_CONTROL

  #define rotate_up_or_down 0       // goes high when elevation up or down is activated
  #define rotate_up_pwm 0           // optional - PWM UP output - set to 0 to disable (must be PWM capable pin)
  #define rotate_down_pwm 0         // optional - PWM DOWN output - set to 0 to disable (must be PWM capable pin)
  #define rotate_up_down_pwm 0      // optional - PWM on both UP and DOWN (must be PWM capable pin)
  #define rotate_up_freq 0          // optional - UP variable frequency output
  #define rotate_down_freq 0        // optional - UP variable frequency output
  #define button_up 0               // normally open button to ground for manual up elevation
  #define button_down 0             // normally open button to ground for manual down rotation
  #define brake_el 0                // goes high to disengage elevation brake (set to 0 to disable)
  #define elevation_speed_voltage  0 // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_up_pwm and rotate_down_pwm)
  #define el_stepper_motor_pulse 0
#endif //FEATURE_ELEVATION_CONTROL

#ifdef FEATURE_PARK
  #define park_in_progress_pin 0  // goes high when a park has been initiated and rotation is in progress
  #define parked_pin 0            // goes high when in a parked position
#endif //FEATURE_PARK

#define heading_reading_inhibit_pin 0 // input - a high will cause the controller to suspend taking azimuth (and elevation) readings; use when RF interferes with sensors

#ifdef FEATURE_AUTOPARK
  #define pin_autopark_disable 0       // Pull low to disable autopark
  #define pin_autopark_timer_reset 0   // Pull low to reset the autopark timer (tie in with rig PTT)
#endif
