
#if !defined(FEATURE_AZ_POSITION_POTENTIOMETER)
  #error "You must specify one AZ position sensor feature"
#endif

#if defined(FEATURE_ELEVATION_CONTROL) && !defined(FEATURE_EL_POSITION_POTENTIOMETER)
  #error "You must specify one EL position sensor feature"
#endif

#if defined(FEATURE_MOON_TRACKING) && !defined(FEATURE_ELEVATION_CONTROL)
  #error "FEATURE_MOON_TRACKING requires FEATURE_ELEVATION_CONTROL"
#endif

#if (defined(FEATURE_MOON_TRACKING) || defined(FEATURE_SUN_TRACKING)) && !defined(FEATURE_CLOCK)
  #error "FEATURE_MOON_TRACKING and FEATURE_SUN_TRACKING requires a clock feature to be activated"
#endif

#if defined(FEATURE_GPS) && !defined(FEATURE_CLOCK)
  #define FEATURE_CLOCK
#endif

#if defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS) && defined(FEATURE_TWO_DECIMAL_PLACE_HEADINGS)
  #error "You need to pick either FEATURE_ONE_DECIMAL_PLACE_HEADINGS or FEATURE_TWO_DECIMAL_PLACE_HEADINGS (or turn both off)"
#endif

#if defined(FEATURE_RTC_DS1307)|| defined(FEATURE_RTC_PCF8583) 
  #define FEATURE_RTC
#endif

#if defined(FEATURE_RTC_DS1307) && defined(FEATURE_RTC_PCF8583)
  #error "You can't have two RTC features enabled!"
#endif
