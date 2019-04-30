// debug.h  contributed by Matt VK5ZM

#ifndef _ROTATOR_DEBUG_h
#define _ROTATOR_DEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "rotator.h"
#include "rotator_features.h" 

#define SERIAL_PORT_CLASS HardwareSerial

class DebugClass
{
 protected:


 public:
	void init();

	void print(const char *str);
	void print(const __FlashStringHelper *str);
	void print(char ch);
	void print(int i);
	void print(float f);
	void print(float f, byte places);
	void print(unsigned int i);
	void print(long unsigned int i);
	void print(long i);
	void print(double i);
	
	void println(double i);
	void println(const char *str);
	void println(const __FlashStringHelper *str);
	
	void write(const char *str);
	void write(int i);
};


extern uint8_t debug_mode;
extern SERIAL_PORT_CLASS * control_port;

// #ifdef FEATURE_ETHERNET
//   extern EthernetClient ethernetclient0;
// #endif

#endif //_ROTATOR_DEBUG_h

