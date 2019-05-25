
#ifndef _ROTATOR_DEBUG_h
#define _ROTATOR_DEBUG_h

#include "Arduino.h"

#include "rotator.h"
#include "rotator_features.h"

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
extern HardwareSerial * control_port;

#endif //_ROTATOR_DEBUG_h
