
#include "rotator_debug.h"

void DebugClass::print(const char *str) {
	  if (debug_mode & CONTROL_PORT0) {
	  	control_port->print(str);
	  }
}

void DebugClass::print(const __FlashStringHelper *str) {
	char c;
	if(!str) return;
	
	/* since str is a const we can't increment it, so do this instead */
	char *p = (char *)str;
	
	/* keep going until we find the null */
	while((c = pgm_read_byte(p++))) {
		 if (debug_mode & CONTROL_PORT0) {
			control_port->write(c);
		 }
	}
}

void DebugClass::println(const __FlashStringHelper *str) {
	char c;
	if(!str) return;
	
	/* since str is a const we can't increment it, so do this instead */
	char *p = (char *)str;
	
	/* keep going until we find the null */
	while((c = pgm_read_byte(p++))) {
		 if (debug_mode & CONTROL_PORT0) {
			control_port->write(c);
		 }
	}
	control_port->println();
}

void DebugClass::print(char ch) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(ch);
	}
}

void DebugClass::print(int i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(i);
	}
}

void DebugClass::print(unsigned int i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(i);
	}
}

void DebugClass::print(long unsigned int i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(i);
	}
}

void DebugClass::print(long i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(i);
	}
}

void DebugClass::print(double i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->print(i);
	}
}

void DebugClass::println(double i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->println(i);
	}
}

void DebugClass::print(float f,byte places) {
	char tempstring[16] = "";
	dtostrf( f,0,places,tempstring);

	if (debug_mode & CONTROL_PORT0) {
		control_port->print(tempstring);
	}
}

void DebugClass::print(float f) {
	char tempstring[16] = "";
	dtostrf( f,0,2,tempstring);

	if (debug_mode & CONTROL_PORT0) {
		control_port->print(tempstring);
	}
}

void DebugClass::println(const char *str) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->println(str);
	}
}

void DebugClass::write(const char *str) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->write(str);
	}
}

void DebugClass::write(int i) {
	if (debug_mode & CONTROL_PORT0) {
		control_port->write(i);
	}
}
