#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <stdint.h>

#include "Arduino.h"

#include "../DynamicStructures/Vector.h"
#include "../Debug/Debug.h"

typedef void (*fxptr)(String str);

class CommandHandler {
public:
	void begin(uint32_t baudrate = 9600, uint16_t timeout = 10);

	void addCommand(String cmd, fxptr f);

	void check(void);
private:
	dyn::Vector<fxptr> fxs;
	dyn::Vector<String> strs;
};

#endif
