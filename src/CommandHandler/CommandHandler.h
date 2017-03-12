#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <stdint.h>

#include "Arduino.h"

#include "../DynamicStructures/Queue.h"

typedef void (*fxptr)(String str);

class CommandHandler {
public:
	void begin(uint32_t baudrate = 9600, uint16_t timeout = 10);

	void addCommand(String cmd, fxptr f);

	void check(void);
private:
	Queue<fxptr> fxs;
	Queue<String> strs;
};

#endif
