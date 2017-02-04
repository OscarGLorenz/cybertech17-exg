#include "CommandHandler.h"

void CommandHandler::begin(uint32_t baudrate = 9600, uint16_t timeout = 10) {
	Serial.begin(baudrate);
	Serial.setTimeout(timeout);
}

void CommandHandler::addCommand(String cmd, fxptr f) {
	fxs.add(f);
	strs.add(cmd);
}

void CommandHandler::check(void) {
	if (Serial.available() > 0) {
		String command = Serial.readStringUntil(' ');
		for (int i = 0; i < strs.size(); i++) {
			if (strs.get(i).equalsIgnoreCase(command))
				fxs.get(i)(Serial.readStringUntil('\n'));
		}
	}
}
