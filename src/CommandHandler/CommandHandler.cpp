#include "CommandHandler.h"

void CommandHandler::begin(uint32_t baudrate, uint16_t timeout) {
	Serial.begin(baudrate);
	Serial.setTimeout(timeout);
}

void CommandHandler::addCommand(String cmd, fxptr f) {
	fxs.pushBack(f);
	strs.pushBack(cmd);
}

void CommandHandler::check(void) {
	if (Serial.available() > 0) {

		String command = Serial.readStringUntil(' ');
		String args = Serial.readStringUntil('\n');

		QueueIterator<String> strsItr = strs.getIterator();
		QueueIterator<fxptr> fxsItr = fxs.getIterator();

		while(strsItr.hasNext()) {
			if (strsItr.next().equalsIgnoreCase(command) && args.length() > 0) {
				fxsItr.next()(args);
				break;
			}
			else
				fxsItr.next();
		}
	}
}
