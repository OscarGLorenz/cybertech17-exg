#include "CommandHandler.h"

void CommandHandler::begin(uint32_t baudrate, uint16_t timeout) {
	Serial.begin(baudrate);
	Serial.setTimeout(timeout);
}

void CommandHandler::addCommand(String cmd, fxptr f) {
	fxs.pushBack(f);
	strs.pushBack(cmd);
	Serial.println(strs.size());

	for(size_t i = 0; i < strs.size(); i++)
	Serial.print(strs.get(i));

}

void CommandHandler::check(void) {
	if (Serial.available() > 0) {
		String command = Serial.readStringUntil(' ');
		QueueIterator<String> itr = strs.getIterator();
		int i = 0;
		while(itr.hasNext()) {
			String args = Serial.readStringUntil('\n');
			Serial.printtab(args);
			Serial.println(strs.get(i));
			if (args.equalsIgnoreCase(args) && args.length() > 0)
				fxs.get(i)(args);
			i++;
			break;
		}
	}
}
