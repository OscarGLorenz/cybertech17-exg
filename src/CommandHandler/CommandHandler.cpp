#include "CommandHandler.h"

void CommandHandler::begin(uint32_t baudrate = 9600, uint16_t timeout = 10) {
	Serial.begin(baudrate);
	Serial.setTimeout(timeout);
}

void CommandHandler::addCommand(String cmd, fxptr f) {
	fxs.pushBack(f);
	strs.pushBack(cmd);
	Serial.println(strs.size());

	for(int i = 0; i < strs.size(); i++)
	Serial.print(strs.getInt(i));

}

void CommandHandler::check(void) {
	if (Serial.available() > 0) {
		String command = Serial.readStringUntil(' ');
		dyn::Queue_Iterator<String> itr = strs.getIterator();
		int i = 0;
		while(itr.hasNext()) {
			String args = Serial.readStringUntil('\n');
			Serial.printtab(args);
			Serial.println(strs.getInt(i));
			if (args.equalsIgnoreCase(args) && args.length() > 0)
				fxs.getInt(i)(args);
			i++;
			break;
		}
	}
}
