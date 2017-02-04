CommandHandler cmds;

void led(String str) {
	if (str.equalsIgnoreCase("ON")) {
		digitalWrite(13, HIGH);
	} else if (str.equalsIgnoreCase("OFF")) {
		digitalWrite(13, LOW);
	}
}

void setup() {
	pinMode(13, OUTPUT);
	cmds.begin(9600);
	cmds.addCommand("LED", led);
}

void loop() {
	cmds.check();
}
