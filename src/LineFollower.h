#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "CommandHandler/CommandHandler.h"

#include "QTRSensors.cpp"

#define DUTY 0.35
Motors motors((char []) {PWM1A, PWM1B}, (char []) {PWM2A, PWM2B}, DUTY);

QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR, PIN2_QTR, PIN3_QTR, PIN4_QTR,
   PIN5_QTR, PIN6_QTR, PIN7_QTR, PIN8_QTR}, 8, 2500);
unsigned int sensorValues[8];
unsigned int position = 3500;

double input(void) {
  return 3500 - position;
}

void output(double dir) {
  motors.move(dir, 1.0);
}

#define KP 0.01
#define KI 0
#define KD 0
PID pid(input,output);

CommandHandler handler;

void showPID() {
  Serial.print("P\tI\tD\n");
  Serial.print(pid.getKp());
  Serial.print("\t");
  Serial.print(pid.getKi());
  Serial.print("\t");
  Serial.println(pid.getKd());

}
void cfgP(String cmd) {
  pid.setKp(cmd.toFloat());
  showPID();
}
void cfgI(String cmd) {
  pid.setKi(cmd.toFloat());
  showPID();
}
void cfgD(String cmd) {
  pid.setKd(cmd.toFloat());
  showPID();
}

void setup(){
  handler.begin();

  delay(500);
  for (int i = 0; i < 400; i++) {
    qtrrc.calibrate();
  }

  pid.setKp(KP);
  pid.setKp(KI);
  pid.setKp(KD);

  handler.addCommand("P", cfgP);
  handler.addCommand("I", cfgI);
  handler.addCommand("D", cfgD);
}

void loop() {
  pid.check();
  handler.check();

  position = qtrrc.readLine(sensorValues);

  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position);

  delay(250);
}
