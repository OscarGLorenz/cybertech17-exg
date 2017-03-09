#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "CommandHandler/CommandHandler.h"
#include "EEPROMHandler/EEPROMHandler.hpp"
#include "DynamicStructures/Queue.h"

#include "QTRSensors.cpp"

EEPROMHandler eepromHandler;

#define DUTY 0.10
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR, PIN2_QTR, PIN3_QTR, PIN4_QTR,
   PIN5_QTR, PIN6_QTR, PIN7_QTR, PIN8_QTR}, 8, 2500);
unsigned int sensorValues[8];
int position = 3500;

double input(void) {
  return (position - 3500.0)/3500.0;
}

void output(double dir) {
  motors.move(dir, 1.0);
}

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
  float K = cmd.toFloat();
  pid.setKp(K);
  eepromHandler.setVariable("P", K);
  showPID();
}
void cfgI(String cmd) {
  float K = cmd.toFloat();
  pid.setKi(K);
  eepromHandler.setVariable("I", K);
  showPID();
}
void cfgD(String cmd) {
  float K = cmd.toFloat();
  pid.setKd(K);
  eepromHandler.setVariable("D", K);
  showPID();
}

bool overSlash = false;
int count = 0;
void read() {
  int sum = 0;
  for (const int& i : sensorValues)
    sum += i;
  if (!overSlash) {
    if (sum > 2000) {
      overSlash = true;
      Serial.println(millis());
      count++;
      while(count >= 10) motors.move(0, 0);
    }
  } else {
    if (sum < 1200) {
      overSlash = false;
    }
  }
}
void setup(){

 handler.begin();

 eepromHandler.defineVariable("P", sizeof(float));
 eepromHandler.defineVariable("I", sizeof(float));
 eepromHandler.defineVariable("D", sizeof(float));

 float K = 0;
 eepromHandler.getVariable("P", K);
 pid.setKp(K);
 eepromHandler.getVariable("I", K);
 pid.setKi(K);
 eepromHandler.getVariable("D", K);
 pid.setKd(K);

 delay(500);
 for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
}
 Serial.println("READY");

 handler.addCommand("P", cfgP);
 handler.addCommand("I", cfgI);
 handler.addCommand("D", cfgD);

}

void loop() {
  handler.check();

  position = (int) qtrrc.readLine(sensorValues);
  pid.check();
  read();
}
