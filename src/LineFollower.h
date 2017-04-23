#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "CommandHandler/CommandHandler.h"
#include "EEPROMHandler/EEPROMHandler.hpp"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"
#include "Global.h"

#include "QTRSensors.cpp"

EEPROMHandler eepromHandler;

#define DUTY 250
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR, PIN2_QTR, PIN3_QTR, PIN4_QTR, PIN5_QTR, PIN6_QTR, PIN7_QTR, PIN8_QTR}, 8, 2500);
unsigned int sensorValues[8];
int position = 3500;

PID pid([]()->double {return (position - 3500.0)/3500.0;},[](double dir){motors.move(-constrain(dir, -1.0, 1.0), 1.0);});

CommandHandler handler;

Angle yaw0(0);
Gyroscope gyro(INTERRUPT_GYRO);

//PID ROTATION
//Lectura de error de girar
#define ROTATION_KP 0.25
#define ROTATION_KI 0
#define ROTATION_KD 0.01
#define ROTATION_KF 0.80
#define ROTATION_SAT 0.20
#define ROTATION_DEAD 0.11
#define ROTATION_THRESHOLD 0.08
PID rotation([]()-> double {return (yaw0-gyro.getAlpha()).get();}, [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));}, 0.15);
//PID ROTATION

void testTurn(double value) {
  motors.move(0,0.2);
  delay(200);

  yaw0 = yaw0 + value;

  int c = 0;
  while(c < 1) {
    gyro.check();
    rotation.check();

    if(abs((yaw0 - gyro.getAlpha()).get()) < ROTATION_THRESHOLD) c++;
    delay(2);
  }

  rotation.resetSumShaft();
}

void showPID() { Serial.println("P\tI\tD\n" + String(pid.getKp()) + "\t" + String(pid.getKi()) + "\t" + String(pid.getKd()));}

void turn90() {
  int sumLeft = sensorValues[0] + sensorValues[1] + sensorValues[2];
  int sumRight = sensorValues[5] + 1000 + sensorValues[7];
motors.setMax(250);
  if (sumLeft > 2500 && sumRight < 2500) {
    Serial.println("TURN RIGHT");
    yaw0 = gyro.getAlpha();
    testTurn(M_PI_2);
  } else if (sumLeft < 2500 && sumRight > 2500) {
    Serial.println("TURN LEFT");
    yaw0 = gyro.getAlpha();
    testTurn(-M_PI_2);
  }
  motors.setMax(80);
}

void setup(){
  start();

  rotation.setKp(ROTATION_KP);
  rotation.setKi(ROTATION_KI);
  rotation.setKd(ROTATION_KD);
  rotation.setKf(ROTATION_KF);
  rotation.setDeadZone(ROTATION_DEAD);
  delay(50);

  Serial.begin(9600);
  gyro.init();
  yaw0 = gyro.getAlpha();
  delay(500);

  handler.begin();
  delay(50);
  eepromHandler.defineVariable("P", sizeof(float));
  eepromHandler.defineVariable("I", sizeof(float));
  eepromHandler.defineVariable("D", sizeof(float));
  float K = 0;
  eepromHandler.getVariable("P", K);
  pid.setKp(K);
  Serial.println(K);
  eepromHandler.getVariable("I", K);
  pid.setKi(K);
  eepromHandler.getVariable("D", K);
  pid.setKd(K);
  handler.addCommand("P", [](String cmd) {float K = cmd.toFloat(); pid.setKp(K); eepromHandler.setVariable("P", K); showPID();});
  handler.addCommand("I", [](String cmd) {float K = cmd.toFloat(); pid.setKi(K); eepromHandler.setVariable("I", K); showPID();});
  handler.addCommand("D", [](String cmd) {float K = cmd.toFloat(); pid.setKd(K); eepromHandler.setVariable("D", K); showPID();});
  handler.addCommand("V", [](String cmd) {float v = cmd.toFloat(); motors.setMax(v); Serial.println(String(v) + " VELOCIDAD");});

  delay(500);
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
  }

  //Ojo quitarl
}

void loop() {
  handler.check();
  position = (int) qtrrc.readLine(sensorValues);

  //if(abs(3500-position) > 2000)
  //  motors.setMax(150);

//  turn90();

  pid.check();

  gyro.check();

}
