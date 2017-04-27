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

Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.6);



//PID STRAIGHT
#define STRAIGHT_KP 3
#define STRAIGHT_KI 0.12
#define STRAIGHT_KD 0.05
#define STRAIGHT_SAT 0.20
PID straight([]()-> double {return (yaw0-gyro.getAlpha()).get();}, [](double dir){motors.move(constrain(dir,-1.0,1.0), STRAIGHT_SAT);}, 1);
//PID STRAIGHT



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
Angle ref(0);
void testTurn(double value,bool right) {
  motors.setMax(250);
  long int tiempos = millis();
  motors.move(0,0);
  delay(100);
  Serial.println(value);
  //yaw0 = gyro.getAlpha();
  yaw0 = yaw0 + value;
  //Angle yawr = yaw0;

  Serial.println("Ang " + String(yaw0.get()) + " " + String(gyro.getAlpha().get()) + " " + String((yaw0-gyro.getAlpha()).get()));
  delay(2);

  int c = 0;
  while(c < 30 ) {
    //  yawr = gyro.getAlpha();
    //Salida por pantalla de ángulo actual y objetivo
    //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 10);

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    if(abs((yaw0 - gyro.getAlpha()).get()) < ROTATION_THRESHOLD) c++;
    if(millis() - tiempos > 4000 && abs((yaw0 - gyro.getAlpha()).get()) < 0.1) break;
    delay(2);

  }

  rotation.resetSumShaft();

  motors.move(0,0);
  delay(500);
  motors.setMax(140);

}

void showPID() { Serial.println("P\tI\tD\n" + String(pid.getKp()) + "\t" + String(pid.getKi()) + "\t" + String(pid.getKd()));}

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

  delay(50);
  for (int i = 0; i < 300; i++) {
    qtrrc.calibrate();
  }
  ready();
  while(millis() < 4000) {
    gyro.check();
    delay(2);
    yaw0 = gyro.getAlpha();
    sharps.check();
  }
  yaw0 = gyro.getAlpha();
  ready();
  flag();
  motors.setMax(140);
  ref = gyro.getAlpha();
}

long int mario = millis();
int rotonda;
long int tiempoDeriva = millis();
bool deriva = false;
long int obstaculo = millis();

void loop() {
  handler.check();
  position = (int) qtrrc.readLine(sensorValues);

  sharps.check();
  gyro.check();

  if(sharps.get(Dirs::Front) > 400 && millis() - obstaculo > 1500) {
    motors.move(0,-1); delay(250);
    motors.rotate((rand()%2 ? 1 : -1)); delay(100);
    motors.move(0,1); delay(300);
    obstaculo = millis();
    int value = 0;
    yaw0 = gyro.getAlpha();
    position = (int) qtrrc.readLine(sensorValues);

    do {
        value = 0;
        position = (int) qtrrc.readLine(sensorValues);
        for (int i = 0; i < 8 ; i++)  value+=sensorValues[i];
        motors.move(0,1);
        delay(2);
    } while (value < 1000);
  }
  pid.check();

  delay(2);
}
