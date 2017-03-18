#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"

#include "QTRSensors.cpp"

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);

//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);

#define DUTY 30
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

//PID ROTATION
//Lectura de error de girar
double inputR() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de girar
void outputR(double dir) {
  motors.rotate(1.0, -constrain(dir,-0.4,0.4));
}
#define ROTATION_KP 0.2
#define ROTATION_KI 0.4
#define ROTATION_KD 0.01
PID rotation(inputR,outputR,1,0.4);
//PID ROTATION


QTRSensorsRC qtrrc((unsigned char[]) {PIN2_QTR, PIN3_QTR, PIN4_QTR,
   PIN5_QTR, PIN6_QTR, PIN7_QTR}, 6, 2500);
unsigned int sensorValues[8];
int position = 3500;

double input(void) {
  return (position - 3500.0)/3500.0;
}

void output(double dir) {
  motors.move(dir, 1.0);
}

PID pid(input,output);

void turnLeft() {
  yaw0 = gyro.getAlpha();
  yaw0 = yaw0 - M_PI_2;

  //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
  int c = 0;
  while(c < 10) {
    //Salida por pantalla de ángulo actual y objetivo
    //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);
    Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    delay(2);

    //Si el error es menor que RAD_TOLERANCE sumamos
    if(abs((yaw0 - gyro.getAlpha()).get()) < 0.05) c++;
  }
}

void turnRight() {
  yaw0 = gyro.getAlpha();
  yaw0 = yaw0 + M_PI_2;

  //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
  int c = 0;
  while(c < 10) {
    //Salida por pantalla de ángulo actual y objetivo
    //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);
    Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    delay(2);

    //Si el error es menor que RAD_TOLERANCE sumamos
    if(abs((yaw0 - gyro.getAlpha()).get()) < 0.05) c++;
  }
}

void turnBack() {
  yaw0 = gyro.getAlpha();
  yaw0 = yaw0 + M_PI;

  //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
  int c = 0;
  while(c < 10) {
    //Salida por pantalla de ángulo actual y objetivo
    //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);
    Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    delay(2);

    //Si el error es menor que RAD_TOLERANCE sumamos
    if(abs((yaw0 - gyro.getAlpha()).get()) < 0.05) c++;
  }
}

void goBack() {

}


void execute();

bool overSlash = false;
int count = 0;
Queue<unsigned long> data;

void read() {
  int sum = sensorValues[2] + sensorValues[5];
  if (!overSlash) {
    if (sum > 800) {
      overSlash = true;
      Serial.println(millis());
      data.pushBack(millis());
      count++;
      if(count >= 10) {
        motors.move(0, 0);
        execute();
      }
    }
  } else {
    if (sum < 800) {
      overSlash = false;
    }
  }
}

void execute() {
  Queue<int> mov;

  QueueIterator<unsigned long> itr_data = data.getIterator();

  unsigned long ref = itr_data.next();

  int count2 = 1;
  while(itr_data.hasNext()) {
    unsigned long now = itr_data.next();
    if((now-ref) < 500) {
      count2++;
    } else {
      mov.pushBack(count2);
      Serial.println(count2);
      count2 = 1;
    }

    if(!itr_data.hasNext()) {
      mov.pushBack(count2);
      Serial.println(count2);
    }

    ref = now;
  }

  QueueIterator<int> itr_mov = mov.getIterator();

    while(itr_mov.hasNext()) {
      switch(itr_mov.next()) {
        case 1:
          turnBack();
        break;
        case 2:
          goBack();
        break;
        case 3:
          turnRight();
        break;
        case 4:
          turnLeft();
        break;
      }
    }

    while(1) {motors.move(0,0);}

}

void setup(){

//LECTURA DEL VOTAJE DE LA LIPO
 int LiPo = analogRead(A1);
 //Valores medidos 7.5V 900 y 7.8V 940
 //Valor límite 6.25 750
 if (LiPo <= 750) {
   pinMode(BLUE_LED, OUTPUT);
   for (int i = 0; i < 50; i++) {
     digitalWrite(BLUE_LED,HIGH);
     delay(50);
     digitalWrite(BLUE_LED,LOW);
     delay(50);
   }
 }
 //LECTURA DEL VOTAJE DE LA LIPO

 Serial.begin(9600);
 delay(50);

 //MENSAJE INICIAL
 if (LiPo <= 750) {
   Serial.println("STATUS: LOW VOLTAGE");
 } else {
   Serial.println("STATUS: READY");
 }
 Serial.print("VOLTAGE: ");
 Serial.print(LiPo/900.0*7.5);
 Serial.println("V");
 //MENSAJE INICIAL

 pid.setKp(1.5);
 rotation.setKp(ROTATION_KP);
 rotation.setKi(ROTATION_KI);
 rotation.setKd(ROTATION_KD);

 delay(500);
 for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
}

//Inicio giroscopio, esperamos para que se estabilice la salida
gyro.init();
while(millis() < 4000) {
  gyro.check();
  delay(2);
}

//Ángulo inicial
yaw0 = gyro.getAlpha();

}

void loop() {
  position = (int) qtrrc.readLine(sensorValues);
  pid.check();
  read();
  gyro.check();

}
