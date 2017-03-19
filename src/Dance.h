#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"

#include "Global.h"

#include "QTRSensors.cpp"

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);



//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);



//Motores
#define DUTY 30
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);



//Sensor de línea
QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR,PIN2_QTR, PIN3_QTR, PIN4_QTR,
   PIN5_QTR, PIN6_QTR, PIN7_QTR,PIN8_QTR}, 8, 2500);
unsigned int sensorValues[8];
int position = 3500;




//PID ROTATION
//Lectura de error de girar
double inputR() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de girar
void outputR(double dir) {
  motors.rotate(1.0, -constrain(dir,-1,1));
}
#define ROTATION_KP 0.2
#define ROTATION_KI 0.4
#define ROTATION_KD 0.01
PID rotation(inputR,outputR,1,1);
//PID ROTATION




//PID STRAIGHT
//Lectura de error de ir recto
double inputS() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de ir recto
void outputS(double dir) {
  motors.move(-constrain(dir,-1.0,1.0), -1);
}
#define STRAIGHT_KP 1.2
#define STRAIGHT_KI 0
#define STRAIGHT_KD 0.02 //10
PID straight(inputS,outputS,1,1);
//PID STRAIGHT




//PID SIGUELINEAS
double input(void) {
  return (position - 3500.0)/3500.0;
}

void output(double dir) {
  motors.move(constrain(dir,-1.0,1.0), 1.0);
}

PID pid(input,output);
//PID SIGUELINEAS




//MOVIMIENTOS
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
  yaw0 = gyro.getAlpha();

  double distance = 0;
  double velocity = 0;
  while(abs(distance) < 0.25) {
    gyro.check();

    straight.check();

    velocity+=gyro.aa.x/350*0.002;
    distance+=velocity*0.002;
    delay(2);
  }
  motors.rotate(0,0);

}
//MOVIMIENTOS




//CONTADOR DE LÍNEAS
bool overSlash = false; //Variable para situacion de linea
int count = 0; //Conteo de lineas
Queue<unsigned long> data; //Guardar milis

void execute() {
  Queue<int> mov; //Guardamos movimientos

  QueueIterator<unsigned long> itr_data = data.getIterator(); //Iterador para data

  unsigned long ref = itr_data.next(); //Valor de referencia para comparar

  int count2 = 1; //Contador de líneas

  while(itr_data.hasNext()) { //Iteramos en data
    unsigned long now = itr_data.next(); //Cogemos el siguiente valor

    if((now-ref) < 300) { //Si now y ref estan cerca sumamos 1
      count2++;
    } else { //Si no, paramos de sumar y guardamos la cantidad
      mov.pushBack(count2);
      Serial.println(count2);
      count2 = 1;
    }

    if(!itr_data.hasNext()) { //Guardamos la última cantidad de líneas
      mov.pushBack(count2);
      Serial.println(count2);
    }

    ref = now; //Actualizamos la referencia
  }

  //TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
  delay(500);
  exit(0);

  QueueIterator<int> itr_mov = mov.getIterator(); //Iterador de movimientos

    while(itr_mov.hasNext()) { //Iteramos por los movimientos y los hacemos
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

    while(1) {motors.move(0,0);} //Detenemos el robot

}

void read() {//Contar lineas

  int sum = sensorValues[2] + sensorValues[5]; //Sumar sensores 3 y 6

  if (!overSlash) { //¿Estamos sobre la linea?
    if (sum > 1200) { //Si su suma es grande estamos sobre una linea

      overSlash = true; //Ahora estamos sobre la línea

      Serial.println(millis()); //Milisegundos actuales
      data.pushBack(millis()); //Guardamos los millis

      count++; //Sumamos 1 al contador

      if(count >= 10) { //TODO Condición para terminar de contar
        motors.move(0, 0); //Detenerse
        execute(); //Ejecutar acciones
      }

    }
  } else {
    if (sum < 1200) { //Si salimos de la línea ponemos que estamos fuera
      overSlash = false;
    }
  }
}
//CONTADOR DE LÍNEAS





void setup(){
 start();//Nivel bateria, +++



 //CONSTANTES PID
 straight.setKp(STRAIGHT_KP);
 straight.setKd(STRAIGHT_KD);

 pid.setKp(2);

 rotation.setKp(ROTATION_KP);
 rotation.setKi(ROTATION_KI);
 rotation.setKd(ROTATION_KD);
 //CONSTANTES PID



//CALIBRACIÓN
Serial.println("QTR-8RC Calibration");
 delay(500);
 for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
}
Serial.println("Calibration finished");
//CALIBRACIÓN



//Inicio giroscopio, esperamos para que se estabilice la salida
gyro.init();
//Ángulo inicial
yaw0 = gyro.getAlpha();

}

void loop() {
  position = (int) qtrrc.readLine(sensorValues); //Leer línea
  pid.check(); //Ejecutar pid linea
  read(); //Leer líneas cruzadas

  gyro.check();//Leer giroscopio
  
  delay(2);
}
