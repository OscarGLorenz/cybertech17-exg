#ifndef GLOBAL_CT_H
#define GLOBAL_CT_H

#include "Pinout.h"
#include "Arduino.h"
#include "DynamicStructures/Queue.h"
#include "Debug/Debug.h"

#define LOW_VOLTAGE 750

void start(void) {
  //LECTURA DEL VOTAJE DE LA LIPO
   int LiPo = analogRead(A1);
   //Valores medidos 7.5V 900 y 7.8V 940
   //Valor límite 6.25 750
   if (LiPo <= LOW_VOLTAGE) {
     pinMode(BLUE_LED, OUTPUT);
     for (int i = 0; i < 50; i++) {
       digitalWrite(BLUE_LED,HIGH);
       delay(50);
       digitalWrite(BLUE_LED,LOW);
       delay(50);
     }
   }
   //LECTURA DEL VOTAJE DE LA LIPO

   delay(50);
   Serial.begin(9600);

   //MENSAJE INICIAL
   if (LiPo <= LOW_VOLTAGE) {
     Serial.println("STATUS: LOW VOLTAGE");
     Serial.print("VOLTAGE: ");
     Serial.print(LiPo/900.0*7.5);
     Serial.println("V");
     delay(100);
     exit(0);
   } else {
     Serial.println("STATUS: READY");
   }
   Serial.print("VOLTAGE: ");
   Serial.print(LiPo/900.0*7.5);
   Serial.println("V");

    while (analogRead(A1) <= LOW_VOLTAGE);
   //MENSAJE INICIAL
}

void flag(void) { //Subida de bandera blanca
  unsigned int c = 0;
  while(c < 5) {
    delay(2);
    if(analogRead(FRONT_SHARP) < 200) c++;
  }
}

void ready(void) {//Llamar para hacer parpadear LED
  Serial.end();
  digitalWrite(BLUE_LED, 0);
  delay(100);
  digitalWrite(BLUE_LED, 1);
  delay(100);
  digitalWrite(BLUE_LED, 0);
  Serial.begin(9600);
}

int filterRead(char pin, unsigned long delays, int times) {//Filtro para leer
  int read = 0;
  for(int i = 0; i < times; i++) {
    read+=analogRead(pin);
    delay(delays);
  }
  return read/times;
}

#ifndef DIRS_
#define DIRS_
enum class Dirs {Front = 0, Left = 1, Back = 2, Right = 3};
#endif

class Sharps {
public:
    Sharps(unsigned char * FLBR, float lowpass) {
      for (int i = 0; i < 4; i++) pins[i] = FLBR[i];
      Kf = lowpass;
    }

    void check() {
      for (int i = 0; i < 4; i++)
        data[i] = data[i] + Kf * (analogRead(pins[i]) - data[i]);
    }

    int get(Dirs dir) {
      return data[(int) dir];
    }

private:
  float data[4];
  float Kf;
  unsigned char pins[4];
};

#define endl '\n';

template<class T>
HardwareSerial& operator<<(HardwareSerial& serial, const T& value) {
  serial.print(value);
  return serial;
}

#endif
