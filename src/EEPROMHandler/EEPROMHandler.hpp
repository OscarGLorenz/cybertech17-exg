#include "EEPROM.h"
#include "Arduino.h"
#include "../DynamicStructures/Queue.h"

//Bloque para guardar nombre de la variable y posición
struct EEPROMBlock {
  EEPROMBlock(){}
  EEPROMBlock(String name, int position) {
    str = name;
    pos = position;
  }
  String getString() {
    return str;
  }
  int getPos() {
    return pos;
  }
private:
    String str;
    int pos;
};

class EEPROMHandler {
public:
  //Busca en blocks uno con el mismo nombre, coge la posición y devuelve la var
  template<typename T>  T getVariable(String name, T &var) {
      for(int i = 0; i < blocks.size(); i++) {
        if (blocks.get(i).getString().equalsIgnoreCase(name)) {
          	EEPROM.get(blocks.get(i).getPos(), var);
        }
      }
      return var;
  }

  //Busca en blocks uno con el mismo nombre, coge la posición y actualiza la var
  template<typename T> void setVariable(String name, const T var) {
    for(int i = 0; i < blocks.size(); i++) {
      if (blocks.get(i).getString().equalsIgnoreCase(name)) {
          EEPROM.put(blocks.get(i).getPos(), var);
      }
    }
  }

//Añade un bloque al vector, aumenta el tamaño
 void defineVariable(String name, int lenght) {
     blocks.pushBack(EEPROMBlock(name, size));
     size += lenght;
  }

private:
  Queue<EEPROMBlock> blocks;
  int size;
};
