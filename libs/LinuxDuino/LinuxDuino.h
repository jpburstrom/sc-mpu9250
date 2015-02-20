#ifndef __LINUXDUINO_H
#define __LINUXDUINO_H


#include <chrono>

namespace LinuxDuino {

typedef uint8_t byte;

uint64_t millis();
void delay(uint64_t mill);


class Wire {
public:
          Wire(int num);
          ~Wire();

  void    begin();
  void    beginTransmission(byte address);
  void    write(byte reg, byte value);
  void    endTransmission();
  void    requestFrom(byte reg, byte* data, byte len);
  uint8_t read(byte reg);

private:
  int     _file;
};


};

#endif
