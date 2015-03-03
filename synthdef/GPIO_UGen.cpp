#include "debug.h"


#include <SC_PlugIn.h>
#include <wiringPi.h>


// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015
//
// and also wiringpi.com/examples/blink  accessed March 2, 2015
// and eventually wiringPi.c from wiringPi


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


struct GPIO : public Unit {
  int pin;
  int fd;
};


// PLUGIN INTERFACE

extern "C" {
  void GPIO_Ctor(GPIO *unit);
  void GPIO_Dtor(GPIO *unit);
  void GPIO_next(GPIO *unit, int numSamples);
}



static InterfaceTable *ft;

PluginLoad(GPIO)
{
  debug("PluginLoad GPIO");
  ft = inTable;
  DefineDtorUnit(GPIO);

  wiringPiSetupSys(); // use sysfs interface
}


void GPIO_Ctor(GPIO *unit) {
    debug("GPIO_Ctor");

    unit->pin = static_cast<int>(IN0(0));
    // assumes pin is already set up
    char fname[64];
    sprintf(fname, "/sys/class/gpio/gpio%d/value", unit->pin);
    unit->fd = open(fname, O_RDONLY);

    std::cout << fname << " fd: " << unit->fd << " " << errno << std::endl;
    if (unit->fd <= 0) {
      exit(1);
    }

    SETCALC(GPIO_next);
    GPIO_next(unit, 1);
}

void GPIO_Dtor(GPIO *unit) {
    close(unit->fd);
}


void GPIO_next(GPIO *unit, int numSamples) {
  float* out = OUT(0);

  char c = 'a';
  lseek(unit->fd, 0, SEEK_SET);
  auto err = read(unit->fd, &c, 1);
  if (err != 1) {
    std::cout << "c : " << c << "err: " << err << " errno: " << errno << std::endl;
  }

  float value = 1;
  if (c == '0') {
    value = 0;
  }
  value *= 1 + value * -2;
  // float value = digitalRead(unit->pin) * -2 + 1;
  // float value = digitalRead(unit->pin);* -2 + 1;
  // HIGH -> -1, LOW = 1 (this is how triggers work in SC)

  for (int i = 0; i < numSamples ; i++) {
    out[i] = value;
  }
}
