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

#include <thread>
#include <chrono>


struct GPIOThread;

struct GPIO : public Unit {
  int pin;
  int fd;
  float value;
  GPIOThread* thread;
};


// PLUGIN INTERFACE

extern "C" {
  void GPIO_Ctor(GPIO *unit);
  void GPIO_Dtor(GPIO *unit);
  void GPIO_next(GPIO *unit, int numSamples);
}

struct GPIOThread {
  std::thread* thread;
  bool running;


  void join() {
    running = false;
    thread->join();
    delete thread;
  }

  void run(GPIO* unit) {
    running = true;
    thread = new std::thread(&GPIOThread::loop, this, unit);
  }

  void loop(GPIO* unit) {
    while (running) {
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

        unit->value = 1 + value * -2;
        // HIGH -> -1, LOW = 1 (this is how triggers work in SC)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
  }
};


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

    unit->value = 0;
    unit->thread = new GPIOThread();
    unit->thread->run(unit);

    SETCALC(GPIO_next);
    GPIO_next(unit, 1);
}

void GPIO_Dtor(GPIO *unit) {
    unit->thread->join();
    close(unit->fd);
}


void GPIO_next(GPIO *unit, int numSamples) {
  float* out = OUT(0);

  for (int i = 0; i < numSamples ; i++) {
    out[i] = unit->value;
  }
}
