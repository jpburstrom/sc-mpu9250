#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Simple_AHRS.h"

#include <signal.h>
#include <iostream>


// based on adafruit's example code at
// https://github.com/adafruit/Adafruit_AHRS/blob/master/examples/ahrs_lsm9ds0/ahrs_lsm9ds0.ino/

bool running = true;

void kill_handler(int s) {
  running = false;
}

int main(int argc, char** argv) {

  // setup ctrl-c handler
  // code from http://stackoverflow.com/questions/1641182/how-can-i-catch-a-ctrl-c-event-c
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = kill_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  Adafruit_LSM9DS0 lsm;
  if (!lsm.begin()) {
    std::cerr <<  "sensor init failed!" << std::endl;
    return 1;
  }

  Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
  sensors_vec_t   orientation;

  while (running) {
    if (ahrs.getOrientation(&orientation)) {
      std::cout << "Orientation{ r:"
        << orientation.roll
        << " p:" << orientation.pitch
        << " h:" << orientation.heading << " }" << std::endl;
    }

    LinuxDuino::delay(100);

  }
}
