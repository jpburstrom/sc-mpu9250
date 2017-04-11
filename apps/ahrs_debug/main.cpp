#include "MPU9250.h"
#include "debug.h"

#include <signal.h>
#include <iostream>


// based on adafruit's example code at
// https://github.com/adafruit/Adafruit_AHRS/blob/master/examples/ahrs_lsm9ds0/ahrs_lsm9ds0.ino/

bool running = true;

void kill_handler(int s) {
  debug("kill_handler");
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

  MPU9250 mpu;
  if (!mpu.init()) {
    std::cerr <<  "sensor init failed!" << std::endl;
    return 1;
  }

  while (running) {
      mpu.read();
  std::cout << "Roll:"
    << mpu.orientation.roll
    << "\tPitch:" << mpu.orientation.pitch
    << "\tYaw:" << mpu.orientation.yaw << std::endl;

    usleep(100000);

  }
}
