#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"

#include <signal.h>
#include <iostream>

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
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

  while (running) {
    lsm.read();
    std::cout << "Accel X: " << lsm.accelData.x
       << " Y: " << (int)lsm.accelData.y
       << " Z: " << lsm.accelData.z
       << "Mag X: " << lsm.magData.x
       << " Y: " << lsm.magData.y
       << " Z: " << lsm.magData.z
       << "Gyro X: " << lsm.gyroData.x
       << " Y: " << lsm.gyroData.y
       << " Z: " << lsm.gyroData.z
       << "Temp: "  << lsm.temperature << std::endl;

    delay(200);
  }
}
