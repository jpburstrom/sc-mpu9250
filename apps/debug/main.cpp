#include <MPU9250.h>

#include <signal.h>
#include <unistd.h>
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

  
  MPU9250 mpu;


  if (!mpu.init()) {
    std::cerr <<  "sensor init failed!" << std::endl;
    return 1;
  } else {
      cout << "Init ok\n";
  }
  
  if (!mpu.testRW()) {
    std::cerr <<  "Can't write to i2c device!" << std::endl;
    return 1;
  } else {
      cout << "R/W OK\n";
  }

  usleep(1000*1000);

  while (running) {
    mpu.read();
    std::cout << "Accel X: " << mpu.accel.x
       << " Y: " << mpu.accel.y
       << " Z: " << mpu.accel.z
       << "\t Mag X: " << mpu.mag.x
       << " Y: " << mpu.mag.y
       << " Z: " << mpu.mag.z
       << "\t Gyro X: " << mpu.gyro.x
       << " Y: " << mpu.gyro.y
       << " Z: " << mpu.gyro.z
       << "\t Temp: "  << mpu.temperature << std::endl;

    usleep(100000);
  }
}
