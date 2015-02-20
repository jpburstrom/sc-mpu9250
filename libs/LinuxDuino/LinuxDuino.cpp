#include "LinuxDuino.h"

#include <iostream>
#include <thread>

// i2c stuff
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace LinuxDuino;

namespace {
  const auto start_time = std::chrono::steady_clock::now();
};

uint64_t LinuxDuino::millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
}


// http://stackoverflow.com/questions/4184468/sleep-for-milliseconds Feb 19, 2015
void LinuxDuino::delay(uint64_t mill) {
  std::this_thread::sleep_for(std::chrono::milliseconds(mill));
}

#include <linux/i2c-dev.h>

// written partly with reference to sample code from http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c/dev-interface (accessed Feb 8 2015)

LinuxDuino::Wire::Wire(int num) {
  char filename[20];
  snprintf(filename, 19, "/dev/i2c-%d", num);

  _file = open(filename, O_RDWR);
  if (_file < 0) {
    std::cerr << "failed to open i2c file: " << filename << std::endl;
    exit(1);
  }
}

LinuxDuino::Wire::~Wire() {
  close(_file);
}

void LinuxDuino::Wire::begin() {
}

void LinuxDuino::Wire::beginTransmission(byte address) {
  if (ioctl(_file, I2C_SLAVE, address) < 0) {
    std::cerr << "failed to set i2c slave " << address << std::endl;
  }
}

void LinuxDuino::Wire::write(byte reg, byte value) {
  if (i2c_smbus_write_byte_data(_file, reg, value) < 0) {
    std::cerr << "failed to write to i2c register" << reg << std::endl;
  }
}

uint8_t LinuxDuino::Wire::read(byte reg) {
  return i2c_smbus_read_byte_data(_file, reg);
}

void LinuxDuino::Wire::endTransmission() { 
}

void LinuxDuino::Wire::requestFrom(byte reg, byte* data, byte len) { 
  auto count = i2c_smbus_read_i2c_block_data(_file, reg, len, data);
  if (count != len) {
    std::cerr << "missing data from block read. wanted " << len << " got " << count << std::endl;
  }
}
