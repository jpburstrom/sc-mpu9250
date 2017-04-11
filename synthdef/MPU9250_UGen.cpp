#include "MPU9250.h"
#include "debug.h"

#include <SC_PlugIn.h>
#include <thread>


// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015


struct MPU : public Unit {
    //Outputs 3 values
    static const int ACCEL = 0;
    static const int GYRO = 1;
    static const int MAG = 2;
    static const int ORIENTATION = 3;
    //Outputs single values
    static const int PITCH  = 4;
    static const int ROLL = 5;
    static const int YAW = 6;

    int channel = ACCEL;
};

// PLUGIN INTERFACE

extern "C" {
  void MPU_Ctor(MPU *unit);
  void MPU_next_k1(MPU *unit, int numSamples);
  void MPU_next_k3(MPU *unit, int numSamples);
}


static InterfaceTable *ft;

PluginLoad(MPU)
{
  debug("PluginLoad")
  ft = inTable;
  DefineSimpleUnit(MPU);
}


// PLUGIN IMPLEMENTATION


class MPU_Singleton {
public:
    static MPU_Singleton* get() {
      if (MPU_Singleton::instance == nullptr) {
        MPU_Singleton::instance = new MPU_Singleton();
        MPU_Singleton::thread = new std::thread(MPU_Singleton::thready);
      }
      return MPU_Singleton::instance;
    }

    static void thready() {
      while (MPU_Singleton::running) {
        MPU_Singleton::get()->update();
        std::this_thread::yield();
      }
    }

    static bool running;

    void update() {
      //FIXME
      mpu.read();
      //ahrs->getOrientation(&orientation);
    }

    float read1(int which) {
      switch (which) {
        case MPU::ROLL:
          return mpu.orientation.roll;
        case MPU::YAW:
          return mpu.orientation.yaw;
        case MPU::PITCH:
        default:
          return mpu.orientation.pitch;
      }
    }

    void read3(int which, float (&valueOut)[3]) {
      switch (which) {
        case MPU::ACCEL:
            valueOut[0] = mpu.accel.x;
            valueOut[1] = mpu.accel.y;
            valueOut[2] = mpu.accel.z;
        case MPU::GYRO:
            valueOut[0] = mpu.gyro.x;
            valueOut[1] = mpu.gyro.y;
            valueOut[2] = mpu.gyro.z;
        case MPU::MAG:
            valueOut[0] = mpu.mag.x;
            valueOut[1] = mpu.mag.y;
            valueOut[2] = mpu.mag.z;
        case MPU::ORIENTATION:
            valueOut[0] = mpu.orientation.pitch;
            valueOut[1] = mpu.orientation.roll;
            valueOut[2] = mpu.orientation.yaw;

      }
    }

private:
    static MPU_Singleton* instance;
    static std::thread* thread;

    //Instance of I2C class
    MPU9250 mpu;

    MPU_Singleton()  {
      debug("MPU_Singleton ctor")
      mpu.init();
    }

    ~MPU_Singleton() {
        //TODO delete resources
    }
};

MPU_Singleton* MPU_Singleton::instance = nullptr;
std::thread* MPU_Singleton::thread = nullptr;
bool MPU_Singleton::running = true;


void MPU_Ctor(MPU *unit) {
    //debug("MPU_Ctor");

    unit->channel = static_cast<int>(IN0(0));

    if (unit->channel < 4) {
        SETCALC(MPU_next_k3);
        MPU_next_k3(unit, 1);
    } else {
        SETCALC(MPU_next_k1);
        MPU_next_k1(unit, 1);
    }
}

//Output 1 single value
void MPU_next_k1(MPU *unit, int numSamples) {
  //debug("MPU_next_k");
  float* out = OUT(0);

  // TODO: this should probably be done differently
  
  float value = MPU_Singleton::get()->read1(unit->channel);

  for (int i = 0; i < numSamples ; i++) {
    out[i] = value;
  }
}

void MPU_next_k3(MPU *unit, int numSamples) {
  //debug("MPU_next_k");
  float* out[3];
  for (int i = 0; i < 3; i++) {
      out[i] = OUT(i);
  }

  // TODO: this should probably be done differently
  float value[3];
  MPU_Singleton::get()->read3(unit->channel, value);

  for (int i = 0; i < numSamples ; i++) {
      for (int o = 0; i < 3; o++) {
        out[o][i] = value[o];
      }
  }
}
