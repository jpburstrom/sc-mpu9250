#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Simple_AHRS.h"

#include "debug.h"


#include <SC_PlugIn.h>

// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015


struct AHRS : public Unit {
    static const int ROLL = 0;
    static const int PITCH = 1;
    static const int HEADING = 2;


    int channel = ROLL;
};

// PLUGIN INTERFACE

extern "C" {
  void AHRS_Ctor(AHRS *unit);
  void AHRS_next_k(AHRS *unit, int numSamples);
}


static InterfaceTable *ft;

PluginLoad(AHRS)
{
  debug("PluginLoad")
  ft = inTable;
  DefineSimpleUnit(AHRS);
}


// PLUGIN IMPLEMENTATION


class AHRS_Singleton {
public:
    static AHRS_Singleton* get() {
      if (AHRS_Singleton::instance == nullptr) {
        AHRS_Singleton::instance = new AHRS_Singleton();
      }
      return AHRS_Singleton::instance;
    }

    float read(int which) {
      // TODO: update this in a thread or something..
      ahrs->getOrientation(&orientation);

      
      switch (which) {
        case AHRS::ROLL:
          return orientation.roll;
        case AHRS::PITCH:
          return orientation.pitch;
        case AHRS::HEADING:
        default:
          return orientation.heading;
      }
    }

private:
    static AHRS_Singleton* instance;

    Adafruit_LSM9DS0 lsm;
    sensors_vec_t   orientation;
    Adafruit_Simple_AHRS* ahrs;

    AHRS_Singleton()  {
      debug("AHRS_Singleton ctor")

      lsm.begin();
      ahrs = new Adafruit_Simple_AHRS(&lsm.getAccel(), &lsm.getMag());
      orientation.roll = 0;
      orientation.pitch = 0;
      orientation.heading = 0;
    }

    ~AHRS_Singleton() {
      delete ahrs;
    }
};

AHRS_Singleton* AHRS_Singleton::instance = nullptr;


void AHRS_Ctor(AHRS *unit) {
    debug("AHRS_Ctor");

    unit->channel = static_cast<int>(IN0(0));

    SETCALC(AHRS_next_k);
    AHRS_next_k(unit, 1);
}


void AHRS_next_k(AHRS *unit, int numSamples) {
  debug("AHRS_next_k");
  float* out = OUT(0);

  // TODO: this should probably be done differently
  float value = AHRS_Singleton::get()->read(unit->channel);

  for (int i = 0; i < numSamples ; i++) {
    out[i] = value;
  }
}
