#include <SC_PlugIn.h>

#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"
#include "Adafruit_Simple_AHRS.h"


// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book

static InterfaceTable *ft;

struct AHRS : public Unit {
    static const int ROLL = 0;
    static const int PITCH = 1;
    static const int HEADING = 2;


    int channel = ROLL;
};


class AHRS_Singleton {
public:
    static AHRS_Singleton* get() {
      if (AHRS_Singleton::instance == nullptr) {
        AHRS_Singleton::instance = new AHRS_Singleton();
      }
      return AHRS_Singleton::instance;
    }

    public float read(int which) {
      // TODO: update this in a thread or something..
      ahrs.getOrientation(&orientation);

      
      switch (which) {
        case ROLL:
          return orientation.roll;
        case PITCH:
          return orientation.pitch;
        case HEADING:
          return orientation.heading;
      }
    }

private:
    static AHRS_Singleton* instance = nullptr;
    Adafruit_LSM9DS0 lsm;
    sensors_vec_t   orientation;
    Adafruit_Simple_AHRS* ahrs;

    AHRS_Singleton()  {
      lsm.begin()
      ahrs = new Adafruit_Simple_AHRS(&lsm.getAccel(), &lsm.getMag());
      orientation.roll = 0;
      orientation.pitch = 0;
      orientation.heading = 0;
    }

    ~AHRS_Singleton() {
      delete ahrs;
    }
};



extern "C" {
  void load(InterfaceTable *inTable);
  void AHRS_Ctor(AHRS *unit);
  void AHRS_Next(AHRS *unit, int numSamples);
}

void AHRS_Ctor(AHRS *unit) {
    unit->channel = static_cast<int>(IN0(1));

    SETCALC(AHRS_Next);
    AHRS_Next(unit, 1);
}

void AHRS_Next(AHRS *unit, int numSamples) {
  float *out = OUT(0);

  // TODO: this should probably be done differently
  float value = AHRS_Singleton.get().read(unit->channel);

  for (int i = 0; i < FULLBUFLENGTH; i++) {
    out[i] = value;
  }
}

void load(InterfaceTable* inTable) {
  ft = inTable;
  DefineSimpleUnit(AHRS);
}
