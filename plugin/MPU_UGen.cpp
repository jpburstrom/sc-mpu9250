#include <pthread.h>
#include <atomic>
#include "SC_Lock.h"
#include "SC_PlugIn.h"

#include "MPU9250.h"
#include "debug.h"

// written with reference to the chapter "Writing Unit Generator Plug-ins" in The SuperCollider Book
// and also http://doc.sccode.org/Guides/WritingUGens.html accessed March 2, 2015
//

//Read interval
#define INTERVAL 2.5
#define IDLE_INTERVAL 100

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
    int outputs = 3;

};

struct MPUCmdData {
    //Current data state
    //Filename for save/load operations
    FILE *fp;
    mpu9250Calibration_t calData;
    char *filename;
};  

mpu9250State_t gData = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

enum mpuTask {
    TASK_STOP = 0,
    TASK_RUN=2,
    TASK_CMD=8,
    TASK_CALIBRATE_ACCEL_GYRO=9,
    TASK_CALIBRATE_MAG=10,
    TASK_SAVE=11,
    TASK_LOAD=12
};

//These should correspond with class methods
enum mpuCmd {
    CMD_CALIBRATE_ACCEL_GYRO = 1,
    CMD_CALIBRATE_MAG,
    CMD_SAVE,
    CMD_LOAD
};

//Current command
std::atomic_int currentTask = { TASK_STOP };


// PLUGIN INTERFACE

extern "C" {
    void MPU_Ctor(MPU *unit);
    //void MPU_next_k1(MPU *unit, int numSamples);
    void MPU_next_k(MPU *unit, int numSamples);
}


static InterfaceTable *ft;

// PLUGIN IMPLEMENTATION
MPU9250 mpu;

void MPU_Ctor(MPU *unit) {
    unit->channel = static_cast<int>(IN0(0));
    unit->outputs = (unit->channel > MPU::ORIENTATION) ? 1 : 3;

    SETCALC(MPU_next_k);
    MPU_next_k(unit, 1);
}

void MPU_next_k(MPU *unit, int numSamples) {

    float value[3] = {0,0,0};
    int outputs = unit->outputs;

    if (currentTask == TASK_RUN) {
        switch (unit->channel) {
            case MPU::GYRO:
                value[0] = gData.gx;
                value[1] = gData.gy;
                value[2] = gData.gz;
                break;
            case MPU::MAG:
                value[0] = gData.mx;
                value[1] = gData.my;
                value[2] = gData.mz;
                break;
            case MPU::ORIENTATION:
                value[0] = gData.pitch;
                value[1] = gData.roll;
                value[2] = gData.yaw;
                break;
            case MPU::PITCH:
                value[0] = gData.pitch;
                break;
            case MPU::ROLL:
                value[0] = gData.roll;
                break;
            case MPU::YAW:
                value[0] = gData.yaw;
                break;
            case MPU::ACCEL:
            default:
                value[0] = gData.ax;
                value[1] = gData.ay;
                value[2] = gData.az;
                break;
        }
    }

    for (int i = 0; i < numSamples ; i++) {
        for (int o = 0; o < outputs; o++) {
            OUT(o)[i] = value[o];
            //OUT(0) = 0;
            //OUT(1) = 0;
            //OUT(2) = 0;
        }
    }
        //    printf("roll:%f", gData.roll);
}


//Second stage, non-RT
bool cmdStage2(World* world, void* inUserData) {
    MPUCmdData* data = (MPUCmdData*)inUserData;
    switch ( currentTask.load( std::memory_order_relaxed ) ) {
        case TASK_CALIBRATE_ACCEL_GYRO:

            printf("MPU: Calibrating Accelerometer & Gyro\n");
            
            mpu.calibrateAccelGyro();
            currentTask = TASK_RUN;
            //TODO: print calibration stats
            break;
        
        case TASK_CALIBRATE_MAG:

            printf("MPU: Calibrating Magnetometer\n");
            printf("MPU: Wave device in a figure eight until done!\n");

            mpu.calibrateMag();
            //TODO: print calibration stats
            currentTask = TASK_RUN;
            break;

        case TASK_SAVE:
            data->fp = fopen(data->filename, "wb");
            if (data->fp != NULL) {
                mpu.getCalibration(data->calData);
                fwrite(&(data->calData), sizeof(mpu9250Calibration_t), 1, data->fp);
                fclose(data->fp);
            } else {
                printf("MPU: Couldn't open file for writing\n");
            }
            currentTask = TASK_RUN;
            break;

        case TASK_LOAD:
            data->fp = fopen(data->filename, "rb");
            if (data->fp != NULL) {
                fread(&(data->calData), sizeof(mpu9250Calibration_t), 1, data->fp);
                fclose(data->fp);
                mpu.setCalibration(data->calData);
            } else {
                printf("MPU: Couldn't open file for reading\n");
            }
            currentTask = TASK_RUN;
            break;
    }
    return true;
}

//Synchronous, sends completion message
bool cmdStage3(World* world, void* inUserData) {
    debug("Stage 3");
    Print("Command completed\n");
    return true;
}


void cmdCleanup(World* world, void* inUserData) {
    debug("Cleanup");
    MPUCmdData* cmdData = (MPUCmdData*)inUserData;
    RTFree(world, cmdData->filename);
    RTFree(world, cmdData);
}

void mpuCmdFunc(World *inWorld, void* inUserData, struct sc_msg_iter *args, void *replyAddr) {
    
    //Allocate a new instance of MPUCmdData
    MPUCmdData* data = (MPUCmdData*)RTAlloc(inWorld, sizeof(MPUCmdData));
    //Empty
    data->filename = 0;

    //Get arguments: First arg is command type
    int cmd = args->geti();
    //Second arg is a string, which might be a filename.
    const char *filename = args->gets();

    if (filename) {
        data->filename = (char*)RTAlloc(inWorld, strlen(filename)+1); // allocate space, free it in cmdCleanup.
        strcpy(data->filename, filename); // copy the string
    }

    int msgSize = args->getbsize();
    char* msgData = 0;
    if (msgSize) {
        // allocate space for completion message
        // scsynth will delete the completion message for you.
        msgData = (char*)RTAlloc(inWorld, msgSize);
        args->getb(msgData, msgSize); // copy completion message.
    }

    switch(cmd) {
        case CMD_CALIBRATE_ACCEL_GYRO:
            currentTask = TASK_CALIBRATE_ACCEL_GYRO;
            break;
        case CMD_CALIBRATE_MAG:
            currentTask = TASK_CALIBRATE_MAG;
            break;
        case CMD_SAVE:
            currentTask = TASK_SAVE;
            break;
        case CMD_LOAD:
            currentTask = TASK_LOAD;
            break;
        default:
            printf("MPU: No such command\n");
    }


    if ((currentTask & TASK_CMD) == TASK_CMD) {
        DoAsynchronousCommand(inWorld, replyAddr, "mpuCmd", (void*)data, (AsyncStageFn)cmdStage2, (AsyncStageFn)cmdStage3, NULL, cmdCleanup, msgSize, msgData);
    }

}

//Threading stuff

void *gstate_update_func(void *param) {
    if ( mpu.init() ) {
        while ( currentTask.load( std::memory_order_relaxed ) != TASK_STOP ) {
            while ( currentTask.load( std::memory_order_relaxed ) == TASK_RUN ) {
                mpu.read(gData);
                std::this_thread::sleep_for( std::chrono::duration<float, std::milli>(INTERVAL)  );
            }
            while ( (currentTask.load( std::memory_order_relaxed ) & TASK_CMD) == TASK_CMD  ) {
                std::this_thread::sleep_for( std::chrono::milliseconds( IDLE_INTERVAL ) );
            }
            
        }
    }
    //We need to return something
    return NULL;
}

pthread_t mpuThread;

PluginLoad(MPU)
{

    ft = inTable;
    currentTask = TASK_RUN;
    //TODO: do we need to start thread here?
    pthread_create(&mpuThread, NULL, gstate_update_func, &gData);

    //mpuThread = std::thread(gstate_update_func);

    DefineSimpleUnit(MPU);

    DefinePlugInCmd("mpuCmd", mpuCmdFunc, 0);
}

C_LINKAGE SC_API_EXPORT void unload(InterfaceTable *inTable)
{
    currentTask = TASK_STOP;
    pthread_join(mpuThread, NULL);
}
