"Loading synthdefs".postln;

SynthDef(\mpuTest, {
    //MPU.accelKr.poll(1, "accel");
    //MPU.magKr.poll(1, "mag");
    //MPU.magKr.poll(1, "mag");
    MPU.orientationKr.poll(1, "ori");
    MPU.rollKr.poll(1, "roll");
    MPU.pitchKr.poll(1, "pitch");
    MPU.yawKr.poll(1, "yaw");
}).store; //Store so we can run server


0.exit;
