s.waitForBoot({
    {PitchShift.ar(SoundIn.ar(0), 0.01, AHRS.rollKr(1/180, AHRS.pitchKr(1/180, 2))) }.play
}, 300, {"oops".postln})
