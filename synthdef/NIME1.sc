s.boot;

s.waitForBoot({
    Buffer.readChannel(s, "/home/ccrma/git/nime/data/audio.wav", channels:[0],
      action:  { |b|
          "yay".postln;  
          b.dump;

          {
            "woop".postln;

            // granulate out of the buffer at a position based on roll, with number of grains
            // based on pitch
            GrainBuf.ar(1, Impulse.kr(GPIO.kr(26, 10, 10)), AHRS.pitchKr(0.5/360, 0.60), sndbuf:b, pos:AHRS.rollKr(1/360, 0.5));
          }.play;
    });
}, 300, {"oops".postln});
