s.waitForBoot({
    Buffer.readChannel(s, "/home/ccrma/git/nime/data/audio.wav", channels:[0],
      action:  { |b|
          "yay".postln;  
          b.dump;

          {
            var offset = AHRS.rollKr(0, 1/360, 0.5);
            "woop".postln;

            // granulate out of the buffer at a position based on roll, with number of grains
            // based on pitch
            GrainBuf.ar(1, Impulse.kr(13), AHRS.pitchKr(0.5/360, 0.3), sndbuf:b, pos:offset);
          }.play;
    });
}, 300, {"oops".postln});
