s.waitForBoot({
  {
      var b = Buffer.readChanne(s, "audio.wav", 0, -1, 0),
          offset = AHRS.rollKr(0, 1/360, 0.5);

      // record into the buffer somewhere depending on AHRS data
      RecordBuf.ar(SoundIn.ar(0), b, offset * b.numFrames, loop:1, run:1, trigger:Impulse.kr(30));

      // granulate out of the buffer at the same position
       GrainBuf.ar(1, Impulse.kr(10), AHRS.pitchKr(0 0.5/360, 0.3) b, offset);
  }.play;
}, 300, {"oops".postln})
