SynthDef.new("NIME" {

    var b = Buffer.alloc(s, 44100 * 60,* 3, 1), // a 3-minute, 1 channel buffer
        offset = AHRS.kr(0, 0.2, AHRS.kr(1, 0.2, 72)),
        trigger = Impulse.ar(60, 0, 2, -1)

    RecordBuf.ar(SoundIn.ar(0), b.buffnum, offset, 1, 1, run, 1, trigger)

    Out.ar(GrainBuf.ar(1, 0, 1, b, offset / 145))
})
