SynthDef.new("NIME" {

    var b = Buffer.alloc(s, 44100 * 60,* 3, 1), // a 3-minute, 1 channel buffer
        offset = AHRS.kr(0, 0.2, AHRS.kr(1, 0.2, 72)) / 145,
        trigger = Impulse.ar(60, 0, 2, -1)

    // record into the buffer somewhere depending on AHRS data
    RecordBuf.ar(SoundIn.ar(0), b.buffnum, offset * 44100 * 60 * 3, 1, 1, GPIO.kr(26, -2 + 1, 1, trigger)

    // granulate out of the buffer at the same position
    Out.ar(GrainBuf.ar(1, 0, 1, b, offset))
})
