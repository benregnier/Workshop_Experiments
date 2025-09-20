#include "ComputerCard.h"
#include "arm_math.h"
#include <cmath>
#include <cstdint>

class FreqShifterQ31 : public ComputerCard {
public:
    static constexpr int FS = 48000;
    static constexpr int HTAPS = 63;          // Hilbert taps (must be odd)
    static constexpr int BLEN = 1;            // Process one sample per call
    static constexpr int DELAYRB = 128;       // Delay buffer length (>= (HTAPS-1)/2)
    static constexpr int LUTN = 1024;         // Sine lookup table size (power of two)
    static constexpr bool UPPER_SB = true;    // true = upper sideband
    static constexpr uint32_t PHASE_BITS = 32;
    static constexpr float MAX_SHIFT_HZ = 4000.0f;
    static constexpr uint32_t STATUS_LED_INDEX = 0;
    static constexpr int AUDIO_Q31_SHIFT = 20; // Convert between 12-bit audio and Q31

    void Init() {
        buildHilbertQ31();
        arm_fir_init_q31(&firQ, HTAPS, h, firStateQ, BLEN);
        buildSinLUT_Q31();
        LedOn(STATUS_LED_INDEX, true);
    }

    void ProcessSample() override {
        q31_t x = audioToQ31(AudioIn1());
        q31_t I = pushDelay(x);

        q31_t xin = x;
        q31_t Q = 0;
        arm_fir_q31(&firQ, &xin, &Q, BLEN);

        float shiftHz = knobToShiftHz();
        uint32_t phaseInc = hzToPhaseIncrement(shiftHz);
        phase += phaseInc;

        q31_t sn = 0;
        q31_t cs = 0;
        sincos_q31(phase, sn, cs);

        q31_t yI = mult_q31(I, cs);
        q31_t yQ = mult_q31(Q, sn);
        q31_t y = UPPER_SB ? sat_q31(static_cast<int64_t>(yI) - static_cast<int64_t>(yQ))
                           : sat_q31(static_cast<int64_t>(yI) + static_cast<int64_t>(yQ));

        AudioOut1(q31ToAudio(y));
    }

private:
    q31_t h[HTAPS] = {};
    q31_t firStateQ[HTAPS + BLEN - 1] = {};
    arm_fir_instance_q31 firQ{};

    q31_t delayBuf[DELAYRB] = {};
    int dWrite = 0;
    int dRead = DELAYRB - (HTAPS - 1) / 2;

    uint32_t phase = 0;
    q31_t sinLUT[LUTN] = {};

    inline float knobToShiftHz() const {
        constexpr float invMax = 1.0f / 4095.0f;
        float k = static_cast<float>(KnobVal(Knob::X)) * invMax; // 0..1
        return (k * 2.0f - 1.0f) * MAX_SHIFT_HZ;
    }

    static uint32_t hzToPhaseIncrement(float shiftHz) {
        double scale = static_cast<double>(static_cast<uint64_t>(1) << PHASE_BITS) /
                       static_cast<double>(FS);
        double inc = static_cast<double>(shiftHz) * scale;
        return static_cast<uint32_t>(std::llround(inc));
    }

    static q31_t audioToQ31(int16_t sample) {
        return static_cast<q31_t>(sample) << AUDIO_Q31_SHIFT;
    }

    static int16_t q31ToAudio(q31_t value) {
        int32_t sample = static_cast<int32_t>(value >> AUDIO_Q31_SHIFT);
        if (sample > 2047) sample = 2047;
        if (sample < -2048) sample = -2048;
        return static_cast<int16_t>(sample);
    }

    static q31_t sat_q31(int64_t a) {
        if (a > 0x7FFFFFFFLL) {
            return 0x7FFFFFFF;
        }
        if (a < -0x80000000LL) {
            return static_cast<q31_t>(0x80000000);
        }
        return static_cast<q31_t>(a);
    }

    static q31_t mult_q31(q31_t a, q31_t b) {
        int64_t acc = static_cast<int64_t>(a) * static_cast<int64_t>(b); // Q62
        acc >>= 31;                                                     // Back to Q31
        return sat_q31(acc);
    }

    q31_t pushDelay(q31_t x) {
        delayBuf[dWrite] = x;
        if (++dWrite >= DELAYRB) {
            dWrite = 0;
        }
        q31_t y = delayBuf[dRead];
        if (++dRead >= DELAYRB) {
            dRead = 0;
        }
        return y;
    }

    void buildHilbertQ31() {
        constexpr double PI = 3.141592653589793238462643383279502884;
        const int mid = (HTAPS - 1) / 2;
        double coeffs[HTAPS];

        for (int n = 0; n < HTAPS; ++n) {
            int k = n - mid;
            double ideal = 0.0;
            if (k != 0 && (k & 1)) {
                ideal = 2.0 / (PI * static_cast<double>(k));
            }
            double w = 0.54 - 0.46 * std::cos(2.0 * PI * static_cast<double>(n) /
                                              static_cast<double>(HTAPS - 1));
            coeffs[n] = ideal * w;
        }

        for (int n = 0; n < mid; ++n) {
            double val = -coeffs[HTAPS - 1 - n];
            coeffs[n] = val;
            coeffs[HTAPS - 1 - n] = -val;
        }
        coeffs[mid] = 0.0;

        for (int n = 0; n < HTAPS; ++n) {
            double scaled = coeffs[n] * 2147483647.0;
            if (scaled > 2147483647.0) {
                scaled = 2147483647.0;
            } else if (scaled < -2147483648.0) {
                scaled = -2147483648.0;
            }
            h[n] = static_cast<q31_t>(std::llround(scaled));
        }
    }

    void buildSinLUT_Q31() {
        constexpr double PI = 3.141592653589793238462643383279502884;
        for (int i = 0; i < LUTN; ++i) {
            double angle = 2.0 * PI * static_cast<double>(i) / static_cast<double>(LUTN);
            double s = std::sin(angle);
            sinLUT[i] = static_cast<q31_t>(std::llround(s * 2147483647.0));
        }
    }

    void sincos_q31(uint32_t ph, q31_t& s, q31_t& c) const {
        static constexpr int LUT_BITS = 10; // log2(LUTN)
        static constexpr uint32_t LUT_MASK = LUTN - 1;
        uint32_t idx = ph >> (PHASE_BITS - LUT_BITS);
        uint32_t idx90 = (idx + LUTN / 4) & LUT_MASK;
        s = sinLUT[idx];
        c = sinLUT[idx90];
    }
};

FreqShifterQ31 gFreqShift;

void setup() {
    gFreqShift.EnableNormalisationProbe();
    gFreqShift.Init();
}

void loop() {
    gFreqShift.Run();
}
