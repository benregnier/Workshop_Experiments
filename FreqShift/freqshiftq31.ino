#ifdef USE_Q31_VARIANT
// High-quality fixed-point frequency shifter with dual-sideband outputs.

#include "ComputerCard.h"
#include "arm_math.h"
#include "hardware/clocks.h"
#include <cmath>
#include <cstdint>

class FreqShifterQ31 : public ComputerCard {
public:
    static constexpr int FS = 48000;
    static constexpr int HTAPS = 95;
    static constexpr int BLEN = 1;
    static constexpr int DELAYRB = 192;

    static constexpr int SIN_LUT_N = 4096;
    static constexpr int SIN_LUT_BITS = 12;
    static constexpr int FREQ_LUT_N = 2048;

    static constexpr uint32_t PHASE_BITS = 32;
    static constexpr int AUDIO_Q31_SHIFT = 20;
    static constexpr int16_t AUDIO_MAX = 2047;
    static constexpr int16_t AUDIO_MIN = -2048;

    // Frequency ranges.
    static constexpr int32_t WIDE_MIN_SHIFT_HZ = 2;
    static constexpr int32_t WIDE_MAX_SHIFT_HZ = 11000;
    static constexpr int32_t NARROW_MAX_SHIFT_HZ = 2200;

    // Optional conservative overclock for extra DSP headroom.
    static constexpr uint32_t TARGET_SYS_CLOCK_KHZ = 250000;

    enum FeedbackMode : uint8_t { FB_DOWN = 0, FB_UP = 1, FB_COMBINED = 2 };

    void Init() {
        set_sys_clock_khz(TARGET_SYS_CLOCK_KHZ, true);

        buildHilbertQ31();
        arm_fir_init_q31(&firQ, HTAPS, h, firStateQ, BLEN);
        buildSinLUT_Q31();
        buildPhaseIncrementLUT();

        phase = 0;
        phaseIncSigned = 0;
        feedbackMode = FB_COMBINED;
        pulse1Last = false;

        for (int i = 0; i < 6; ++i) {
            LedOff(i);
        }
    }

    void ProcessSample() override {
        updateControl();

        // Y: mix AudioIn2 into AudioIn1
        q31_t a1 = audioToQ31(AudioIn1());
        q31_t a2 = audioToQ31(AudioIn2());
        q31_t inMix = sat_q31(static_cast<int64_t>(mult_q31(a1, in1GainQ31)) +
                              static_cast<int64_t>(mult_q31(a2, in2GainQ31)));

        // Internal feedback path into shifter input
        q31_t feedbackSignal = calcFeedbackSignal();
        q31_t fbApplied = mult_q31(feedbackSignal, feedbackGainQ31);
        q31_t x = sat_q31(static_cast<int64_t>(inMix) + static_cast<int64_t>(fbApplied));

        q31_t I = pushDelay(x);

        q31_t xin = x;
        q31_t Q = 0;
        arm_fir_q31(&firQ, &xin, &Q, BLEN);

        phase += phaseIncSigned;

        q31_t sn = 0;
        q31_t cs = 0;
        sincos_q31(phase, sn, cs);

        // Dual sidebands.
        q31_t yI = mult_q31(I, cs);
        q31_t yQ = mult_q31(Q, sn);
        lowSideband = sat_q31(static_cast<int64_t>(yI) + static_cast<int64_t>(yQ));
        highSideband = sat_q31(static_cast<int64_t>(yI) - static_cast<int64_t>(yQ));

        AudioOut1(q31ToAudio(lowSideband));
        AudioOut2(q31ToAudio(highSideband));

        updateLeds();
    }

private:
    q31_t h[HTAPS] = {};
    q31_t firStateQ[HTAPS + BLEN - 1] = {};
    arm_fir_instance_q31 firQ{};

    q31_t delayBuf[DELAYRB] = {};
    int dWrite = 0;
    int dRead = DELAYRB - (HTAPS - 1) / 2;

    uint32_t phase = 0;
    uint32_t phaseIncSigned = 0;

    q31_t sinLUT[SIN_LUT_N] = {};
    uint32_t widePhaseIncLUT[FREQ_LUT_N] = {};

    q31_t lowSideband = 0;
    q31_t highSideband = 0;

    q31_t in1GainQ31 = 0x7FFFFFFF;
    q31_t in2GainQ31 = 0;
    q31_t feedbackGainQ31 = 0;
    q31_t feedbackBlendQ31 = 0x40000000; // CV2: up/down blend for combined mode

    int32_t currentShiftHz = 0;
    FeedbackMode feedbackMode = FB_COMBINED;
    bool pulse1Last = false;

    uint16_t controlDivider = 0;

    static int16_t clamp12(int32_t v) {
        if (v < 0) return 0;
        if (v > 4095) return 4095;
        return static_cast<int16_t>(v);
    }

    static q31_t q12ToQ31(int32_t vQ12) {
        int32_t centered = (vQ12 << 19); // 12-bit full scale to Q31 space
        return sat_q31(centered);
    }

    static q31_t audioToQ31(int16_t sample) {
        return static_cast<q31_t>(sample) << AUDIO_Q31_SHIFT;
    }

    static int16_t q31ToAudio(q31_t value) {
        int32_t sample = static_cast<int32_t>(value >> AUDIO_Q31_SHIFT);
        if (sample > AUDIO_MAX) sample = AUDIO_MAX;
        if (sample < AUDIO_MIN) sample = AUDIO_MIN;
        return static_cast<int16_t>(sample);
    }

    static q31_t sat_q31(int64_t a) {
        if (a > 0x7FFFFFFFLL) return 0x7FFFFFFF;
        if (a < -0x80000000LL) return static_cast<q31_t>(0x80000000);
        return static_cast<q31_t>(a);
    }

    static q31_t mult_q31(q31_t a, q31_t b) {
        int64_t acc = static_cast<int64_t>(a) * static_cast<int64_t>(b);
        acc >>= 31;
        return sat_q31(acc);
    }

    void updateControl() {
        if (++controlDivider < 16) {
            return;
        }
        controlDivider = 0;

        // Main + switch + CV1 -> shift frequency.
        int32_t mainQ12 = clamp12(KnobVal(Knob::Main));
        int32_t shiftCVQ12 = clamp12(CVIn1() + 2048);
        int32_t shiftPos = clamp12(mainQ12 + (shiftCVQ12 - 2048));

        const Switch sw = SwitchVal();

        if (sw == Switch::Up) {
            // Wide, log-scaled bipolar mapping.
            int32_t magIdx = (shiftPos >= 2048) ? (shiftPos - 2048) : (2047 - shiftPos);
            if (magIdx < 0) magIdx = 0;
            if (magIdx >= FREQ_LUT_N) magIdx = FREQ_LUT_N - 1;
            int32_t coarseHz = phaseIncToHz(widePhaseIncLUT[magIdx]);
            currentShiftHz = (shiftPos >= 2048) ? coarseHz : -coarseHz;
        } else {
            // Middle (and Down) use narrower linear range around zero.
            int32_t centered = shiftPos - 2048;
            currentShiftHz = (centered * NARROW_MAX_SHIFT_HZ) / 2048;
        }

        phaseIncSigned = hzToPhaseIncrement(currentShiftHz);

        // X = feedback amount.
        int32_t xQ12 = clamp12(KnobVal(Knob::X));
        feedbackGainQ31 = q12ToQ31(xQ12);

        // Y = AudioIn2 mix into AudioIn1.
        int32_t yQ12 = clamp12(KnobVal(Knob::Y));
        in2GainQ31 = q12ToQ31(yQ12);
        in1GainQ31 = static_cast<q31_t>(0x7FFFFFFF - in2GainQ31);

        // CV2 controls internal combined up/down feedback blend.
        int32_t cv2Q12 = clamp12(CVIn2() + 2048);
        feedbackBlendQ31 = q12ToQ31(cv2Q12);

        // Down switch: PulseIn1 toggles feedback mode (down/up/combined).
        if (sw == Switch::Down) {
            bool p1 = PulseIn1();
            if (p1 && !pulse1Last) {
                feedbackMode = static_cast<FeedbackMode>((static_cast<uint8_t>(feedbackMode) + 1) % 3);
            }
            pulse1Last = p1;
        } else {
            pulse1Last = PulseIn1();
            // Middle/Up: direct mapping for deterministic behavior.
            feedbackMode = (sw == Switch::Middle) ? FB_COMBINED : FB_UP;
        }
    }

    q31_t calcFeedbackSignal() const {
        switch (feedbackMode) {
            case FB_DOWN:
                return lowSideband;
            case FB_UP:
                return highSideband;
            case FB_COMBINED:
            default: {
                // CV2 crossfades between down and up in combined mode.
                q31_t downPart = mult_q31(lowSideband, static_cast<q31_t>(0x7FFFFFFF - feedbackBlendQ31));
                q31_t upPart = mult_q31(highSideband, feedbackBlendQ31);
                return sat_q31(static_cast<int64_t>(downPart) + static_cast<int64_t>(upPart));
            }
        }
    }

    int32_t phaseIncToHz(uint32_t inc) const {
        uint64_t numer = static_cast<uint64_t>(inc) * FS;
        return static_cast<int32_t>(numer >> PHASE_BITS);
    }

    static uint32_t hzToPhaseIncrement(int32_t shiftHz) {
        int64_t numer = static_cast<int64_t>(shiftHz) << PHASE_BITS;
        int64_t inc = numer / FS;
        return static_cast<uint32_t>(inc);
    }

    q31_t pushDelay(q31_t x) {
        delayBuf[dWrite] = x;
        if (++dWrite >= DELAYRB) dWrite = 0;
        q31_t y = delayBuf[dRead];
        if (++dRead >= DELAYRB) dRead = 0;
        return y;
    }

    void buildHilbertQ31() {
        constexpr double kPi = 3.141592653589793238462643383279502884;
        const int mid = (HTAPS - 1) / 2;

        for (int n = 0; n < HTAPS; ++n) {
            int k = n - mid;
            double ideal = 0.0;
            if (k != 0 && (k & 1)) {
                ideal = 2.0 / (kPi * static_cast<double>(k));
            }
            double w = 0.54 - 0.46 * std::cos(2.0 * kPi * static_cast<double>(n) /
                                              static_cast<double>(HTAPS - 1));
            double coeff = ideal * w;
            double scaled = coeff * 2147483647.0;
            if (scaled > 2147483647.0) scaled = 2147483647.0;
            if (scaled < -2147483648.0) scaled = -2147483648.0;
            h[n] = static_cast<q31_t>(std::llround(scaled));
        }

        for (int n = 0; n < mid; ++n) {
            q31_t val = h[n] - h[HTAPS - 1 - n];
            h[n] = static_cast<q31_t>(val / 2);
            h[HTAPS - 1 - n] = static_cast<q31_t>(-h[n]);
        }
        h[mid] = 0;
    }

    void buildSinLUT_Q31() {
        constexpr double kPi = 3.141592653589793238462643383279502884;
        for (int i = 0; i < SIN_LUT_N; ++i) {
            double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(SIN_LUT_N);
            double s = std::sin(angle);
            sinLUT[i] = static_cast<q31_t>(std::llround(s * 2147483647.0));
        }
    }

    void buildPhaseIncrementLUT() {
        const double minHz = static_cast<double>(WIDE_MIN_SHIFT_HZ);
        const double maxHz = static_cast<double>(WIDE_MAX_SHIFT_HZ);
        const double ratio = std::pow(maxHz / minHz, 1.0 / static_cast<double>(FREQ_LUT_N - 1));

        double hz = minHz;
        for (int i = 0; i < FREQ_LUT_N; ++i) {
            widePhaseIncLUT[i] = hzToPhaseIncrement(static_cast<int32_t>(std::llround(hz)));
            hz *= ratio;
        }
    }

    void sincos_q31(uint32_t ph, q31_t& s, q31_t& c) const {
        static constexpr uint32_t LUT_MASK = SIN_LUT_N - 1;
        uint32_t idx = ph >> (PHASE_BITS - SIN_LUT_BITS);
        uint32_t idx90 = (idx + SIN_LUT_N / 4) & LUT_MASK;
        s = sinLUT[idx & LUT_MASK];
        c = sinLUT[idx90];
    }

    void updateLeds() {
        LedOn(0, currentShiftHz >= 0);
        LedOn(1, currentShiftHz < 0);
        LedOn(2, feedbackMode == FB_DOWN);
        LedOn(3, feedbackMode == FB_UP);
        LedOn(4, feedbackMode == FB_COMBINED);
        LedOn(5, KnobVal(Knob::Y) > 2048);
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

#endif // USE_Q31_VARIANT
