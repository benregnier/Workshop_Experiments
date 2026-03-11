// High-quality fixed-point frequency shifter with dual-sideband outputs.
//
// Converted from freqshiftq31.ino to Pico SDK (main.cpp) format so it can
// be built with the existing CMake toolchain.
//
// Key fixes versus the .ino sketch:
//   1. set_sys_clock_khz() is called FIRST in main(), before ComputerCard
//      constructs and calls spi_init().  Previously it was called inside
//      Init() which ran after spi_init() had already latched the wrong clock
//      divider, crashing DAC output.
//   2. arm_math.h / arm_fir_q31 dependency removed.  Replaced with a small
//      hand-rolled antisymmetric Hilbert FIR that the Pico SDK can build
//      without CMSIS-DSP.
//   3. HTAPS raised from 3 to 31.  A 31-tap Hilbert FIR needs only ~8
//      multiply-accumulates per sample (non-zero coefficients at odd k only,
//      halved again by antisymmetry), giving much better sideband separation
//      while staying well inside the ~5200-cycle budget at 250 MHz / 48 kHz.

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "ComputerCard.h"
#include <cmath>
#include <cstdint>

class FreqShifterQ31 : public ComputerCard {
public:
    static constexpr int FS = 48000;

    // 31-tap Hilbert FIR.  Non-zero coefficients only at odd k = (n – mid),
    // so effective multiply count ≈ 8 per sample with the antisymmetric trick.
    static constexpr int HTAPS = 31;

    // Delay buffer aligns the I path to match the FIR group delay of (HTAPS-1)/2.
    static constexpr int DELAYRB = 64;

    static constexpr int     SIN_LUT_N    = 4096;
    static constexpr int     SIN_LUT_BITS = 12;
    static constexpr int     FREQ_LUT_N   = 2048;
    static constexpr uint32_t PHASE_BITS  = 32;

    static constexpr int     AUDIO_Q31_SHIFT = 20;
    static constexpr int16_t AUDIO_MAX       = 2047;
    static constexpr int16_t AUDIO_MIN       = -2048;

    static constexpr int32_t WIDE_MIN_SHIFT_HZ  = 2;
    static constexpr int32_t WIDE_MAX_SHIFT_HZ  = 11000;
    static constexpr int32_t NARROW_MAX_SHIFT_HZ = 220;

    enum FeedbackMode : uint8_t { FB_DOWN = 0, FB_UP = 1, FB_COMBINED = 2 };

    void Init() {
        buildHilbertQ31();
        buildSinLUT_Q31();
        buildPhaseIncrementLUT();

        phase         = 0;
        phaseIncSigned = 0;
        feedbackMode  = FB_COMBINED;
        pulse1Last    = false;

        for (int i = 0; i < 6; ++i) LedOff(i);
    }

    void ProcessSample() override {
        updateControl();

        // Mix AudioIn1 and AudioIn2 according to Y knob.
        q31_t a1    = audioToQ31(AudioIn1());
        q31_t a2    = audioToQ31(AudioIn2());
        q31_t inMix = sat_q31(static_cast<int64_t>(mult_q31(a1, in1GainQ31)) +
                              static_cast<int64_t>(mult_q31(a2, in2GainQ31)));

        // Internal feedback path.
        q31_t feedbackSignal = calcFeedbackSignal();
        q31_t fbApplied      = mult_q31(feedbackSignal, feedbackGainQ31);
        q31_t x = sat_q31(static_cast<int64_t>(inMix) + static_cast<int64_t>(fbApplied));
        inputLevelQ31 = smoothLevel(inputLevelQ31, abs_q31(x));

        // I path: direct signal delayed to match FIR group delay.
        q31_t I = pushDelay(x);

        // Q path: 90-degree phase-shifted via Hilbert FIR.
        q31_t Q = applyHilbertFIR(x);

        // Advance oscillator phase and look up sin/cos.
        phase += phaseIncSigned;
        q31_t sn, cs;
        sincos_q31(phase, sn, cs);

        // Single-sideband mixing: upper = I·cos − Q·sin, lower = I·cos + Q·sin.
        q31_t yI       = mult_q31(I, cs);
        q31_t yQ       = mult_q31(Q, sn);
        lowSideband    = sat_q31(static_cast<int64_t>(yI) + static_cast<int64_t>(yQ));
        highSideband   = sat_q31(static_cast<int64_t>(yI) - static_cast<int64_t>(yQ));

        int32_t outAbs  = abs_q31(lowSideband);
        int32_t highAbs = abs_q31(highSideband);
        if (highAbs > outAbs) outAbs = highAbs;
        outputLevelQ31 = smoothLevel(outputLevelQ31, outAbs);

        AudioOut1(q31ToAudio(lowSideband));
        AudioOut2(q31ToAudio(highSideband));

        updateLeds();
    }

private:
    // -----------------------------------------------------------------------
    // Hilbert FIR state
    // -----------------------------------------------------------------------
    q31_t h[HTAPS]        = {};   // full coefficient array built at Init()
    q31_t firState[HTAPS] = {};   // circular delay line for FIR
    int   firStatePtr     = 0;    // next-write position in firState[]

    // -----------------------------------------------------------------------
    // Alignment delay (compensates for FIR group delay on the I path)
    // -----------------------------------------------------------------------
    q31_t delayBuf[DELAYRB] = {};
    int   dWrite = 0;
    int   dRead  = DELAYRB - (HTAPS - 1) / 2;   // pre-delayed by group delay

    // -----------------------------------------------------------------------
    // Oscillator
    // -----------------------------------------------------------------------
    uint32_t phase         = 0;
    uint32_t phaseIncSigned = 0;   // treated as two's-complement for negative freq

    // -----------------------------------------------------------------------
    // Lookup tables
    // -----------------------------------------------------------------------
    q31_t    sinLUT[SIN_LUT_N]        = {};
    uint32_t widePhaseIncLUT[FREQ_LUT_N] = {};

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    q31_t   lowSideband    = 0;
    q31_t   highSideband   = 0;
    int32_t inputLevelQ31  = 0;
    int32_t outputLevelQ31 = 0;

    q31_t in1GainQ31      = 0x7FFFFFFF;
    q31_t in2GainQ31      = 0;
    q31_t feedbackGainQ31 = 0;
    q31_t feedbackBlendQ31 = 0x40000000;

    int32_t      currentShiftHz = 0;
    FeedbackMode feedbackMode   = FB_COMBINED;
    bool         pulse1Last     = false;
    uint16_t     controlDivider = 0;

    // -----------------------------------------------------------------------
    // Fixed-point helpers
    // -----------------------------------------------------------------------
    static int16_t clamp12(int32_t v) {
        if (v < 0)    return 0;
        if (v > 4095) return 4095;
        return static_cast<int16_t>(v);
    }

    static q31_t q12ToQ31(int32_t vQ12) {
        // Scale 12-bit unsigned (0–4095) to Q31 full scale.
        return sat_q31(static_cast<int64_t>(vQ12) << 19);
    }

    static q31_t audioToQ31(int16_t sample) {
        return static_cast<q31_t>(sample) << AUDIO_Q31_SHIFT;
    }

    static int16_t q31ToAudio(q31_t value) {
        int32_t s = static_cast<int32_t>(value >> AUDIO_Q31_SHIFT);
        if (s > AUDIO_MAX) s = AUDIO_MAX;
        if (s < AUDIO_MIN) s = AUDIO_MIN;
        return static_cast<int16_t>(s);
    }

    static q31_t sat_q31(int64_t a) {
        if (a >  0x7FFFFFFFLL)  return  0x7FFFFFFF;
        if (a < -0x80000000LL)  return static_cast<q31_t>(0x80000000U);
        return static_cast<q31_t>(a);
    }

    static q31_t mult_q31(q31_t a, q31_t b) {
        int64_t acc = static_cast<int64_t>(a) * static_cast<int64_t>(b);
        return sat_q31(acc >> 31);
    }

    static int32_t abs_q31(q31_t v) {
        if (v >= 0) return static_cast<int32_t>(v);
        if (v == static_cast<q31_t>(0x80000000U)) return 0x7FFFFFFF;
        return static_cast<int32_t>(-v);
    }

    static uint16_t q31ToLedQ12(int32_t v) {
        if (v <= 0) return 0;
        uint32_t s = static_cast<uint32_t>(v) >> 19;
        if (s > 4095U) s = 4095U;
        return static_cast<uint16_t>(s);
    }

    static int32_t smoothLevel(int32_t prev, int32_t cur) {
        if (cur > prev) return prev + ((cur  - prev) >> 2);
        return           prev - ((prev - cur)  >> 6);
    }

    // -----------------------------------------------------------------------
    // Antisymmetric Hilbert FIR
    //
    // A Hilbert FIR has h[n] = -h[N-1-n] and h[mid] = 0, and the non-zero
    // coefficients occur only at positions where (n – mid) is odd.  Both
    // properties are exploited here:
    //   • Antisymmetry halves the multiply count (15 → 15 pairs → 15 mults)
    //   • Only-odd-k coefficients halve it again (~8 actual multiplications)
    // -----------------------------------------------------------------------
    q31_t applyHilbertFIR(q31_t x) {
        // Write newest sample into circular buffer.
        firState[firStatePtr] = x;
        if (++firStatePtr >= HTAPS) firStatePtr = 0;

        int64_t acc = 0;
        constexpr int mid = (HTAPS - 1) / 2;

        // For HTAPS=31, mid=15 (odd), so non-zero h[n] at even n (k = n-mid odd).
        // Loop with stride 2 to skip the zero-valued even-k coefficients.
        // If HTAPS is changed such that mid becomes even, swap the starting
        // index from 0 to 1 below.
        static_assert(((HTAPS - 1) / 2) % 2 == 1,
            "HTAPS gives even mid; change loop start from 0 to 1");

        for (int n = 0; n < mid; n += 2) {
            int new_idx = firStatePtr - 1 - n;
            if (new_idx < 0) new_idx += HTAPS;
            int old_idx = firStatePtr + n;
            if (old_idx >= HTAPS) old_idx -= HTAPS;
            acc += static_cast<int64_t>(h[n]) *
                   (static_cast<int64_t>(firState[new_idx]) -
                    static_cast<int64_t>(firState[old_idx]));
        }
        return sat_q31(acc >> 31);
    }

    // -----------------------------------------------------------------------
    // Alignment delay for I path
    // -----------------------------------------------------------------------
    q31_t pushDelay(q31_t x) {
        delayBuf[dWrite] = x;
        if (++dWrite >= DELAYRB) dWrite = 0;
        q31_t y = delayBuf[dRead];
        if (++dRead >= DELAYRB) dRead = 0;
        return y;
    }

    // -----------------------------------------------------------------------
    // Control update (runs every 16 samples to reduce ISR load)
    // -----------------------------------------------------------------------
    void updateControl() {
        if (++controlDivider < 16) return;
        controlDivider = 0;

        int32_t mainQ12    = clamp12(KnobVal(Knob::Main));
        int32_t shiftCVQ12 = clamp12(CVIn1() + 2048);
        int32_t shiftPos   = clamp12(mainQ12 + (shiftCVQ12 - 2048));

        const Switch sw = SwitchVal();

        if (sw == Switch::Up) {
            // Wide, log-scaled bipolar range.
            int32_t magIdx = (shiftPos >= 2048) ? (shiftPos - 2048) : (2047 - shiftPos);
            if (magIdx < 0) magIdx = 0;
            if (magIdx >= FREQ_LUT_N) magIdx = FREQ_LUT_N - 1;
            int32_t coarseHz = phaseIncToHz(widePhaseIncLUT[magIdx]);
            currentShiftHz   = (shiftPos >= 2048) ? coarseHz : -coarseHz;
        } else {
            // Middle / Down: narrow linear range.
            int32_t centered = shiftPos - 2048;
            currentShiftHz   = (centered * NARROW_MAX_SHIFT_HZ) / 2048;
        }

        phaseIncSigned = hzToPhaseIncrement(currentShiftHz);

        int32_t xQ12    = clamp12(KnobVal(Knob::X));
        feedbackGainQ31 = q12ToQ31(xQ12);

        int32_t yQ12  = clamp12(KnobVal(Knob::Y));
        in2GainQ31    = q12ToQ31(yQ12);
        in1GainQ31    = static_cast<q31_t>(0x7FFFFFFF - in2GainQ31);

        int32_t cv2Q12   = clamp12(CVIn2() + 2048);
        feedbackBlendQ31 = q12ToQ31(cv2Q12);

        // Switch Down: PulseIn1 rising edge cycles feedback mode.
        if (sw == Switch::Down) {
            bool p1 = PulseIn1();
            if (p1 && !pulse1Last) {
                feedbackMode = static_cast<FeedbackMode>(
                    (static_cast<uint8_t>(feedbackMode) + 1) % 3);
            }
            pulse1Last = p1;
        } else {
            pulse1Last   = PulseIn1();
            feedbackMode = (sw == Switch::Middle) ? FB_COMBINED : FB_UP;
        }
    }

    q31_t calcFeedbackSignal() const {
        switch (feedbackMode) {
            case FB_DOWN: return lowSideband;
            case FB_UP:   return highSideband;
            case FB_COMBINED:
            default: {
                q31_t downPart = mult_q31(lowSideband,
                                          static_cast<q31_t>(0x7FFFFFFF - feedbackBlendQ31));
                q31_t upPart   = mult_q31(highSideband, feedbackBlendQ31);
                return sat_q31(static_cast<int64_t>(downPart) + static_cast<int64_t>(upPart));
            }
        }
    }

    // -----------------------------------------------------------------------
    // Phase / frequency helpers
    // -----------------------------------------------------------------------
    int32_t phaseIncToHz(uint32_t inc) const {
        uint64_t n = static_cast<uint64_t>(inc) * FS;
        return static_cast<int32_t>(n >> PHASE_BITS);
    }

    static uint32_t hzToPhaseIncrement(int32_t shiftHz) {
        // Negative shiftHz yields a large uint32_t via two's complement,
        // which correctly decrements the phase accumulator on each sample.
        int64_t n = static_cast<int64_t>(shiftHz) * static_cast<int64_t>(1ULL << PHASE_BITS);
        return static_cast<uint32_t>(n / FS);
    }

    // -----------------------------------------------------------------------
    // LUT lookups
    // -----------------------------------------------------------------------
    void sincos_q31(uint32_t ph, q31_t& s, q31_t& c) const {
        constexpr uint32_t LUT_MASK = SIN_LUT_N - 1;
        uint32_t idx   = ph >> (PHASE_BITS - SIN_LUT_BITS);
        uint32_t idx90 = (idx + SIN_LUT_N / 4) & LUT_MASK;
        s = sinLUT[idx   & LUT_MASK];
        c = sinLUT[idx90];
    }

    // -----------------------------------------------------------------------
    // LED feedback
    // -----------------------------------------------------------------------
    void updateLeds() {
        LedBrightness(0, q31ToLedQ12(inputLevelQ31));
        LedBrightness(1, q31ToLedQ12(outputLevelQ31));
        LedOff(2);

        int32_t magHz  = currentShiftHz >= 0 ? currentShiftHz : -currentShiftHz;
        int32_t maxHz  = (SwitchVal() == Switch::Up) ? WIDE_MAX_SHIFT_HZ : NARROW_MAX_SHIFT_HZ;
        uint16_t shiftLevel = 0;
        if (maxHz > 0) {
            int32_t scaled = (magHz * 4095) / maxHz;
            if (scaled < 0)    scaled = 0;
            if (scaled > 4095) scaled = 4095;
            shiftLevel = static_cast<uint16_t>(scaled);
        }

        LedBrightness(3, currentShiftHz >= 0 ? shiftLevel : 0);
        LedBrightness(4, currentShiftHz  < 0 ? shiftLevel : 0);
        LedOff(5);
    }

    // -----------------------------------------------------------------------
    // Initialisation helpers (run once, floating-point OK here)
    // -----------------------------------------------------------------------
    void buildHilbertQ31() {
        constexpr double kPi = 3.141592653589793238462643383279502884;
        const int mid = (HTAPS - 1) / 2;

        for (int n = 0; n < HTAPS; ++n) {
            int    k     = n - mid;
            double ideal = (k != 0 && (k & 1)) ? (2.0 / (kPi * k)) : 0.0;
            double w     = 0.54 - 0.46 * std::cos(2.0 * kPi * n / (HTAPS - 1));
            double scaled = ideal * w * 2147483647.0;
            if (scaled >  2147483647.0) scaled =  2147483647.0;
            if (scaled < -2147483648.0) scaled = -2147483648.0;
            h[n] = static_cast<q31_t>(std::llround(scaled));
        }

        // Enforce exact antisymmetry after Q31 quantisation.
        for (int n = 0; n < mid; ++n) {
            int64_t val    = static_cast<int64_t>(h[n]) - static_cast<int64_t>(h[HTAPS - 1 - n]);
            h[n]           = static_cast<q31_t>(val / 2);
            h[HTAPS-1-n]   = static_cast<q31_t>(-h[n]);
        }
        h[mid] = 0;
    }

    void buildSinLUT_Q31() {
        constexpr double kPi = 3.141592653589793238462643383279502884;
        for (int i = 0; i < SIN_LUT_N; ++i) {
            double angle = 2.0 * kPi * i / SIN_LUT_N;
            sinLUT[i] = static_cast<q31_t>(std::llround(std::sin(angle) * 2147483647.0));
        }
    }

    void buildPhaseIncrementLUT() {
        const double minHz = static_cast<double>(WIDE_MIN_SHIFT_HZ);
        const double maxHz = static_cast<double>(WIDE_MAX_SHIFT_HZ);
        const double ratio = std::pow(maxHz / minHz, 1.0 / (FREQ_LUT_N - 1));
        double hz = minHz;
        for (int i = 0; i < FREQ_LUT_N; ++i) {
            widePhaseIncLUT[i] = hzToPhaseIncrement(static_cast<int32_t>(std::llround(hz)));
            hz *= ratio;
        }
    }
};

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int main() {
    // Overclock MUST happen before ComputerCard is constructed.
    // ComputerCard's constructor calls spi_init() which latches a clock
    // divider based on the current clk_sys.  The MCP4822 DAC max SCK is
    // 20 MHz; at 125 MHz, spi_init(SPI_PORT, 15625000) gives ~15.6 MHz
    // (fine).  At 250 MHz with the same divider it would hit ~31 MHz
    // (too fast, causing corrupt DAC output).  Overclocking first lets
    // spi_init() calculate the right divider for 250 MHz.
    //
    // Note: clk_adc is derived from the USB PLL (48 MHz), not clk_sys,
    // so ADC sample timing is unaffected by the overclock.
    set_sys_clock_khz(250000, true);

    FreqShifterQ31 freqShift;
    freqShift.EnableNormalisationProbe();
    freqShift.Init();
    freqShift.Run();
}
