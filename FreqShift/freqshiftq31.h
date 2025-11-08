#include "ComputerCard.h"
#include "arm_math.h"
#include <stdint.h>

class FreqShifterQ31 : public ComputerCard {
public:
    // ===== Config =====
    static constexpr int FS      = 48000;
    static constexpr int HTAPS   = 63;          // must be odd
    static constexpr int BLEN    = 1;           // process per-sample for simplicity
    static constexpr int DELAYRB = 128;         // >= (HTAPS-1)/2
    static constexpr int LUTN    = 1024;
    static constexpr bool UPPER_SB = true;

    // FIR state
    q31_t h[HTAPS];
    q31_t firStateQ[HTAPS + BLEN - 1];
    arm_fir_instance_q31 firQ;

    // Delay for I
    q31_t delayBuf[DELAYRB] = {0};
    int dW = 0, dR = DELAYRB - (HTAPS - 1) / 2;

    // DDS
    uint32_t phase = 0;
    static constexpr uint32_t PHASE_BITS = 32;
    q31_t sinLUT[LUTN];

    void setup() override {
        buildHilbertQ31();
        arm_fir_init_q31(&firQ, HTAPS, h, firStateQ, BLEN);
        buildSinLUT_Q31();
        setLED(0, true);
    }

    // Map knob to ±4000 Hz shift
    inline float knobToShiftHz() {
        float k = knobX(); // 0..1
        return (k * 2.0f - 1.0f) * 4000.0f;
    }

    void loop() override {
        // Convert float in [-1,1] to Q31 if your audioIn1() is float.
        // If your input is already integer, adjust accordingly.
        float xf = audioIn1();
        q31_t x = float_to_q31(xf);

        // I path: pure delay
        q31_t I = pushDelay(x);

        // Q path: Hilbert FIR
        q31_t xin = x, qout;
        arm_fir_q31(&firQ, &xin, &qout, BLEN); // BLEN=1, processes one sample

        // DDS phase
        float shiftHz = knobToShiftHz();
        uint32_t phaseInc = (uint32_t)llround((double)shiftHz * (double)((uint64_t)1 << PHASE_BITS) / (double)FS);
        phase += phaseInc;

        q31_t sn, cs;
        sincos_q31(phase, sn, cs);

        // y = I*cos -/+ Q*sin  (Q31 multiply -> Q63 -> back to Q31)
        q31_t y1, y2, y;
        y1 = mult_q31(I, cs);
        y2 = mult_q31(qout, sn);
        y  = UPPER_SB ? sat_q31(y1 - y2) : sat_q31(y1 + y2);

        // Convert back to float for audioOut1() if needed
        audioOut1(q31_to_float(y));
    }

private:
    // ---------- Utilities ----------
    static inline q31_t float_to_q31(float x) {
        // saturate
        if (x >= 0.999999f) x = 0.999999f;
        if (x <= -1.0f)     x = -1.0f;
        return (q31_t)(x * 2147483647.0f); // 2^31 - 1
    }
    static inline float q31_to_float(q31_t x) {
        return (float)x / 2147483647.0f;
    }
    static inline q31_t sat_q31(int64_t a) {
        if (a >  0x7FFFFFFFll) return 0x7FFFFFFF;
        if (a < -0x80000000ll) return (q31_t)0x80000000;
        return (q31_t)a;
    }
    static inline q31_t mult_q31(q31_t a, q31_t b) {
        // (q31 * q31) >> 31 with saturation
        int64_t acc = (int64_t)a * (int64_t)b;   // Q62
        acc = acc >> 31;                         // back to Q31
        return sat_q31(acc);
    }

    // ---------- I delay ----------
    q31_t pushDelay(q31_t x) {
        delayBuf[dW] = x;
        if (++dW >= DELAYRB) dW = 0;
        q31_t y = delayBuf[dR];
        if (++dR >= DELAYRB) dR = 0;
        return y;
    }

    // ---------- Hilbert taps (Q31) ----------
    void buildHilbertQ31() {
        // Same as float: ideal Hilbert * Hamming, then quantize to Q31.
        const int M = HTAPS;
        const int mid = (M - 1) / 2;
        for (int n = 0; n < M; ++n) {
            int k = n - mid;
            double ideal = 0.0;
            if (k != 0 && (k & 1)) { // odd k
                ideal = 2.0 / (M_PI * (double)k);
            }
            double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (M - 1));
            double coeff = ideal * w;
            // Force antisymmetry
            if (n < mid) {
                // We'll mirror after the loop; first compute all
            }
            h[n] = (q31_t) llround(coeff * 2147483647.0);
        }
        // Enforce antisymmetry and zero center
        for (int n = 0; n < mid; ++n) {
            h[n] = (q31_t) llround(-(double)h[HTAPS - 1 - n]);
        }
        h[mid] = 0;
    }

    // ---------- DDS sine table ----------
    void buildSinLUT_Q31() {
        for (int i = 0; i < LUTN; ++i) {
            double s = sin(2.0 * M_PI * i / LUTN);
            sinLUT[i] = (q31_t) llround(s * 2147483647.0);
        }
    }
    void sincos_q31(uint32_t ph, q31_t& s, q31_t& c) {
        // index from upper bits
        uint32_t idx = ph >> (PHASE_BITS - 10);      // for 1024 table
        uint32_t idx90 = (idx + LUTN/4) & (LUTN - 1);
        s = sinLUT[idx];
        c = sinLUT[idx90];
    }
};
