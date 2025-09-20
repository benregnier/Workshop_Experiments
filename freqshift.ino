#include "ComputerCard.h"
#include <math.h>
#include <stdint.h>

static constexpr int    HTAPS     = 63;        // Hilbert taps (odd)
float h[HTAPS];               // Hilbert coefficients (odd, antisymmetric)

// Small sine LUT (optionally increase to 2048 for lower THD)
static constexpr int LUTN = 1024;
float sinLUT[LUTN];


class FreqShifterFloat : public ComputerCard {
public:
    // ===== Config =====
    static constexpr int    FS        = 48000;     // sample rate

    static constexpr float  MAX_SHIFT = 4000.0f;   // Hz limit for knob mapping
    static constexpr bool   UPPER_SB  = true;      // true=upper, false=lower

    static constexpr uint32_t STATUS_LED_INDEX      = 0;                     // LED to flash
    static constexpr uint32_t STATUS_TOGGLE_SAMPLES = FS / 2;                // toggle twice per second

    void InitStatusLed() {
        statusCounter = 0;
        statusLedOn = false;
        LedOff(STATUS_LED_INDEX);
    }

    // FIR state

    float delayLine[HTAPS] = {0}; // circular buffer for FIR
    int   idx = 0;
    int   groupDelay = (HTAPS - 1) / 2;
    float fifoDelay[128] = {0};   // delay to align I with Q (must >= groupDelay)
    int   dWrite = 0, dRead = 128 - groupDelay; // ring pointers

    // DDS state
    uint32_t phase = 0;
    static constexpr uint32_t PHASE_BITS = 32;
    static constexpr uint32_t PHASE_MASK = 0xFFFFFFFFu;



    // Map your desired shift: here we map X knob (0..1) to ±MAX_SHIFT
    inline float knobToShiftHz() {
        float k = KnobVal(X);           // 0..1 from library; adjust if your API differs
        return (k * 2.0f - 1.0f) * MAX_SHIFT;
    }

    // Main 48 kHz loop
    void ProcessSample() override {
        float x = AudioIn1();              // input sample (−1..+1)
        float I = pushDelay(x);            // matched delay
        float Q = hilbert(x);              // 90° shifted

        // DDS
        float shiftHz = knobToShiftHz();
        uint32_t phaseInc = (uint32_t)llround((double)shiftHz * (double)((uint64_t)1 << PHASE_BITS) / (double)FS);
        phase += phaseInc;

        float cs, sn;
        sincosLUT(phase, sn, cs);

        // Single-sideband mixing
        float y = UPPER_SB ? (I * cs - Q * sn)
                           : (I * cs + Q * sn);

        // Optional tiny limiter
        if (y > 1.0f) y = 1.0f; else if (y < -1.0f) y = -1.0f;

        AudioOut1(y);

        updateStatusLed();
    }

private:


    // ---- FIR process: returns Q (Hilbert output) ----
    float hilbert(float x) {
        delayLine[idx] = x;
        float acc = 0.0f;
        int p = idx;
        for (int n = 0; n < HTAPS; ++n) {
            acc += h[n] * delayLine[p];
            if (--p < 0) p = HTAPS - 1;
        }
        if (++idx >= HTAPS) idx = 0;
        return acc;
    }

    // ---- Matching pure delay for I branch ----
    float pushDelay(float x) {
        fifoDelay[dWrite] = x;
        if (++dWrite >= (int) (sizeof(fifoDelay)/sizeof(fifoDelay[0]))) dWrite = 0;
        float y = fifoDelay[dRead];
        if (++dRead >= (int) (sizeof(fifoDelay)/sizeof(fifoDelay[0]))) dRead = 0;
        return y;
    }

    inline void sincosLUT(uint32_t ph, float& s, float& c) {
        // Use upper bits as index (wrap is automatic)
        uint32_t idx = ph >> (PHASE_BITS - 10); // for 1024 points
        uint32_t idx90 = (idx + LUTN / 4) & (LUTN - 1);
        s = sinLUT[idx];
        c = sinLUT[idx90];
    }

    void updateStatusLed() {
        if (++statusCounter >= STATUS_TOGGLE_SAMPLES) {
            statusCounter = 0;
            statusLedOn = !statusLedOn;
            LedOn(STATUS_LED_INDEX, statusLedOn);
        }
    }

    uint32_t statusCounter = 0;
    bool statusLedOn = false;
};


// ---- FIR Hilbert: build Hamming‑windowed ideal transformer ----
void buildHilbert() {
    // Ideal hilbert: h[n] = 2/(pi*n) for n odd; 0 for n even; n centered at 0
    // Then apply Hamming window and force antisymmetry.
    const int M = HTAPS;
    const int mid = (M - 1) / 2;
    for (int n = 0; n < M; ++n) {
        int k = n - mid;
        float ideal = 0.0f;
        if (k != 0 && (k & 1)) { // odd k
            ideal = 2.0f / (float)(M_PI * k);
        }
        // Hamming window
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n / (M - 1));
        h[n] = ideal * w;
    }
    // Force perfect antisymmetry numerically
    for (int n = 0; n < mid; ++n) {
        h[n] = -h[M - 1 - n];
    }
    h[mid] = 0.0f; // center tap is zero
}


// ---- LUT sine/cos utilities ----
void buildSinLUT() {
    for (int i = 0; i < LUTN; ++i) {
        sinLUT[i] = sinf(2.0f * (float)M_PI * i / LUTN);
    }
}

// Global instance
FreqShifterFloat fs;

void setup() {
    fs.EnableNormalisationProbe();
    fs.InitStatusLed();
    buildHilbert();
    buildSinLUT();
}

void loop() {
    fs.Run(); // This will call ProcessSample() at 48kHz
}
