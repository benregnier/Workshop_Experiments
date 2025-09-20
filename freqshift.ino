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
    static constexpr float  AUDIO_SCALE = 2048.0f;
    static constexpr float  INV_AUDIO_SCALE = 1.0f / AUDIO_SCALE;
    static constexpr uint32_t CONTROL_UPDATE_SAMPLES = 64;                  // control-rate divider
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
    static constexpr float  PHASE_SCALE =
        (float)((double)(1ULL << PHASE_BITS) / (double)FS);                 // Hz->phase scale



    // Map your desired shift: here we map X knob (0..4095) to ±MAX_SHIFT
    inline float knobToShiftHz(uint16_t knobRaw) {
        float k = knobRaw / 4095.0f; // raw knob is 0..4095; normalise to 0..1
        return (k * 2.0f - 1.0f) * MAX_SHIFT;
    }

    // Main 48 kHz loop
    void ProcessSample() override {
        float x = AudioIn1() * INV_AUDIO_SCALE; // input sample scaled to (−1..+1)
        float I = pushDelay(x);            // matched delay
        float Q = hilbert(x);              // 90° shifted

        uint16_t knobRaw = static_cast<uint16_t>(KnobVal(X));
        bool knobChanged = (knobRaw != lastKnobRaw);

        if (controlCounter == 0 || knobChanged) {
            lastKnobRaw = knobRaw;
            updatePhaseIncrement(knobRaw);
        }

        if (++controlCounter >= CONTROL_UPDATE_SAMPLES) {
            controlCounter = 0;
        }

        phase += cachedPhaseInc;

        float cs, sn;
        sincosLUT(phase, sn, cs);

        // Single-sideband mixing
        float y = UPPER_SB ? (I * cs - Q * sn)
                           : (I * cs + Q * sn);

        float scaled = y * AUDIO_SCALE;
        const float maxVal = AUDIO_SCALE - 1.0f;
        const float minVal = -AUDIO_SCALE;
        if (scaled > maxVal) {
            scaled = maxVal;
        } else if (scaled < minVal) {
            scaled = minVal;
        }

        updateStatusLed();

        int16_t out = static_cast<int16_t>(scaled);
        AudioOut1(out);
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
    uint16_t lastKnobRaw = 0;
    uint32_t cachedPhaseInc = 0;
    uint32_t controlCounter = 0;

    void updatePhaseIncrement(uint16_t knobRaw) {
        float shiftHz = knobToShiftHz(knobRaw);
        float phaseIncF = roundf(shiftHz * PHASE_SCALE);
        int32_t phaseIncSigned = static_cast<int32_t>(phaseIncF);
        cachedPhaseInc = static_cast<uint32_t>(phaseIncSigned);
    }
};


// ---- FIR Hilbert: build Hamming‑windowed ideal transformer ----
void buildHilbert() {
    // Ideal hilbert: h[n] = 2/(pi*n) for n odd; 0 for n even; n centered at 0
    // Then apply Hamming window and force antisymmetry.
    const int M = HTAPS;
    const int mid = (M - 1) / 2;
    constexpr float passbandNormFreq = 0.15f; // Representative pass-band frequency (fraction of Fs)
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

    // Measure Hilbert branch gain at a representative pass-band frequency.
    float gainRe = 0.0f;
    float gainIm = 0.0f;
    const float omega = 2.0f * (float)M_PI * passbandNormFreq;
    for (int n = 0; n < M; ++n) {
        float phase = -omega * (float)(n - mid);
        float c = cosf(phase);
        float s = sinf(phase);
        gainRe += h[n] * c;
        gainIm += h[n] * s;
    }
    float gainMag = sqrtf(gainRe * gainRe + gainIm * gainIm);

    if (gainMag > 0.0f) {
        float compensation = 1.0f / gainMag;
        for (int n = 0; n < M; ++n) {
            h[n] *= compensation;
        }
        // Re-enforce antisymmetry after scaling to guarantee Hilbert behaviour.
        for (int n = 0; n < mid; ++n) {
            float val = 0.5f * (h[n] - h[M - 1 - n]);
            h[n] = val;
            h[M - 1 - n] = -val;
        }
        h[mid] = 0.0f;
    }
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
