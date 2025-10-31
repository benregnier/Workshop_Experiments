#include "ComputerCard.h"
#include <stdint.h>

// --- Q15 helpers ---
static inline int16_t sat16(int32_t v) {
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

class CombFilter : public ComputerCard {
public:
    CombFilter() { InitComb(); }

    void InitComb() {
        sampleRate = 48000;

        size = BUF_CAP;
        for (int i = 0; i < size; ++i) buffer[i] = 0;
        writeIdx = 0;

        delaySamples = ms_to_samples(12);
        readIdx = (writeIdx - delaySamples) & (size - 1);

        feedbackQ15 = (int16_t)((32767 * 7) / 10); // ~0.7
        dryQ15 = 16384;  // 0.5
        wetQ15 = 16384;  // 0.5

        dampState  = 0;
        dampA_q15  = 4096;  // ~0.125
    }

    virtual void ProcessSample() {
        int16_t xin = AudioIn1();

        // -------- Delay control (modes) --------
        const int32_t MIN_MS   = 1;
        const int32_t MAX_MS   = 30;
        const int32_t SPAN_MS  = (MAX_MS - MIN_MS); // 29
        const int32_t BASE_MS  = 12;                // Up-mode 1V/oct center
        const int32_t OFFSET_MS_MAX = 10;           // ±offset range in Middle mode (when CV1 patched)

        int32_t D = 0;
        int sw = SwitchVal();
        bool swUp     = (sw == Switch::Up);
        bool swMiddle = (sw == Switch::Middle);
        bool swDown   = (sw == Switch::Down);

        if (swUp) {
            if (Connected(Input::CV1)) {
                // ---- 1 V/oct from CV1; MAIN = semitone offset ----
                int32_t S_cv = (int32_t)CVIn1();        // −2048..+2047
                S_cv = (S_cv * 15) >> 9;                // semitones (~±60)

                int32_t delta = (int32_t)KnobVal(Knob::Main) - 2048; // −2048..+2047
                int32_t S_off = (delta * 3) >> 8;                     // ~±24 st
                if (S_off < -24) S_off = -24;
                if (S_off >  24) S_off =  24;

                int32_t S = S_cv + S_off;

                int32_t D0 = ms_to_samples(BASE_MS);

                int32_t q, r;
                if (S >= 0) { q = S / 12; r = S % 12; }
                else { q = -(( -S + 11) / 12); r = S - 12 * q; }

                static const uint16_t inv_ratio_q15[12] = {
                    32768,30929,29205,27554,26004,24538,23170,21885,20675,19527,18449,17431
                };
                int32_t D_scaled = (int32_t)(((int64_t)D0 * inv_ratio_q15[r] + (1<<14)) >> 15);
                if (q >= 0) { int sh = (q > 16) ? 16 : q; D_scaled >>= sh; }
                else        { int sh = (-q > 16) ? 16 : -q; D_scaled <<= sh; }

                D = D_scaled;
            } else {
                // ---- Up, CV1 NOT connected: MAIN sets 30→1 ms (reversed) ----
                const int64_t den = 4095LL * 1000LL;
                int64_t num = (int64_t)KnobVal(Knob::Main) * SPAN_MS * sampleRate;
                int32_t subSamples = (int32_t)((num + (den >> 1)) / den);
                D = ms_to_samples(MAX_MS) - subSamples;
            }
        } else { // Middle or Down
            if (Connected(Input::CV1)) {
                // ---- Middle/Down, CV1 connected: BASE from CV1 (reversed 30→1 ms) + MAIN offset ----
                // Base from CV1 alone (unipolar 0..4095):
                uint32_t cvu = (uint32_t)(CVIn1() + 2048); // 0..4095
                const int64_t den = 4095LL * 1000LL;
                int64_t num = (int64_t)cvu * SPAN_MS * sampleRate;
                int32_t subSamples = (int32_t)((num + (den >> 1)) / den);
                int32_t D_base = ms_to_samples(MAX_MS) - subSamples;

                // MAIN as ±offset in samples.
                // Positive MAIN (clockwise) should REDUCE delay (keep pitch direction same as Up),
                // so offset sign is negative for positive delta.
                int32_t mainDelta = (int32_t)KnobVal(Knob::Main) - 2048;            // −2048..+2047
                int32_t maxOffSamp = ms_to_samples(OFFSET_MS_MAX);                  // +N ms in samples
                int32_t offSamp = (int32_t)(((int64_t)mainDelta * maxOffSamp) >> 11); // scale by 2048
                offSamp = -offSamp;  // clockwise -> less delay

                D = D_base + offSamp;
            } else {
                // ---- Middle/Down, CV1 NOT connected: MAIN sets 30→1 ms (reversed) ----
                const int64_t den = 4095LL * 1000LL;
                int64_t num = (int64_t)KnobVal(Knob::Main) * SPAN_MS * sampleRate;
                int32_t subSamples = (int32_t)((num + (den >> 1)) / den);
                D = ms_to_samples(MAX_MS) - subSamples;
            }
        }

        // Clamp & apply
        if (D < 1) D = 1;
        if (D >= size) D = size - 1;
        delaySamples = D;
        readIdx = (writeIdx - delaySamples) & (size - 1);

        // -------- Tone/Damping: Knob X -> α (Q15) --------
        {
            // α ≈ 0.02..0.30 (dark..bright)
            const int32_t a_min = 655;   // ~0.02*32768
            const int32_t a_max = 9830;  // ~0.30*32768
            uint32_t tone = KnobVal(Knob::X); // 0..4095
            dampA_q15 = (int16_t)(a_min + ((int64_t)(a_max - a_min) * tone + 2047) / 4095);
        }

        // -------- Feedback: Knob Y (and CV2) -> 0.75..0.99 (Q15) --------
        {
            const int32_t fb_min = (int32_t)((32767 * 75) / 100); // ≈24575
            const int32_t fb_max = (int32_t)((32767 * 99) / 100); // ≈32439
            const int32_t fb_span = fb_max - fb_min;

            uint32_t ky = Connected(Input::CV2)
                ? (((uint32_t)KnobVal(Knob::Y) * (uint32_t)(CVIn2() + 2048)) >> 12)
                : (uint32_t)KnobVal(Knob::Y); // 0..4095

            feedbackQ15 = (int16_t)(fb_min + ((int64_t)fb_span * ky + 2047) / 4095);
        }

        // -------- Comb with damping --------
        int16_t y_delayed = buffer[readIdx];
        {
            int32_t e = (int32_t)y_delayed - dampState;
            dampState = (int16_t)(dampState + ((e * dampA_q15) >> 15));
        }
        int16_t y_damped = dampState;

        int32_t y = (int32_t)xin + ( ((int32_t)feedbackQ15 * (int32_t)y_damped) >> 15 );
        int16_t y_sat = sat16(y);

        buffer[writeIdx] = y_sat;
        writeIdx = (writeIdx + 1) & (size - 1);

        int32_t mixed = ( ((int32_t)dryQ15 * xin) + ((int32_t)wetQ15 * y_sat) ) >> 15;
        int16_t out = sat16(mixed);

        AudioOut1(out);

        PulseOut1(PulseIn1());
        PulseOut2(PulseIn2());

        LedOn(4, swDown);
        LedOn(2, swMiddle);
        LedOn(0, swUp);
        LedBrightness(1, KnobVal(Knob::Main)); // frequency / offset
        LedBrightness(3, KnobVal(Knob::X));    // tone
        LedBrightness(5, KnobVal(Knob::Y));    // feedback
    }

private:
    inline int32_t ms_to_samples(int32_t ms) const {
        return (int32_t)((int64_t)sampleRate * (int64_t)ms / 1000LL);
    }

    static const int BUF_CAP = 8192;
    int16_t buffer[BUF_CAP];
    int     size        = BUF_CAP;
    int     writeIdx    = 0;
    int     readIdx     = 0;
    int     delaySamples= 240;

    int16_t feedbackQ15 = 22936;
    int16_t dryQ15      = 16384;
    int16_t wetQ15      = 16384;
    int     sampleRate  = 48000;

    // Tone/damping
    int16_t dampState   = 0;
    int16_t dampA_q15   = 4096;
};

int main() {
    CombFilter cf;
    cf.EnableNormalisationProbe();
    cf.Run();
}
