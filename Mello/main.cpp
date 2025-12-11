#include "ComputerCard.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>

// Mellotron emulator
// Load in your own sounds! Will implement the "sample upload" html example
// - Main knob controls tone/brightness
// - Two channels
// - Audio In: amplitude. values over 5 volts increase tone/brightness (aftertouch)
// - CV In: pitch (1V/oct)
// - Pulse in: If patched, acts as gate
// - X and Y select samples to be used for each channel
// - Switch up or momentary: drone mode
// Outputs
// - Audio Out 1 and 2 for two channels of audio
//
// Implementation notes
// --------------------
// - Two independent sample voices share a small baked-in bank of three loops.
// - CV1/2 modulate pitch per channel; Pulse1/2 gate playback unless the switch
//   is in the Up (drone) position or the inputs are unpatched.
// - AudioIn1 is treated as aftertouch: large peaks brighten the tone.
// - The main knob sets base brightness; X/Y choose the sample for channel 1/2.

static inline int16_t sat12(int32_t x) {
    if (x > 2047) return 2047;
    if (x < -2048) return -2048;
    return static_cast<int16_t>(x);
}

// Simple sample bank generated on boot (kept in RAM/flash on the card).
struct SampleBank {
    static constexpr int kSampleLength = 2048; // power of two for masking
    using Sample = std::array<int16_t, kSampleLength>;
    std::array<Sample, 3> bank{};

    SampleBank() {
        constexpr float twoPi = 6.28318530718f;
        for (int i = 0; i < kSampleLength; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(kSampleLength);

            // Sample 0: flute-like sine blend
            float s0 = std::sin(twoPi * t) * 0.75f + std::sin(twoPi * 3.0f * t) * 0.25f;
            bank[0][i] = static_cast<int16_t>(s0 * 1700.0f);

            // Sample 1: airy strings (soft saw with slow AM)
            float saw = 2.0f * (t - std::floor(t + 0.5f));
            float am = 0.7f + 0.3f * std::sin(twoPi * 0.25f * t);
            bank[1][i] = static_cast<int16_t>(saw * am * 1800.0f);

            // Sample 2: choir-ish cluster
            float s2 = std::sin(twoPi * 0.5f * t) + 0.5f * std::sin(twoPi * 2.5f * t) +
                       0.35f * std::sin(twoPi * 5.0f * t + 0.2f);
            bank[2][i] = static_cast<int16_t>(s2 * 1500.0f);
        }
    }
};

struct Voice {
    float pos = 0.0f;
    float rate = 1.0f;
    float env = 0.0f;
    float lp = 0.0f;
    int sampleIndex = 0;
};

class MelloCard : public ComputerCard {
public:
    MelloCard() = default;

    void ProcessSample() override {
        updateControls();

        int16_t outA = renderVoice(0);
        int16_t outB = renderVoice(1);

        AudioOut1(outA);
        AudioOut2(outB);

        // LED feedback: channel activity + sample choice
        LedBrightness(0, gate_[0] ? 4095 : 0);
        LedBrightness(1, gate_[1] ? 4095 : 0);
        LedBrightness(2, 1200 + sampleIndex_[0] * 1400);
        LedBrightness(3, 1200 + sampleIndex_[1] * 1400);
    }

private:
    SampleBank samples_{};
    Voice voices_[2]{};
    bool gate_[2] = {true, true};
    int sampleIndex_[2] = {0, 1};
    float tone_ = 0.5f;

    static constexpr float kEnvSlew = 0.0008f; // ~60 ms to settle
    static constexpr float kToneBase = 0.004f;

    void updateControls() {
        tone_ = static_cast<float>(KnobVal(Knob::Main)) / 4095.0f;

        // Aftertouch from AudioIn1: values over ~5 V (~1300 in 12-bit) boost tone.
        int16_t aftertouchRaw = AudioIn1();
        int16_t afterAbs = aftertouchRaw >= 0 ? aftertouchRaw : static_cast<int16_t>(-aftertouchRaw);
        if (afterAbs > 1300) {
            float extra = static_cast<float>(afterAbs - 1300) / 2047.0f;
            tone_ = std::min(1.0f, tone_ + extra * 0.4f);
        }

        sampleIndex_[0] = selectorToIndex(KnobVal(Knob::X));
        sampleIndex_[1] = selectorToIndex(KnobVal(Knob::Y));

        bool drone = SwitchVal() == Switch::Up;
        gate_[0] = drone || !Connected(Input::Pulse1) || PulseIn1();
        gate_[1] = drone || !Connected(Input::Pulse2) || PulseIn2();

        updatePitch(0);
        updatePitch(1);
    }

    static int selectorToIndex(uint16_t selector) {
        // Map 0-4095 → 0,1,2 with soft edges
        uint32_t bucket = (static_cast<uint32_t>(selector) * 3u) / 4096u;
        return bucket > 2 ? 2 : static_cast<int>(bucket);
    }

    void updatePitch(int ch) {
        int16_t cv = (ch == 0) ? CVIn1() : CVIn2();
        // ADC range maps roughly ±2048 to ±8 V, so ~256 counts per volt.
        float volts = static_cast<float>(cv) / 256.0f;
        // 1V/octave scaling
        float ratio = std::pow(2.0f, volts);
        voices_[ch].rate = ratio;
        voices_[ch].sampleIndex = sampleIndex_[ch];
    }

    int16_t renderVoice(int ch) {
        Voice &v = voices_[ch];
        const auto &table = samples_.bank[v.sampleIndex];

        // Linear interpolation
        int32_t idx = static_cast<int32_t>(v.pos);
        float frac = v.pos - static_cast<float>(idx);
        int32_t idxNext = (idx + 1) & (SampleBank::kSampleLength - 1);
        float a = static_cast<float>(table[idx]);
        float b = static_cast<float>(table[idxNext]);
        float sample = a + (b - a) * frac;

        // Integrator for dark/bright balance
        float toneCoef = kToneBase + tone_ * 0.03f;
        v.lp += toneCoef * (sample - v.lp);
        float bright = sample - v.lp;
        float shaped = v.lp + bright * (0.5f + 0.6f * tone_);

        // Envelope
        float target = gate_[ch] ? 1.0f : 0.0f;
        v.env += (target - v.env) * kEnvSlew;
        float out = shaped * v.env;

        // Advance phase
        v.pos += v.rate;
        while (v.pos >= SampleBank::kSampleLength) v.pos -= SampleBank::kSampleLength;
        while (v.pos < 0.0f) v.pos += SampleBank::kSampleLength;

        return sat12(static_cast<int32_t>(out));
    }
};

int main() {
    MelloCard card;
    card.Run();
}
