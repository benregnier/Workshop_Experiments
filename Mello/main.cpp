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
    uint32_t pos = 0;
    uint32_t rate = 1u << 16; // Q16 phase increment
    int32_t env = 0;          // Q15 envelope
    int32_t lp = 0;           // low-pass accumulator
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
    int32_t toneQ15_ = 16384; // Q15 tone value

    static constexpr int32_t kEnvSlewQ15 = 26; // ~60 ms to settle (0.0008 * 2^15)
    static constexpr int32_t kToneBaseQ15 = 131; // 0.004 * 2^15
    static constexpr int32_t kToneScaleQ15 = 983; // 0.03 * 2^15
    static constexpr int32_t kBrightBaseQ15 = 16384; // 0.5 * 2^15
    static constexpr int32_t kBrightScaleQ15 = 19661; // 0.6 * 2^15
    static constexpr uint32_t kPhaseMask = (SampleBank::kSampleLength << 16) - 1;

    static constexpr std::array<uint32_t, 256> kExp2Table = [] {
        std::array<uint32_t, 256> lut{};
        for (int i = 0; i < 256; ++i) {
            double val = std::pow(2.0, static_cast<double>(i) / 256.0);
            lut[i] = static_cast<uint32_t>(val * static_cast<double>(1u << 16) + 0.5);
        }
        return lut;
    }();

    void updateControls() {
        uint16_t knob = KnobVal(Knob::Main);
        toneQ15_ = static_cast<int32_t>((static_cast<uint32_t>(knob) * 32768u + 2047u) / 4095u);

        // Aftertouch from AudioIn1: values over ~5 V (~1300 in 12-bit) boost tone.
        int16_t aftertouchRaw = AudioIn1();
        int16_t afterAbs = aftertouchRaw >= 0 ? aftertouchRaw : static_cast<int16_t>(-aftertouchRaw);
        if (afterAbs > 1300) {
            int32_t extraRaw = afterAbs - 1300;
            int32_t extraScaled = (static_cast<int32_t>(extraRaw) * 32768) / 2047;
            int32_t extra = (static_cast<int64_t>(extraScaled) * 13107 + 16384) >> 15; // *0.4
            toneQ15_ += extra;
            if (toneQ15_ > 32768) toneQ15_ = 32768;
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
        int32_t volts = cv / 256;
        int32_t remainder = cv % 256;
        if (remainder < 0) {
            remainder += 256;
            --volts;
        }

        uint32_t rate = kExp2Table[static_cast<uint8_t>(remainder)];
        if (volts >= 0) {
            if (volts > 8) volts = 8;
            rate <<= volts;
        } else {
            int32_t shift = -volts;
            if (shift > 8) shift = 8;
            rate >>= shift;
        }

        voices_[ch].rate = rate;
        voices_[ch].sampleIndex = sampleIndex_[ch];
    }

    int16_t renderVoice(int ch) {
        Voice &v = voices_[ch];
        const auto &table = samples_.bank[v.sampleIndex];

        // Linear interpolation
        uint32_t idx = (v.pos >> 16) & (SampleBank::kSampleLength - 1);
        uint32_t frac = v.pos & 0xFFFF;
        int32_t a = table[idx];
        int32_t b = table[(idx + 1) & (SampleBank::kSampleLength - 1)];
        int32_t sample = a + static_cast<int32_t>((static_cast<int64_t>(b - a) * frac) >> 16);

        // Integrator for dark/bright balance
        int32_t toneCoef = kToneBaseQ15 + static_cast<int32_t>((static_cast<int64_t>(kToneScaleQ15) * toneQ15_) >> 15);
        v.lp += static_cast<int32_t>((static_cast<int64_t>(toneCoef) * (sample - v.lp)) >> 15);
        int32_t bright = sample - v.lp;
        int32_t brightGain = kBrightBaseQ15 + static_cast<int32_t>((static_cast<int64_t>(kBrightScaleQ15) * toneQ15_) >> 15);
        int32_t shaped = v.lp + static_cast<int32_t>((static_cast<int64_t>(bright) * brightGain) >> 15);

        // Envelope
        int32_t target = gate_[ch] ? 32768 : 0;
        v.env += static_cast<int32_t>((static_cast<int64_t>(target - v.env) * kEnvSlewQ15) >> 15);
        int32_t out = static_cast<int32_t>((static_cast<int64_t>(shaped) * v.env) >> 15);

        // Advance phase
        v.pos = (v.pos + v.rate) & kPhaseMask;

        return sat12(out);
    }
};

int main() {
    MelloCard card;
    card.Run();
}
