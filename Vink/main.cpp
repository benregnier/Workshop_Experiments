#include "ComputerCard.h"
#include <cstdint>
#include <cstring>
#include <algorithm>

/**
 * @file main.cpp
 * @brief Vink: dual delay loops with saturation for Jaap Vink /
 * Roland Kayn style feedback patching.
 *
 * Reference performance: https://www.youtube.com/watch?v=X_Bcr_HS9XM
 *
 * Blocks
 *  - Two delay taps (≈250 ms max, down to 3 samples)
 *  - Sigmoid soft clipper per tap (toggleable)
 *
 * Controls
 *  - `Knob::Main`   : shared delay time, averaged with `CVIn1`
 *  - `Knob::X`      : tap 2 offset from tap 1, averaged with `CVIn2`
 *  - `Knob::Y`      : crossfade dry ↔ saturated
 *  - `Switch::Up`   : split I/O; `Switch::Center/Down` : shared mix
 *
 * I/O mapping
 *  - `AudioIn1/2` : summed if both patched
 *  - `CVIn1/2`    : tap modulation (averaged with knobs)
 *  - `AudioOut1`  : tap 1 or mono mix
 *  - `AudioOut2`  : tap 2 (split) or mono mix
 *  - `CVOut1/2`   : chaotic slow LFO pair
 *  - `PulseOut1/2`: pulse tracks tap periods
 */

// === Helper utilities ========================================================
/// Clamp a 32-bit value to the Q15 audio range.
static inline int16_t sat16(int32_t x){
    return static_cast<int16_t>(std::clamp<int32_t>(x, -32768, 32767));
}
/// Clamp a 32-bit value to the 12-bit audio range.
static inline int16_t sat12(int32_t x){
    return static_cast<int16_t>(std::clamp<int32_t>(x, -2048, 2047));
}
/// Convert packed 12-bit audio (JLC board format) to Q15.
static inline int16_t audio12_to_q15(int16_t x){ return (int16_t)(((int32_t)x) << 4); }
/// Convert Q15 back to packed 12-bit audio with rounding.
static inline int16_t q15_to_audio12(int32_t x){
    int32_t rounded;
    if (x >= 0) {
        rounded = (x + 8) >> 4;
    } else {
        rounded = -(((-x) + 8) >> 4);
    }
    return sat12(rounded);
}
static inline uint16_t led_from_audio12(int16_t x){
    int32_t v = static_cast<int32_t>(x) + 2048;
    return static_cast<uint16_t>(std::clamp<int32_t>(v, 0, 4095));
}

static inline uint32_t knobToMixQ16(uint16_t knob){
    uint32_t mix = ((uint32_t)knob << 16) / 4095u;
    if (mix > 65536u) mix = 65536u;
    return mix;
}

static inline int16_t mixDryWet(int16_t dry, int16_t wet, uint32_t mixQ16){
    uint32_t wetMix = std::min<uint32_t>(mixQ16, 65536u);
    uint32_t dryMix = 65536u - wetMix;
    int64_t blended = ( (int64_t)dry * (int64_t)dryMix + (int64_t)wet * (int64_t)wetMix ) >> 16;
    return sat12((int32_t)blended);
}

static inline int16_t SigSat(int16_t x);


// === Sigmoid saturation ======================================================
/**
 * @brief Scales input into the sigmoid wave-shaper domain.
 */
struct SigmoidSaturator {
    // Drive in Q12 (4096 ≈ 1.0). Range expanded via knob mapping.
    uint16_t driveQ12 = 4096;

    inline int16_t process(int16_t x) const {
        int32_t scaled = ((int32_t)x * (int32_t)driveQ12) >> 12;
        scaled = sat12(scaled);
        return SigSat((int16_t)scaled);
    }

    void setDriveQ12(uint16_t drive){ driveQ12 = std::max<uint16_t>(drive, 256u); }
};

/**
 * @brief Sigmoid saturator approximation x/(1+|x|) optimized for 12-bit audio.
 * Thanks to Allsnop @ the Serge Discord!
 */
static inline int16_t SigSat(int16_t x)
{
    int32_t ax = (x >= 0) ? x : -(int32_t)x;
    int32_t denom = 2048 + ax;          // 12-bit range + |x|
    int32_t num   = (int32_t)x * 2047;  // target range scaling

    if (num >= 0) {
        num += denom / 2;               // round toward nearest
    } else {
        num -= denom / 2;
    }

    int32_t result = num / denom;
    return sat12(result);
}


// === Delay lines =============================================================
/**
 * @brief Circular-buffer delay line with fp16 interpolation and slew-limited
 * delay-time modulation.
 */
class SmoothDelay {
public:
    // maxDelayMs: upper bound (e.g., 1000 = 1s)
    explicit SmoothDelay(uint32_t sampleRate, uint32_t maxDelayMs)
    : sr_(sampleRate),
      bufSize_(0),
      mask_(0),
      buffer_(nullptr),
      w_(0),
      current_fp16_(0),
      target_fp16_(0),
      slew_step_fp16_(0),
      initialised_(false)
    {
        uint32_t maxSamples = (uint64_t)sampleRate * maxDelayMs / 1000u + 2u; // +2 for interp
        bufSize_ = 1u; while (bufSize_ < maxSamples) bufSize_ <<= 1;
        mask_ = bufSize_ - 1u;
        buffer_ = new int16_t[bufSize_];
        std::memset(buffer_, 0, bufSize_ * sizeof(int16_t));
        setDelayMs(250);                 // default 250 ms
        setSlewPerSecondMs(50.0f);       // default: limit jumps ~50 ms/s (optional)
    }

    ~SmoothDelay(){ delete[] buffer_; }

    // --- Delay control (choose one style) ---

    // 1) Set target delay in ms; the class slews toward it to avoid clicks
    void setDelayMs(uint32_t ms){
        uint64_t fp = ((uint64_t)sr_ * ms << 16) / 1000u;
        setDelaySamplesFP16(fp);
    }

    // 2) Set target delay in 16.16 samples (for LFOs/envelopes in your code)
    //    Call per block or per sample; slewing still applies.
    void setDelaySamplesFP16(uint64_t delay_fp16){
        uint64_t max_fp16 = ((uint64_t)(bufSize_-2) << 16);
        if (delay_fp16 > max_fp16) delay_fp16 = max_fp16;
        target_fp16_ = (uint32_t)delay_fp16;
        if (!initialised_) {
            current_fp16_ = target_fp16_;
            initialised_ = true;
        }
    }

    // Optional: control slew (how fast we allow delay to change)
    // For modulation: higher = snappier; 0 disables slewing (still clickless, but rapid moves can zipper)
    void setSlewPerSecondMs(float ms_per_sec){
        // convert to fp16 samples / sample: step = (sr * ms/1000) << 16 / sr = (ms << 16)/1000
        // so step per sample (fp16) = (ms_per_sec * 65536 / 1000)
        if (ms_per_sec < 0) ms_per_sec = 0;
        slew_step_fp16_ = (uint32_t)((ms_per_sec * 65536.0f) / 1000.0f);
    }

    // If you'd rather specify slew in samples-per-sample (fp16), use this:
    void setSlewStepFP16(uint32_t step_fp16){ slew_step_fp16_ = step_fp16; }

    // --- Process one sample ---
    inline int16_t process(int16_t in){
        // 1) Slew current delay toward target to avoid sudden tap jumps
        if (slew_step_fp16_ == 0){
            current_fp16_ = target_fp16_;
        } else if (current_fp16_ != target_fp16_){
            int32_t diff = (int32_t)target_fp16_ - (int32_t)current_fp16_;
            int32_t step = (diff > 0) ? (int32_t)slew_step_fp16_ : -(int32_t)slew_step_fp16_;
            if ((diff > 0 && diff < step) || (diff < 0 && diff > step)) {
                current_fp16_ = target_fp16_;
            } else {
                current_fp16_ += step;
            }
        }

        // 2) Compute integer + fractional delay
        uint32_t delayInt  = current_fp16_ >> 16;
        uint32_t delayFrac = current_fp16_ & 0xFFFFu;

        // 3) Read with linear interpolation between base and base-1
        uint32_t base = (w_ - delayInt) & mask_;
        bool wrapped = base < prev_base_;
        prev_base_ = base;
        int32_t  x0   = buffer_[base];
        int32_t  x1   = buffer_[(base - 1) & mask_];

        // y = (x0*(1-f) + x1*f), f in [0..1) as 16-bit fraction
        uint32_t f = delayFrac;
        int32_t y  = ( (x0 * (int32_t)(65536u - f)) + (x1 * (int32_t)f) ) >> 16;
        int16_t out = sat16(y);

        // Fade the start of each tap loop to suppress single-sample discontinuities
        // (e.g., with heavy feedback) without low-passing the whole path.
        if (wrapped) {
            edge_fade_ = kEdgeFadeLen;
        }
        if (edge_fade_ > 0) {
            uint32_t weightPrev = edge_fade_;
            uint32_t weightNew = kEdgeFadeLen - edge_fade_;
            int32_t blended = ( (int32_t)last_out_ * (int32_t)weightPrev
                              + (int32_t)out       * (int32_t)weightNew );
            out = sat16( blended / (int32_t)kEdgeFadeLen );
            edge_fade_--;
        }
        last_out_ = out;

        // 4) Write current input and advance
        buffer_[w_] = in;
        w_ = (w_ + 1) & mask_;

        return out; // pure delayed signal
    }

    void clear(){ std::memset(buffer_, 0, bufSize_*sizeof(int16_t)); }

    // Expose the current delay setting (16.16 samples)
    uint32_t currentDelaySamplesFP16() const { return current_fp16_; }

private:
    uint32_t sr_;
    uint32_t bufSize_;
    uint32_t mask_;
    int16_t* buffer_;
    uint32_t w_;
    static constexpr uint8_t kEdgeFadeLen = 4; // samples to crossfade after wrap
    uint32_t prev_base_{0};
    uint8_t  edge_fade_{0};
    int16_t  last_out_{0};

    // Delay (16.16 samples)
    uint32_t current_fp16_;
    uint32_t target_fp16_;

    // Slew step per sample in 16.16 samples
    uint32_t slew_step_fp16_;
    bool initialised_;
};

class Vink : public ComputerCard
{
    static constexpr uint32_t kSampleRate = 48000u;
    static constexpr uint32_t kMaxDelayMs = 250u;
    static constexpr uint32_t kMaxDelaySamples = (kSampleRate * kMaxDelayMs) / 1000u;
    static constexpr uint32_t kMaxDelaySamplesFP16 = kMaxDelaySamples << 16;
    static constexpr uint32_t kMinDelaySamplesFP16 = 1u << 16; // ensure at least one sample of delay
    static constexpr uint32_t kDelayRangeSamplesFP16 = kMaxDelaySamplesFP16 - kMinDelaySamplesFP16;

    static constexpr uint32_t msToFP16(uint32_t ms){
        return (uint32_t)(((uint64_t)kSampleRate * ms << 16) / 1000u);
    }

    /**
     * @brief Chaotic logistic-map LFO with heavy smoothing.
     */
    struct SlowChaosLFO {
        void configure(float seed, float rate, float smooth, uint32_t updateIntervalSamples){
            logistic_q31_ = floatToUnsignedQ31(seed);
            if (logistic_q31_ <= 0u) logistic_q31_ = 1u;
            r_q30_ = floatToQ30(rate);
            if (r_q30_ < (1u << 30)) r_q30_ = (1u << 30);
            smoothing_q31_ = floatToQ31(smooth);
            if (smoothing_q31_ == 0u) smoothing_q31_ = 1u;
            interval_ = updateIntervalSamples ? updateIntervalSamples : 1u;
            counter_ = interval_;
            target_q31_ = signedFromUnsigned(logistic_q31_);
            value_q31_ = target_q31_;
        }

        inline int32_t step(){
            if (--counter_ == 0u) {
                counter_ = interval_;
                uint32_t x = logistic_q31_;
                uint32_t one_minus_x = 0x7FFFFFFFu - x;
                if (one_minus_x == 0u) {
                    // avoid degeneracy
                    one_minus_x = 1u;
                    if (x > 1u) {
                        x -= 1u;
                    }
                }
                uint64_t prod = (uint64_t)x * (uint64_t)one_minus_x;          // Q1.31 * Q1.31 -> Q2.62
                uint32_t mid = (uint32_t)(prod >> 31);                         // back to Q1.31
                uint64_t next = (uint64_t)r_q30_ * (uint64_t)mid;              // Q2.30 * Q1.31 -> Q3.61
                next >>= 30;                                                   // -> Q1.31 range
                if (next <= 0u) next = 1u;
                if (next >= 0x7FFFFFFEu) next = 0x7FFFFFFEu;
                logistic_q31_ = (uint32_t)next;
                target_q31_ = signedFromUnsigned(logistic_q31_);
            }

            int64_t diff = (int64_t)target_q31_ - (int64_t)value_q31_;
            int64_t delta = (diff * (int64_t)smoothing_q31_) >> 31;            // Q1.31
            value_q31_ += (int32_t)delta;
            return value_q31_;
        }

    private:
        static uint32_t floatToUnsignedQ31(float v){
            if (v <= 0.0f) return 1u;
            if (v >= 0.999999f) return 0x7FFFFFFEu;
            return (uint32_t)(v * 2147483647.0f);
        }

        static uint32_t floatToQ30(float v){
            if (v <= 0.0f) return 0u;
            if (v >= 3.999999f) return 0xFFFFFFFFu;
            return (uint32_t)(v * (float)(1u << 30));
        }

        static uint32_t floatToQ31(float v){
            if (v <= 0.0f) return 0u;
            if (v >= 0.999999f) return 0x7FFFFFFFu;
            return (uint32_t)(v * 2147483647.0f);
        }

        static int32_t signedFromUnsigned(uint32_t v){
            return (int32_t)(((int64_t)v << 1) - 0x7FFFFFFFLL);
        }

        uint32_t logistic_q31_{1u};
        uint32_t r_q30_{1u << 30};
        uint32_t smoothing_q31_{1u};
        uint32_t interval_{1u};
        uint32_t counter_{1u};
        int32_t  target_q31_{0};
        int32_t  value_q31_{0};
    };

public:
    Vink()
        : dl1_(kSampleRate, kMaxDelayMs),
          dl2_(kSampleRate, kMaxDelayMs)
    {
        initialiseDelay(dl1_, msToFP16(100));
        initialiseDelay(dl2_, msToFP16(50));
        sat = makeSaturator();
        sat2 = makeSaturator();

        PulseOut1(false);
        PulseOut2(false);

        lfo1_.configure(0.412345f, 3.9935f, 2.0e-7f, 32768u); //
        lfo2_.configure(0.762531f, 3.9855f, 3.0e-7f, 16384u); //
    }

    /**
     * @brief Audio callback executed once per sample.
     */
    void ProcessSample() override
    {
        Switch sw = SwitchVal();
        bool splitMode = (sw == Switch::Up);

        // 2 delay lines
        int16_t in1 = AudioIn1();
        int16_t in2 = AudioIn2();
        bool audio2Connected = Connected(Input::Audio2);
        int16_t sharedIn = in1;
        if (audio2Connected) {
            sharedIn = (int16_t)((in1 + in2) >> 1);
        }

        int16_t delayInput1 = splitMode ? in1 : sharedIn;
        int16_t delayInput2 = splitMode ? (audio2Connected ? in2 : sharedIn) : sharedIn;

        int16_t delay1 = dl1_.process(delayInput1);  // delayed only (you mix elsewhere)
        int16_t delay2 = dl2_.process(delayInput2);

        uint32_t center_fp16 = knobToDelayFP16(KnobVal(Knob::Main));
        uint32_t spread_fp16 = knobToDelayFP16(KnobVal(Knob::X));

        uint32_t delay1_fp16 = clampDelayFP16(center_fp16);
        if (Connected(Input::CV1)) {
            delay1_fp16 = averageWithCv(delay1_fp16, CVIn1());
        }

        uint32_t delay2_fp16 = clampDelayFP16((int64_t)center_fp16 + spread_fp16);
        if (Connected(Input::CV2)) {
            delay2_fp16 = averageWithCv(delay2_fp16, CVIn2());
        }

        dl1_.setDelaySamplesFP16(delay1_fp16);
        dl2_.setDelaySamplesFP16(delay2_fp16);

        auto updatePulse = [&](uint32_t delay_fp16,
                               uint32_t& countdown,
                               uint32_t& hold,
                               bool& state,
                               auto&& setter){
            uint32_t periodSamples = std::max<uint32_t>(1u, (delay_fp16 + 0x8000u) >> 16);
            if (countdown > periodSamples) {
                countdown = periodSamples;
            }

            bool triggered = false;
            if (countdown == 0u) {
                countdown = periodSamples;
                if (!state) {
                    state = true;
                    setter(true);
                }
                hold = std::max<uint32_t>(1u, periodSamples >> 2);
                triggered = true;
            } else {
                countdown--;
            }

            if (state && !triggered) {
                if (hold > 0u) {
                    hold--;
                    if (hold == 0u) {
                        state = false;
                        setter(false);
                    }
                }
            }
        };

        updatePulse(delay1_fp16,
                    pulseCountdown1_,
                    pulseHold1_,
                    pulseState1_,
                    [this](bool v){ PulseOut1(v); LedOn(4, v); });

        updatePulse(delay2_fp16,
                    pulseCountdown2_,
                    pulseHold2_,
                    pulseState2_,
                    [this](bool v){ PulseOut2(v); LedOn(5, v); });

        uint32_t mixQ16 = knobToMixQ16(KnobVal(Knob::Y));
        auto applyCrossfade = [mixQ16](int16_t dry, int16_t wet){
            return mixDryWet(dry, wet, mixQ16);
        };

        if (splitMode) {
            int16_t saturated1 = sat.process(delay1);
            int16_t saturated2 = sat2.process(delay2);
            int16_t out1 = applyCrossfade(delay1, saturated1);
            int16_t out2 = applyCrossfade(delay2, saturated2);
            AudioOut1(out1);
            AudioOut2(out2);
            LedBrightness(0, led_from_audio12(out1));
            LedBrightness(1, led_from_audio12(out2));
        } else {
            int16_t mix = (int16_t)(((int32_t)delay1 + (int32_t)delay2) >> 1);
            int16_t saturated = sat.process(mix);
            int16_t outMono = applyCrossfade(mix, saturated);
            AudioOut1(outMono);
            AudioOut2(outMono);
            LedBrightness(0, led_from_audio12(outMono));
            LedBrightness(1, led_from_audio12(outMono));
        }

        int32_t lfo1 = lfo1_.step();
        int32_t lfo2 = lfo2_.step();
        int32_t cvOut1 = lfo1 >> 20; // Q1.31 -> approx ±2048
        int32_t cvOut2 = lfo2 >> 20;
        if (cvOut1 < -2048) cvOut1 = -2048;
        if (cvOut1 > 2047) cvOut1 = 2047;
        if (cvOut2 < -2048) cvOut2 = -2048;
        if (cvOut2 > 2047) cvOut2 = 2047;
        CVOut1((int16_t)cvOut1);
        LedBrightness(2, (uint16_t)(cvOut1 + 2048)); // debug
        LedBrightness(3, (uint16_t)(cvOut2 + 2048)); // debug
        CVOut2((int16_t)cvOut2);

    }

private:
    static SigmoidSaturator makeSaturator(){
        SigmoidSaturator s;
        s.setDriveQ12(4096);
        return s;
    }

    static void initialiseDelay(SmoothDelay& delay, uint32_t defaultDelay){
        delay.setDelaySamplesFP16(defaultDelay);
        delay.setSlewPerSecondMs(200.0f);
    }

    static uint32_t clampDelayFP16(int64_t value){
        if (value < (int64_t)kMinDelaySamplesFP16) return kMinDelaySamplesFP16;
        if (value > (int64_t)kMaxDelaySamplesFP16) return kMaxDelaySamplesFP16;
        return (uint32_t)value;
    }

    static uint32_t knobToDelayFP16(uint16_t knob){
        return (uint32_t)(((uint64_t)knob * kMaxDelaySamplesFP16) / 4095u);
    }

    static uint32_t averageWithCv(uint32_t knobTarget, int16_t cv){
        int64_t offset = ((int64_t)kDelayRangeSamplesFP16 * (int64_t)cv) / 2048;
        int64_t cvTarget = (int64_t)knobTarget + offset;
        return (uint32_t)(((int64_t)knobTarget + (int64_t)clampDelayFP16(cvTarget)) >> 1);
    }

    SmoothDelay dl1_;
    SmoothDelay dl2_;
    SigmoidSaturator sat;
    SigmoidSaturator sat2;
    uint32_t pulseCountdown1_{0};
    uint32_t pulseCountdown2_{0};
    uint32_t pulseHold1_{0};
    uint32_t pulseHold2_{0};
    bool pulseState1_{false};
    bool pulseState2_{false};
    SlowChaosLFO lfo1_;
    SlowChaosLFO lfo2_;
};


int main()
{
    Vink v;
    v.EnableNormalisationProbe();
    v.Run();
}

  
