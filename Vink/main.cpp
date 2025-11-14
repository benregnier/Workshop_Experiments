#include "ComputerCard.h"
#include <cstdint>
#include <cstring>
#include <algorithm>

/* Vink
        Dual delay loops with sigmoid saturation for Jaap Vink / Roland Kayn style feedback patching
	For more info see this Mr Sonology Video: https://www.youtube.com/watch?v=X_Bcr_HS9XM

	Blocks:
		- Two delay taps (approx 250ms max, down to 3 samples)
        - Sigmoid soft clipper on combined delay output (switchable bypass)

	Control Mapping:
        Knob::Main: shared delay timing control (averaged with CVIn1 when patched)
        Knob::X: Second tap delay - offset from tap 1 (averaged with CVIn2 when patched)
                Knob::Y: saturation drive
        Switch: Up = split inputs/outputs (parallel taps); Center = shared mix
                Momentary Down: toggles tape saturation on/off

    Inputs/Outputs:
		AudioIn1: Audio In
		AudioIn2: Audio In (averaged with AudioIn1 if patched)
		CVIn1: Tap 1 control (when patched averaged with Knob Main)
		CVIn2: Tap 2 control (when patched averaged with Knob X)

		AudioOut1: Tap 1 or shared
		AudioOut2: Tap 2 if split
		CVOut1: Super-slow chaotic LFO 1
        CVOut2: Super-slow chaotic LFO 2
        PulseOut1: pulse tracking delay 1 time
        PulseOut2: pulse tracking delay 2 time
*/

// ---------- helpers ----------
static inline int16_t sat16(int32_t x){ if(x>32767) return 32767; if(x<-32768) return -32768; return (int16_t)x; }
static inline int16_t sat12(int32_t x){ if(x>2047) return 2047; if(x<-2048) return -2048; return (int16_t)x; }
static inline int16_t audio12_to_q15(int16_t x){ return (int16_t)(((int32_t)x) << 4); }
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
    int16_t v = x + 2048;
    if(v < 0) v = 0;
    if(v > 4095) v = 4095;
    return v;
}
static inline int16_t mul_q15(int16_t a, int16_t b){ return (int16_t)(((int32_t)a * (int32_t)b) >> 15); }

// One-pole (state-space) lowpass: y += a*(x - y), a in Q15
struct OnePoleLP {
    int16_t y = 0;         // state in Q15
    uint16_t a = 0;        // coeff Q15 (0..32767), ~cutoff control
    inline int16_t process(int16_t x){
        int16_t diff = (int16_t)(x - y);
        y = (int16_t)(y + ((int32_t)diff * a >> 15));
        return y;
    }
    void clear(){ y = 0; }
};

// Highpass via “x - lowpass(x)”
struct OnePoleHP {
    OnePoleLP lp;
    inline int16_t process(int16_t x){
        int16_t lo = lp.process(x);
        return (int16_t)(x - lo);
    }
    void clear(){ lp.clear(); }
};

// ---------- sigmoid saturation ----------
/**
 * @brief Simple sigmoid-based saturator with adjustable drive.
 */
struct SigmoidSaturator {
    // Drive in Q12 (4096 ≈ 1.0). Range expanded via knob mapping.
    uint16_t driveQ12 = 4096;

    inline int16_t process(int16_t x) const {
        int32_t scaled = ((int32_t)x * (int32_t)driveQ12) >> 12;
        scaled = sat12(scaled);
        return SigSat((int16_t)scaled);
    }

    void setDriveQ12(uint16_t drive){
        if (drive < 256) drive = 256;
        driveQ12 = drive;
    }

    void clear(){}
};


static inline int16_t RingMod(int16_t a, int16_t b)
{
    int32_t prod = (int32_t)a * (int32_t)b;   // range: ±(32768²)
    prod >>= 15;                              // Q15 scaling back to ±32767
    if (prod >  32767) prod =  32767;
    if (prod < -32768) prod = -32768;
    return (int16_t)prod;
}

static inline int16_t SigSat(int16_t x) // Thanks to Allsnop @ the serge discord! x/(1+abs(x))
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


/**
 * @brief Delay line with click-free modulation.
 *
 * Internally stores the delay time in 16.16 fixed-point samples and slews toward
 * a target value to avoid zippering. The buffer size is rounded up to the next
 * power of two so wrapping becomes a bitmask operation.
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
        int32_t  x0   = buffer_[base];
        int32_t  x1   = buffer_[(base - 1) & mask_];

        // y = (x0*(1-f) + x1*f), f in [0..1) as 16-bit fraction
        uint32_t f = delayFrac;
        int32_t y  = ( (x0 * (int32_t)(65536u - f)) + (x1 * (int32_t)f) ) >> 16;
        int16_t out = sat16(y);

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

    // Delay (16.16 samples)
    uint32_t current_fp16_;
    uint32_t target_fp16_;

    // Slew step per sample in 16.16 samples
    uint32_t slew_step_fp16_;
    bool initialised_;
};

/**
 * @brief Implementation of the Vink dual-delay feedback card.
 *
 * The class encapsulates the full signal flow for the instrument described at
 * the top of the file. It keeps the runtime hot path compact by offering small
 * helper utilities that encapsulate repeated tasks (delay computation,
 * saturation toggling, pulse generation, etc.).
 */
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

        lfo1_.configure(0.412345f, 3.9700f, 3.0e-7f, 16384u); //0.412345f, 3.9935f, 2.0e-7f, 32768u
        lfo2_.configure(0.762531f, 3.9500f, 4.0e-7f, 12384u); //0.762531f, 3.9855f, 3.0e-7f, 16384u
    }

    /**
     * @brief Audio callback executed once per sample.
     */
    void ProcessSample() override
    {
        Switch sw = SwitchVal();
        if (SwitchChanged() && sw == Switch::Down) {
            saturationEnabled_ = !saturationEnabled_;
            if (!saturationEnabled_) {
                sat.clear();
                sat2.clear();
            }
        }
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

        uint16_t driveQ12 = knobToDriveQ12(KnobVal(Knob::Y));
        sat.setDriveQ12(driveQ12);
        sat2.setDriveQ12(driveQ12);

        auto applySaturation = [this](int16_t sample, SigmoidSaturator& s){
            return saturationEnabled_ ? s.process(sample) : sample;
        };


        if (splitMode) {
            int16_t out1 = sat12(applySaturation(delay1, sat));
            int16_t out2 = sat12(applySaturation(delay2, sat2));
            AudioOut1(out1);
            AudioOut2(out2);
            LedBrightness(0, led_from_audio12(out1));
            LedBrightness(1, led_from_audio12(out2));
        } else {
            int16_t mix = (int16_t)(((int32_t)delay1 + (int32_t)delay2) >> 1);
            int16_t outMono = sat12(applySaturation(mix, sat));
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

    static uint16_t knobToDriveQ12(uint16_t knob){
        uint32_t drive = 4096u + ((uint32_t)knob * 12288u) / 4095u; // ≈1x .. 4x
        if (drive > 16384u) drive = 16384u;
        return (uint16_t)drive;
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
    bool saturationEnabled_{true};
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

  
