#include "ComputerCard.h"
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>

/* Vink
	Dual delay loops + ring mod with a limiter for Jaap Vink / Roland Kayn style feedback patching 
	For more info see this Mr Sonology Video: https://www.youtube.com/watch?v=X_Bcr_HS9XM

	Blocks:
		- Two delay taps (x to y ms)
		- Ring modulators for each loop sharing a single input
		- Compressor/limiter end of chain

	Control Mapping:
		Knob::Main: shared delay timing control (becomes attenuator for CVIn1 when patched)
		Knob::X: delay timing spread (becomes attenuator for CVIn2 when patched)
		Knob::Y: limiter volume

		AudioIn1: Audio In
		AudioIn2: Ring Modulator In (disconnects when not patched)
		(Future) CVIn1: Main Knob control
		(Future) CVIn2: Knob X control
		(Future) Switch: Select split (Up) or shared (mom/down) output
		AudioOut1: Tap 1 or shared
		(Future) AudioOut2: Tap 2 if split
		(Optional) CVOut1 envelope follower signal from limiter?
		
*/

// Helper Functions



static inline int16_t RingMod(int16_t a, int16_t b)
{
    // Multiply 16-bit signed values and scale back to 16-bit range.
    // Using 32-bit intermediate avoids overflow.
    int32_t prod = (int32_t)a * (int32_t)b;   // range: ±(32768²)
    prod >>= 15;                              // Q15 scaling back to ±32767
    if (prod >  32767) prod =  32767;
    if (prod < -32768) prod = -32768;
    return (int16_t)prod;
}

static inline int16_t sat16(int32_t x){
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

// One-pole envelope + fixed-ratio gain computer (Q15 domain)
class FixedRatioLimiter {
public:
    // sampleRate: e.g., 48000
    // attackMs/releaseMs: envelope times
    // thresholdQ15: 0..32767 (e.g., 26214 ≈ 0.8 FS)
    // ratio: 1=no compression, 4=4:1, 1000≈ limiter
    FixedRatioLimiter(uint32_t sampleRate,
                      float attackMs, float releaseMs,
                      uint16_t thresholdQ15, uint16_t ratio)
    : sr_(sampleRate), thr_(thresholdQ15), ratio_(ratio ? ratio : 1)
    {
        setTimes(attackMs, releaseMs);
    }

    // Change times later (computes Q15 coeffs once; floats not used in process())
    void setTimes(float attackMs, float releaseMs){
        atk_q15_ = timeToCoeffQ15(attackMs);
        rel_q15_ = timeToCoeffQ15(releaseMs);
    }

    void setThresholdQ15(uint16_t thr){ thr_ = thr; }
    void setRatio(uint16_t r){ ratio_ = (r ? r : 1); }

    // Process one sample (int16_t Q15 audio)
    inline int16_t process(int16_t x){
        // 1) Peak envelope with attack/release smoothing (Q15 math)
        uint16_t mag = (uint16_t)(x >= 0 ? x : (x == -32768 ? 32767 : -x)); // |x| in 0..32767
        uint32_t diff;
        if (mag > env_) {
            diff = (uint32_t)mag - env_;
            env_ += (uint32_t)((diff * atk_q15_) >> 15);
        } else {
            diff = (uint32_t)env_ - mag;
            env_ -= (uint32_t)((diff * rel_q15_) >> 15);
        }

        // 2) Compute gain (Q15). If below threshold => unity
        uint16_t gain_q15 = 32767;
        if (env_ > thr_) {
            // a = thr/env in Q15: ((thr<<15)/env)
            uint16_t a = (uint16_t)(((uint32_t)thr_ << 15) / env_);
            // Fixed-ratio law: out = thr + (env - thr)/ratio  => gain = out/env
            // => gain = a + (1/ratio)*(1 - a)
            uint16_t one_minus_a = (uint16_t)(32767 - a);
            uint16_t term = (uint16_t)(one_minus_a / ratio_); // integer, cheap
            gain_q15 = (uint16_t)(a + term);
        }

        // 3) Apply gain
        int32_t y = ((int32_t)x * (int32_t)gain_q15) >> 15;
        return sat16(y);
    }     
    // Optional: clear envelope
    void reset(){ env_ = 0; }

private:
    // Convert time constant to per-sample smoothing coeff in Q15:
    // coeff = 1 - exp(-1/(tau * sr))  (performed once, out of the DSP loop)
    uint16_t timeToCoeffQ15(float ms){
        if (ms <= 0.0f) return 32767; // immediate
        float tau = ms / 1000.0f;
        float c = 1.0f - std::exp(-1.0f / (tau * (float)sr_));
        if (c < 0.0f) c = 0.0f; 
        if (c > 0.9999695f) c = 0.9999695f; // clamp to <1
        return (uint16_t)(c * 32767.0f + 0.5f);
    }

    uint32_t sr_;
    uint16_t thr_{26214};    // ~0.8 FS by default
    uint16_t ratio_{1000};   // ≈ limiter by default
    uint16_t atk_q15_{1638}; // ~50 ms at 48 kHz default-ish
    uint16_t rel_q15_{328};  // ~250 ms default-ish
    uint16_t env_{0};        // envelope in Q15 (0..32767)
};

class SmoothDelay {
public:
    // maxDelayMs: upper bound (e.g., 1000 = 1s)
    explicit SmoothDelay(uint32_t sampleRate, uint32_t maxDelayMs)
    : sr_(sampleRate)
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

private:
    uint32_t sr_;
    uint32_t bufSize_{0}, mask_{0};
    int16_t* buffer_{nullptr};
    uint32_t w_{0};

    // Delay (16.16 samples)
    uint32_t current_fp16_{0};
    uint32_t target_fp16_{0};

    // Slew step per sample in 16.16 samples
    uint32_t slew_step_fp16_{0};
};

class Vink : public ComputerCard
{
public:
    Vink()
        : dl1_(48000, 1000),
          dl2_(48000, 1000),
          lim_(48000, 1.0f, 100.0f, 29491, 1000)
    {
        dl1_.setDelayMs(300);
        dl1_.setSlewPerSecondMs(200.0f);
        dl2_.setDelayMs(500);
        dl2_.setSlewPerSecondMs(200.0f);
    }

    void ProcessSample() override
    {
        // 2 delay lines
        int16_t in1 = AudioIn1();
        int16_t in2 = AudioIn2();
        int16_t delay1 = dl1_.process(in1);  // delayed only (you mix elsewhere)
        int16_t delay2 = dl2_.process(in1);

        // Modulate without clicks (two common options):
        // A) Block-by-block sweep (slew handles smoothing)
        uint16_t newDelayMs1 = (KnobVal(Knob::X) * 1000u) / 4095u;
        dl1_.setDelayMs(newDelayMs1);
        uint16_t newDelayMs2 = (KnobVal(Knob::Y) * 1000u) / 4095u;
        dl2_.setDelayMs(newDelayMs2);

        // B) Per-sample LFO in samples with 16.16 precision (still smoothed)
        // uint32_t lfo_fp16 = /* your computed (samples<<16)+frac */;
        // dl.setDelaySamplesFP16(lfo_fp16);

        // Ring Modulation
        int16_t out1 = delay1;
        int16_t out2 = delay2;
        if (Connected(Input::Audio2)) {
            out1 = RingMod(delay1, in2);
            out2 = RingMod(delay2, in2);
        }

        // Mix delays and output
        int16_t out = (int16_t)(((int32_t)out1 + (int32_t)out2) >> 1);

        // Limit
        uint16_t thrQ15 = (KnobVal(Knob::Main) * 32767u) / 4095u;
        lim_.setThresholdQ15(thrQ15);
        int16_t outlim = lim_.process(out);

        // Output
        AudioOut1(outlim);
        LedBrightness(0, outlim+2047);
    }

private:
    SmoothDelay dl1_;
    SmoothDelay dl2_;
    FixedRatioLimiter lim_;
};


int main()
{
    Vink v;
    v.EnableNormalisationProbe();
    v.Run();
}

  
