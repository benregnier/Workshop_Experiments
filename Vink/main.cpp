#include "ComputerCard.h"

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
		CVIn1: Main Knob control
		CVIn2: Knob X control
		Switch: Select split (Up) or shared (mom/down) output
		AudioOut1: Tap 1 or shared
		AudioOut2: Tap 2 if split
		(Optional) CVOut1 envelope follower signal from limiter?
		
*/

// Helper Functions
#include <cstdint>
#include <cstring>
#include <algorithm>

static inline int16_t sat16(int32_t x){
    if (x >  32767) return  32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

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

// Vink State
struct VinkState {
};

class Vink : public ComputerCard
{
public:
	VinkState vink;

	Vink(){
	};

	virtual void ProcessSample()
	{

		// Todo: Implement Delay lines
		int16_t in  = AudioIn1();
		int16_t out = dl.process(in);  // delayed only (you mix elsewhere)
		AudioOut1(out);

		// Modulate without clicks (two common options):
		// A) Block-by-block sweep (slew handles smoothing)
		// dl.setDelayMs(newDelayMs);
	
		// B) Per-sample LFO in samples with 16.16 precision (still smoothed)
		// uint32_t lfo_fp16 = /* your computed (samples<<16)+frac */;
		// dl.setDelaySamplesFP16(lfo_fp16);
		
		// Todo: Implement ring modulation
		// Todo: Implement limiter
		
		// Transfer audio/CV/Pulse inputs directly to outputs

		CVOut1(CVIn1());
		CVOut2(CVIn2());

		PulseOut1(PulseIn1());
		PulseOut2(PulseIn2());

		// Get switch position and set LEDs 0, 2, 4 accordingly
		int s = SwitchVal();
		LedOn(4, s == Switch::Down);
		LedOn(2, s == Switch::Middle);
		LedOn(0, s == Switch::Up);

		// Set LED 1, 3, 5 brightness to knob values
		LedBrightness(1, KnobVal(Knob::Main));
		LedBrightness(3, KnobVal(Knob::X));
		LedBrightness(5, KnobVal(Knob::Y));
	}
};


int main()
{
	// Init delays
	SmoothDelay dl(48000, 1000);   // up to 1000 ms
	dl.setDelayMs(300);            // start at 300 ms
	dl.setSlewPerSecondMs(200.0f); // optional: ~0.2 ms change per ms of audio
	Vink v;
	v.Run();
}

  
