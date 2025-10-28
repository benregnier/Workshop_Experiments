#include "ComputerCard.h"
#include <stdint.h>

// ============ Fixed-point helpers (Q15) ============
static inline int16_t sat16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}
static inline int16_t q15_mul(int16_t a, int16_t b) {
  // (a*b)>>15 with sign, 32-bit intermediate
  return (int16_t)((int32_t)a * (int32_t)b >> 15);
}
// One-pole: y += alpha * (x - y)    (alpha in Q15)
static inline int16_t one_pole_lerp(int16_t x, int16_t y, int16_t alphaQ15) {
  int16_t diff = x - y;
  int16_t step = q15_mul(diff, alphaQ15);
  return (int16_t)(y + step);
}

// ============ Alpha LUTs for 48 kHz (Q15) ============
// alpha = 1 - exp(-2*pi*fc/fs).  Log-spaced 20 Hz .. 5 kHz (32 steps).
// Index 0 = ~20 Hz  …  31 = ~5 kHz
static const uint16_t ALPHA_Q15_32[32] = {
  86,102,122,146,174,208,249,297,355,423,
  505,603,719,858,1022,1218,1450,1725,2050,2435,
  2888,3421,4045,4773,5619,6597,7719,8997,10439,12048,
  13819,15738
};

// Map a 10-bit knob (0..1023) to 0..31 LUT index (log-ish sweep feels good).
static inline uint8_t map_knob_to_idx(uint16_t k) {
  // Simple linear map; you can apply tapering if you like.
  uint32_t idx = ((uint32_t)k * 31) / 1023;
  if (idx > 31) idx = 31;
  return (uint8_t)idx;
}

// ============ ESP state ============
struct ESP {
  // Preamp
  // gainQ8_8: 256 = 1.0x, 512 = 2.0x, etc.
  uint16_t gainQ8_8 = 256 * 4; // default 4x

  // Filters (one-pole HP then LP → band-pass)
  int16_t hp_state = 0;
  int16_t lp_state = 0;
  uint8_t hp_idx = 2;  // ~30–40 Hz
  uint8_t lp_idx = 24; // ~560–660 Hz (nice for pitch tracking on voice/gtr)

  // Envelope follower (rectify → attack/release LP)
  int16_t env = 0;
  uint8_t env_attack_idx = 20;  // ~3–5 ms
  uint8_t env_release_idx = 6;  // ~1–2 ms per sample step (slower alpha ⇒ longer)
  // (Note: lower alpha index = slower; feel free to invert if you prefer)

  // Trigger (Schmitt) in Q15 envelope domain
  uint16_t trig_on_Q15  = 3000;   // ~0.091 (≈ 9% FS), tune per source
  uint16_t trig_off_Q15 = 1500;   // ~0.046
  bool gate = false;

  // Pitch: zero-crossing with hysteresis, based on band-passed signal
  int32_t zc_last_cross_n = 0; // last zero-cross sample index
  int32_t n = 0;               // running sample counter
  int16_t zc_pos_thresh = 1200;  // +/- thresholds mitigate chatter
  int16_t zc_neg_thresh = -1200;
  bool zc_was_pos = false;

  // Outputs
  int16_t env_out = 0;  // Q15 (0..1 → 0..32767)
  int16_t trig_out = 0; // 0 or +32767
  // pitch_hzQ16_16: integer Hz in 16.16 fixed for smoother downstream
  uint32_t pitch_hzQ16_16 = 0;
};

class ESPCard : public ComputerCard {
public:
  ESP esp;

  // ======= Helpers: control-rate updates (NO floats) =======
  void UpdateControls() {
    // Map knobs/switches → state
    // Example: use Knob1 = preamp gain (0.5x .. 32x), Knob2 = HP cutoff, Knob3 = LP cutoff
    // Feel free to remap to your UI.
    uint16_t k1 = Knob1(); // 0..1023
    uint16_t k2 = Knob2();
    uint16_t k3 = Knob3();

    // Preamp gain: 0.5x..32x → Q8.8
    // gain = 0.5 * 2^(0..6) across travel (nice exponential feel via bit-shifts)
    uint8_t steps = (k1 * 6) / 1023;
    uint16_t base = 128; // 0.5 in Q8.8
    uint32_t g = base << steps; // shift = doubling
    if (g > 0xFFFF) g = 0xFFFF;
    esp.gainQ8_8 = (uint16_t)g;

    // Cutoffs
    esp.hp_idx = map_knob_to_idx(k2);
    esp.lp_idx = map_knob_to_idx(k3);
    if (esp.lp_idx <= esp.hp_idx) esp.lp_idx = esp.hp_idx + 1; // keep LP > HP minimally
  }

  // ======= Audio-rate blocks (all integer) =======

  // Preamp with soft clip (tanh-ish via fast cubic) — optional but handy
  static inline int16_t Preamp(int16_t x, uint16_t gainQ8_8) {
    // scale
    int32_t y = ((int32_t)x * (int32_t)gainQ8_8) >> 8; // back to Q0
    // fast soft clip: y - y^3/3 in Q0 (keep small)
    int32_t y3 = (y * y / 32768) * y / 32768; // approx using 16-bit scaling
    y = y - (y3 / 3);
    return sat16(y);
  }

  // One-pole HP via HP = x - LP(x)  (alpha from LUT)
  static inline int16_t HighPass(int16_t x, int16_t &lp_state, uint16_t alphaQ15) {
    int16_t lp = one_pole_lerp(x, lp_state, alphaQ15);
    lp_state = lp;
    int32_t hp = (int32_t)x - lp;
    return sat16(hp);
  }

  // One-pole LP
  static inline int16_t LowPass(int16_t x, int16_t &lp_state, uint16_t alphaQ15) {
    int16_t y = one_pole_lerp(x, lp_state, alphaQ15);
    lp_state = y;
    return y;
  }

  // Full-wave rectify → attack/release envelope
  static inline int16_t EnvelopeFollow(int16_t x_abs, int16_t env,
                                       uint16_t a_on_Q15, uint16_t a_off_Q15) {
    // choose attack vs release alpha
    uint16_t alpha = (x_abs > env) ? a_on_Q15 : a_off_Q15;
    return one_pole_lerp(x_abs, env, alpha);
  }

  // Schmitt trigger on envelope (Q15)
  static inline bool GateFromEnv(int16_t envQ15, bool prev,
                                 uint16_t th_onQ15, uint16_t th_offQ15) {
    if (!prev && (uint16_t)envQ15 >= th_onQ15) return true;
    if (prev  && (uint16_t)envQ15 <= th_offQ15) return false;
    return prev;
  }

  // Zero-crossing pitch (band-passed signal recommended)
  // Returns Hz in Q16.16 (0 if undefined/no crossing yet)
  inline uint32_t PitchZC(int16_t bp) {
    // hysteresis: detect positive-going crossings around ~0
    // First: track last polarity beyond hysteresis
    if (bp > esp.zc_pos_thresh) esp.zc_was_pos = true;
    else if (bp < esp.zc_neg_thresh) esp.zc_was_pos = false;

    uint32_t hzQ16_16 = 0;
    if (!esp.zc_was_pos && bp >= 0) {
      // Rising zero-cross event
      int32_t period = esp.n - esp.zc_last_cross_n;
      esp.zc_last_cross_n = esp.n;
      // Reject implausible periods (for, say, 60–1500 Hz)
      if (period > 32 && period < 800) {
        // fs/period in Q16.16 : (48000<<16)/period
        hzQ16_16 = ((uint32_t)48000 << 16) / (uint32_t)period;
      }
    }
    esp.n++;
    return hzQ16_16; // 0 means "no new estimate this sample"
  }

  // Helper: convert Hz (Q16.16) → 1V/Oct CV in Q16.16
  static inline uint32_t Hz_to_1VOct_Q16(uint32_t hzQ16) {
    // reference A4 = 440 Hz → 4.75 V (pick your scale!)
    // 1V/Oct: volts = log2(hz/440) + 4.75
    // No logs allowed: use a coarse polynomial/log2 LUT at control rate instead.
    // For the audio loop, just return raw Hz; do the scaling in a slower tick.
    return hzQ16; // placeholder: scale it in a control task to DAC codes
  }

  // Helper: convert Hz (Q16.16) → Hz/Volt (Korg-style) in Q16.16
  static inline uint32_t Hz_to_HzPerVolt_Q16(uint32_t hzQ16, uint32_t hzPerVoltQ16) {
    // volts = hz / (HzPerVolt). E.g., if 100 Hz per Volt, set hzPerVoltQ16 = 100<<16
    return (hzQ16 << 16) / hzPerVoltQ16; // returns Volts in Q16.16
  }

  // ======= ComputerCard hooks =======
  virtual void Setup() {
    // Anything you want on boot
    UpdateControls();
  }

  // Consider calling UpdateControls() at ~100–400 Hz (timer/ISR or ProcessControl)
  virtual void ProcessControl() {
    UpdateControls();
  }

  virtual void ProcessSample() {
    // 1) Read
    int16_t in = AudioIn1();  // mono input

    // 2) Preamp
    int16_t pre = Preamp(in, esp.gainQ8_8);

    // 3) Band-pass: HP then LP
    uint16_t hp_alpha = ALPHA_Q15_32[esp.hp_idx];
    int16_t hp = HighPass(pre, esp.hp_state, hp_alpha);

    uint16_t lp_alpha = ALPHA_Q15_32[esp.lp_idx];
    int16_t bp = LowPass(hp, esp.lp_state, lp_alpha);

    // 4) Envelope follower (rectify + dual time constant)
    int16_t x_abs = (bp >= 0) ? bp : (int16_t)(-bp);
    uint16_t a_on  = ALPHA_Q15_32[esp.env_attack_idx];
    uint16_t a_off = ALPHA_Q15_32[esp.env_release_idx];
    esp.env = EnvelopeFollow(x_abs, esp.env, a_on, a_off);
    esp.env_out = esp.env; // Q15 -> route to CV if desired

    // 5) Trigger with hysteresis
    esp.gate = GateFromEnv(esp.env_out, esp.gate, esp.trig_on_Q15, esp.trig_off_Q15);
    esp.trig_out = esp.gate ? 32767 : 0;

    // 6) Pitch estimate (zero-cross; new value only on crossings)
    uint32_t hzQ16 = PitchZC(bp);
    if (hzQ16) esp.pitch_hzQ16_16 = hzQ16;

    // ===== Outputs =====
    // Audio “monitor” path (optional: hear filtered input through your synth path)
    AudioOut1(bp);   // or pre / in for raw/filtered

    // CV outs (Workshop Computer: adapt to your DAC scaling)
    // Example mappings (pseudo):
    //   CVOut1(esp.env_out);         // envelope (0..+1)
    //   PulseOut1(esp.gate);         // trigger/gate
    //   CVOut2(scale_to_DAC(esp.pitch_hzQ16_16)); // pitch (convert at control rate)
  }
};
