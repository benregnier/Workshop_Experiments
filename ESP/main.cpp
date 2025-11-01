// main.cpp
#include "ComputerCard.h"
#include <stdint.h>
#include <limits.h>

/*
  Workshop Computer — MS-20-style External Signal Processor with 1 V/oct CV

  Blocks:
    - Preamp (Q8.8 gain, integer soft clip)
    - HP + LP → Band-pass (one-pole each; alphas from LUT for 48 kHz)
    - Envelope follower (rectify + separate attack/release)
    - Gate (Schmitt) from envelope
    - Pitch estimator (zero-crossing w/ hysteresis)
    - 1 V/oct fixed-point log2 → millivolts → CVOutMillivolts(0, mV)

  Control mapping (adjust in UpdateControls()):
    Knob1: Preamp gain (0.5x .. 32x, exponential)
    Knob2: HP cutoff (index into alpha LUT, ~20 Hz .. ~5 kHz)
    Knob3: LP cutoff (index into alpha LUT, forced >= HP+1)

    Switch: When set to middle, updates only when gate is high, otherwise continuous

  Input and Output Mapping:
    AudioOut1: Bandpassed audio
    CVOut1 : 1 V/oct pitch
    CVOut2 : envelope
    Pulse1 : gate
  
*/

// ================= Fixed-point helpers =================
static inline int16_t sat16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}
static inline int16_t abs16(int16_t x) {
    int16_t mask = x >> 15;        // 0x0000 if x >= 0, 0xFFFF if x < 0
    return (x + mask) ^ mask;      // if negative → (~x + 1), else → x
}
static inline int16_t q15_mul(int16_t a, int16_t b) { // (a*b)>>15
  return (int16_t)((int32_t)a * (int32_t)b >> 15);
}
static inline int16_t one_pole_lerp(int16_t x, int16_t y, uint16_t alphaQ15) {
  int16_t diff = (int16_t)(x - y);
  int16_t step = q15_mul(diff, (int16_t)alphaQ15);
  return (int16_t)(y + step);
}
static inline uint8_t clampu8(uint32_t v, uint32_t hi) {
  return (uint8_t)(v > hi ? hi : v);
}

// ============ Alpha LUTs for 48 kHz (Q15) ============
// alpha = 1 - exp(-2*pi*fc/fs). Log-spaced ~20 Hz → ~5 kHz (32 steps).
static const uint16_t ALPHA_Q15_32[32] = {
  86,102,122,146,174,208,249,297,355,423,
  505,603,719,858,1022,1218,1450,1725,2050,2435,
  2888,3421,4045,4773,5619,6597,7719,8997,10439,12048,
  13819,15738
};
static inline uint8_t map_knob_to_idx(uint16_t k) {
  uint32_t idx = ((uint32_t)k * 31) / 4095;
  return clampu8(idx, 31);
}

// ================= 1 V/oct fixed-point (Hz Q16.16 → mV int) =================
#define ONEVOCT_REF_HZ        440
#define ONEVOCT_REF_VOLTS_Q16 ((4<<16) + 0xC000)  // 4.75 V in Q16.16
#define CV_FULL_SCALE_MV      8000                // rail for clamping

static inline uint32_t q16_from_u32(uint32_t x){ return x<<16; }
static inline int32_t  q16_mul32(int32_t a,int32_t b){ return (int32_t)(((int64_t)a*(int64_t)b)>>16); }
static inline int32_t  q30_mul(int32_t a,int32_t b){ return (int32_t)(((int64_t)a*(int64_t)b)>>30); }
static inline int32_t  clamp32(int32_t x,int32_t lo,int32_t hi){ return x<lo?lo:(x>hi?hi:x); }

static inline int32_t floor_log2_u32(uint32_t x){ int32_t n=-1; while(x){ x>>=1; n++; } return n; }

// log2(x_Q16) → Q16.16 using cubic approx on [1,2)
static inline int32_t log2_q16(uint32_t xQ16){
  if(!xQ16) return INT32_MIN/2;
  int32_t exp = floor_log2_u32(xQ16);
  int32_t n = exp - 16;                       // integer part of log2(x_Q16/2^16)
  uint32_t yQ16;
  if (n >= 0) yQ16 = xQ16 >> n;               // keep Q16.16 mantissa in [1,2)
  else        yQ16 = xQ16 << (-n);
  int32_t fQ16  = (int32_t)yQ16 - (1<<16);    // f in [0,1)
  const int32_t a1=1528445166, a2=-631032126, a3=177730538; // Q2.30 coeffs
  int32_t fQ30  = fQ16 << 14;
  int32_t f2Q30 = q30_mul(fQ30, fQ30);
  int32_t f3Q30 = q30_mul(f2Q30, fQ30);
  int32_t polyQ30 = q30_mul(a1,fQ30) + q30_mul(a2,f2Q30) + q30_mul(a3,f3Q30);
  int32_t polyQ16 = polyQ30 >> 14;
  return (n<<16) + polyQ16;
}

struct OneVOct {
  int32_t log2_refQ16;
};
static inline void OneVOct_Init(OneVOct &s){
  s.log2_refQ16 = log2_q16(q16_from_u32(ONEVOCT_REF_HZ));
}
static inline int32_t OneVOct_VoltsQ16(const OneVOct &s, uint32_t hzQ16){
  if(!hzQ16) return INT32_MIN/2;
  int32_t lg = log2_q16(hzQ16);
  return (lg - s.log2_refQ16) + (int32_t)ONEVOCT_REF_VOLTS_Q16;
}
static inline int32_t VoltsQ16_to_mV(int32_t voltsQ16){
  if (voltsQ16 < 0) return 0;
  int32_t mv = (int32_t)(((int64_t)voltsQ16 * 1000) >> 16);
  return clamp32(mv, 0, CV_FULL_SCALE_MV);
}

// ================= ESP state =================
struct ESPState {
  // Preamp gain (Q8.8). 256=1.0x
  uint16_t gainQ8_8 = 256 * 4; // default 4x

  // Filters (HP via x-LP, then LP)
  int16_t hp_state = 0;
  int16_t lp_state = 0;
  uint8_t hp_idx = 2;    // ~30–40 Hz
  uint8_t lp_idx = 24;   // ~560–660 Hz

  // Envelope follower
  int16_t env = 0;
  uint8_t env_attack_idx = 20; // faster
  uint8_t env_release_idx = 6; // slower
  uint16_t trig_on_Q15  = 1500;  // ~9% FS
  uint16_t trig_off_Q15 = 500;  // ~4.6% FS
  bool gate = false;

  // Pitch (zero-crossing with hysteresis)
  int32_t n = 0;
  int32_t zc_last_cross_n = 0;
  int16_t zc_pos_thresh = 2000;
  int16_t zc_neg_thresh = -2000;
  bool zc_was_pos = false;

  // Outputs
  int16_t env_out = 0;           // Q15 envelope
  int16_t trig_out = 0;          // 0 or 32767
  uint32_t pitch_hzQ16_16 = 0;   // Hz in Q16.16 (updated on new ZC)
  int32_t pitch_mv = 0;          // cached millivolts for CVOut1
  uint16_t pitch_led = 0;        // brightness for LED 2
};

// ================= ESP processing =================
class ESPCard : public ComputerCard {
public:
  ESPState esp;
  OneVOct  onev;
  uint32_t sm_hzQ16 = 0; // control-rate smoothed Hz (Q16.16)
  uint32_t control_counter = 0;

  static constexpr uint32_t kControlIntervalSamples = 240; // ~200 Hz

  ESPCard() {
    OneVOct_Init(onev);
    UpdateControls();
  }

  // ----- Control mapping -----
  void UpdateControls() {
    uint16_t k1 = (uint16_t)KnobVal(Knob::Main); // 0..4095
    uint16_t k2 = (uint16_t)KnobVal(Knob::X);
    uint16_t k3 = (uint16_t)KnobVal(Knob::Y);

    // Preamp gain: 0.5x .. 32x using bit-doublings from 0.5 base
    uint8_t steps = (uint8_t)(((uint32_t)k1 * 6) / 4095); // 0..6 doublings
    uint16_t base = 128; // 0.5 in Q8.8
    uint32_t g = ((uint32_t)base) << steps;
    if (g > 0xFFFF) g = 0xFFFF;
    esp.gainQ8_8 = (uint16_t)g;

    // HP/LP cutoffs
    esp.hp_idx = map_knob_to_idx(k2);
    esp.lp_idx = map_knob_to_idx(k3);
    if (esp.lp_idx <= esp.hp_idx) esp.lp_idx = (uint8_t)(esp.hp_idx + 1);
  }

  // ----- Building blocks (integer only) -----
  static inline int16_t Preamp(int16_t x, uint16_t gainQ8_8) {
    // scale Q8.8 -> Q0
    int32_t y = ((int32_t)x * (int32_t)gainQ8_8) >> 8;
    // soft clip approx: y - y^3/3 (scaled to int16 range)
    // use 16-bit friendly cubic
    int32_t y32 = y;
    int32_t y2 = (y32 * y32) >> 15;     // roughly keep magnitude
    int32_t y3 = (y2 * y32)  >> 15;
    y = y32 - (y3 / 3);
    return sat16(y);
  }

  static inline int16_t HighPass(int16_t x, int16_t &lp_state, uint16_t alphaQ15) {
    int16_t lp = one_pole_lerp(x, lp_state, alphaQ15);
    lp_state = lp;
    int32_t hp = (int32_t)x - lp;
    return sat16(hp);
  }

  static inline int16_t LowPass(int16_t x, int16_t &lp_state, uint16_t alphaQ15) {
    int16_t y = one_pole_lerp(x, lp_state, alphaQ15);
    lp_state = y;
    return y;
  }

  static inline int16_t EnvelopeFollow(int16_t x_abs, int16_t env, uint16_t a_on_Q15, uint16_t a_off_Q15) {
    uint16_t a = (x_abs > env) ? a_on_Q15 : a_off_Q15;
    return one_pole_lerp(x_abs, env, a);
  }

  inline uint32_t PitchZC(int16_t bp) {
    // hysteresis polarity tracking
    if (bp > esp.zc_pos_thresh) esp.zc_was_pos = true;
    else if (bp < esp.zc_neg_thresh) esp.zc_was_pos = false;

    uint32_t hzQ16 = 0;
    if (!esp.zc_was_pos && bp >= 0) {
      int32_t period = esp.n - esp.zc_last_cross_n;
      esp.zc_last_cross_n = esp.n;
      // plausible period window: ~60..1500 Hz at 48k → 32..800 samples
      if (period > 16 && period < 800) {
        hzQ16 = ((uint32_t)48000 << 16) / (uint32_t)period;
      }
    }
    esp.n++;
    return hzQ16;
  }

  void RunControlFrame() {
    UpdateControls();

    // Smooth the Hz estimate a bit (prevents warble)
    uint32_t inQ16 = esp.pitch_hzQ16_16;
    if (inQ16) {
      const int32_t a = 13107; // ~0.2 in Q16.16
      sm_hzQ16 = (uint32_t)(((int64_t)a * inQ16 + (int64_t)(65536 - a) * sm_hzQ16) >> 16);
    }

    if (sm_hzQ16) {
      int32_t vQ16 = OneVOct_VoltsQ16(onev, sm_hzQ16); // Hz -> volts (Q16.16)
      int32_t mv   = VoltsQ16_to_mV(vQ16);             // volts -> millivolts (int)
      esp.pitch_mv = mv;
      esp.pitch_led = (uint16_t)(((uint32_t)mv * 4095) / CV_FULL_SCALE_MV);
    }
    else {
      esp.pitch_mv = 0;
      esp.pitch_led = 0;
    }
  }

  void ProcessSample() override {
    if (++control_counter >= kControlIntervalSamples) {
      control_counter = 0;
      RunControlFrame();
    }

    // Read mono input
    int16_t in = AudioIn1();

    // Preamp
    int16_t pre = Preamp(in, esp.gainQ8_8);

    // Band-pass
    uint16_t hp_alpha = ALPHA_Q15_32[esp.hp_idx];
    int16_t hp = HighPass(pre, esp.hp_state, hp_alpha);

    uint16_t lp_alpha = ALPHA_Q15_32[esp.lp_idx];
    int16_t bp = LowPass(hp, esp.lp_state, lp_alpha);

    // Envelope follower
    int16_t x_abs = (bp >= 0) ? bp : (int16_t)(-bp);
    uint16_t a_on  = ALPHA_Q15_32[esp.env_attack_idx];
    uint16_t a_off = ALPHA_Q15_32[esp.env_release_idx];
    esp.env = EnvelopeFollow(x_abs, esp.env, a_on, a_off);
    esp.env_out = esp.env;

    // Gate (Schmitt) from envelope
    if (!esp.gate && (uint16_t)esp.env_out > esp.trig_on_Q15) esp.gate = true;
    else if (esp.gate && (uint16_t)esp.env_out <= esp.trig_off_Q15) esp.gate = false;
    // esp.trig_out = esp.gate ? 32767 : 0;
    PulseOut1(esp.gate);
    LedOn(4, esp.gate);

    // Pitch estimate (update on crossings)
    uint32_t hzQ16 = PitchZC(bp);
    if ((SwitchVal() != Switch::Middle) || esp.gate) {
      if (hzQ16) esp.pitch_hzQ16_16 = hzQ16;
    };
    

    // Pitch CV outs
    (void)CVOutMillivolts(0, esp.pitch_mv);
    LedBrightness(2, esp.pitch_led);

    // Envelope Out
    //uint16_t envQ15 = (uint16_t)esp.env_out;
    //int32_t env_mv = ((int32_t)envQ15 * CV_FULL_SCALE_MV) >> 15;
    //(void)CVOutMillivolts(1, env_mv);
    //uint16_t env_led = (uint16_t)(((uint32_t)env_mv * 4095) / CV_FULL_SCALE_MV);
    if (esp.env_out > 2047) esp.env_out = 2047;
    if (esp.env_out < 0 ) esp.env_out = 0;
    CVOut2(esp.env_out);
    int16_t env_led = esp.env_out * 2;
    LedBrightness(3, env_led);

    // Monitor: send band-passed audio out (or choose pre/in)
    if (bp > 2047) bp = 2047;
    else if (bp < -2048) bp = -2048;
    AudioOut1(bp);
    LedBrightness(0, abs16(bp)*2);
  }
};

int main()
{
  ESPCard esp;
  esp.Run();
  esp.EnableNormalisationProbe();
}
