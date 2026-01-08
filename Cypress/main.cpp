#include "ComputerCard.h"
#include <cstdint>
#include <limits>

// Cypress: Pulse train / FOF synthesis
//
// - Main knob controls pulse frequency (summed with AudioIn2 1V/oct)
// - X knob + CV1 control pulse width (CV1 patched: knob = attenuator)
// - Y knob + CV2 control pulse tilt (rise/fall ratio)
// - Switch up: constant-time width; switch middle: constant duty cycle
// - AudioIn1 provides excitation for FOF; if unpatched, a static value is used

namespace {
constexpr int32_t kSampleRate = 48000;
constexpr int32_t kEnvMax = 32768; // Q15
constexpr int16_t kStaticSource = 1800;
constexpr int32_t kRiseMinSamples = 4;
constexpr int32_t kFallMinSamples = 4;
constexpr int32_t kWidthMinSamples = kRiseMinSamples + kFallMinSamples;
constexpr int32_t kWidthMaxSamples = 6000; // ~125 ms
constexpr int32_t kBaseFreqHz = 20;       // 0 V base frequency
constexpr int32_t kMaxOctaveShift = 8;
constexpr int32_t kDutyMinQ15 = 1638;   // ~5%
constexpr int32_t kDutyMaxQ15 = 31130;  // ~95%

static inline int16_t sat12(int32_t x) {
    if (x > 2047) return 2047;
    if (x < -2048) return -2048;
    return static_cast<int16_t>(x);
}

constexpr uint32_t kBasePhaseInc = static_cast<uint32_t>((static_cast<uint64_t>(kBaseFreqHz) << 32) / kSampleRate);

static constexpr uint32_t kExp2Table[] = {
    65536, 65714, 65893, 66073, 66253, 66435, 66617, 66800, 66984, 67168, 67354, 67540, 67727, 67914, 68103, 68292,
    68482, 68673, 68865, 69058, 69252, 69446, 69641, 69838, 70034, 70232, 70431, 70630, 70831, 71032, 71234, 71437,
    71641, 71846, 72051, 72258, 72465, 72674, 72883, 73093, 73303, 73515, 73728, 73941, 74155, 74370, 74586, 74803,
    75020, 75239, 75458, 75678, 75899, 76121, 76344, 76568, 76792, 77018, 77244, 77471, 77700, 77928, 78159, 78389,
    78621, 78854, 79088, 79322, 79558, 79794, 80031, 80269, 80508, 80748, 80989, 81230, 81473, 81716, 81960, 82205,
    82451, 82698, 82945, 83194, 83443, 83693, 83944, 84196, 84449, 84703, 84957, 85213, 85469, 85726, 85984, 86243,
    86502, 86763, 87024, 87286, 87549, 87813, 88078, 88343, 88610, 88877, 89145, 89414, 89684, 89955, 90227, 90499,
    90773, 91047, 91322, 91598, 91875, 92153, 92431, 92711, 92991, 93273, 93555, 93838, 94121, 94406, 94692, 94979,
    95266, 95555, 95844, 96134, 96425, 96717, 97010, 97303, 97598, 97893, 98190, 98487, 98785, 99084, 99384, 99685,
    99987, 100290, 100594, 100899, 101205, 101512, 101820, 102129, 102439, 102750, 103062, 103375, 103689, 104004,
    104320, 104637, 104955, 105274, 105594, 105915, 106237, 106560, 106884, 107209, 107535, 107862, 108190, 108519,
    108849, 109181, 109513, 109846, 110181, 110516, 110853, 111190, 111529, 111869, 112210, 112552, 112895, 113239,
    113584, 113931, 114278, 114627, 114976, 115327, 115679, 116032, 116387, 116742, 117099, 117457, 117816, 118176,
    118538, 118900, 119264, 119629, 119995, 120363, 120731, 121101, 121472, 121845, 122218, 122593, 122969, 123347,
    123725, 124105, 124486, 124869, 125252, 125637, 126024, 126411, 126800, 127190, 127582, 127975, 128369, 128764,
    129161, 129559, 129959, 130359, 130762, 131165, 131570, 131976, 132384, 132793, 133204, 133616, 134029, 134444,
    134861, 135279, 135698, 136119, 136541, 136965, 137391, 137818, 138246, 138676, 139108, 139541, 139976, 140412,
    140850, 141290, 141731, 142173, 142618, 143064, 143511, 143960, 144411, 144864, 145318, 145774, 146231, 146690,
    147151, 147614, 148078, 148544, 149012, 149481, 149952, 150425, 150900, 151376, 151854, 152334, 152816, 153300,
    153785, 154273, 154762, 155253, 155746, 156241, 156737, 157236, 157736, 158239, 158743, 159249, 159758, 160268,
    160780, 161294, 161811, 162329, 162849, 163372, 163896, 164423, 164951, 165482, 166015, 166550, 167087, 167626,
    168168, 168711, 169257, 169805, 170355, 170908, 171462, 172019, 172578, 173139, 173703, 174269, 174837, 175407,
    175980, 176555, 177133, 177713, 178295, 178880, 179467, 180057, 180649, 181243, 181840, 182439, 183041, 183645,
    184252, 184861, 185473, 186087, 186704, 187323, 187945, 188569, 189196, 189826, 190458, 191093, 191731, 192371,
    193014, 193659, 194307, 194958, 195612, 196268, 196928, 197590, 198255, 198923, 199593, 200267, 200944, 201623
};
} // namespace

class CypressCard : public ComputerCard {
public:
    void ProcessSample() override {
        updateControls();

        uint32_t nextPhase = phase_ + phaseInc_;
        bool newPulse = nextPhase < phase_;
        phase_ = nextPhase;

        if (newPulse) {
            env_ = 0;
            stage_ = Stage::Rise;
        }

        if (stage_ == Stage::Rise) {
            env_ += riseStep_;
            if (env_ >= kEnvMax) {
                env_ = kEnvMax;
                stage_ = Stage::Fall;
            }
        } else if (stage_ == Stage::Fall) {
            env_ -= fallStep_;
            if (env_ <= 0) {
                env_ = 0;
                stage_ = Stage::Idle;
            }
        }

        int16_t source = inputConnected_ ? AudioIn1() : kStaticSource;
        int32_t shaped = static_cast<int32_t>((static_cast<int64_t>(source) * env_) >> 15);
        int16_t out = sat12(shaped);

        AudioOut1(out);
        AudioOut2(out);

        LedBrightness(0, KnobVal(Knob::Main));
        LedBrightness(2, widthControl_);
        LedBrightness(4, tiltControl_);
    }

private:
    enum class Stage { Idle, Rise, Fall };

    uint32_t phase_ = 0;
    uint32_t phaseInc_ = kBasePhaseInc;
    int32_t env_ = 0;
    int32_t riseStep_ = kEnvMax;
    int32_t fallStep_ = kEnvMax;
    Stage stage_ = Stage::Idle;

    bool inputConnected_ = false;
    uint16_t widthControl_ = 0;
    uint16_t tiltControl_ = 0;

    void updateControls() {
        inputConnected_ = Connected(Input::Audio1);

        uint16_t knobWidth = KnobVal(Knob::X);
        uint16_t knobTilt = KnobVal(Knob::Y);

        widthControl_ = Connected(Input::CV1)
            ? static_cast<uint16_t>((static_cast<uint32_t>(knobWidth) * static_cast<uint32_t>(CVIn1() + 2048)) >> 12)
            : knobWidth;

        tiltControl_ = Connected(Input::CV2)
            ? static_cast<uint16_t>((static_cast<uint32_t>(knobTilt) * static_cast<uint32_t>(CVIn2() + 2048)) >> 12)
            : knobTilt;

        phaseInc_ = computePhaseInc();
        int32_t widthSamples = computeWidthSamples();
        int32_t riseMaxSamples = widthSamples - kFallMinSamples;
        if (riseMaxSamples < kRiseMinSamples) {
            riseMaxSamples = kRiseMinSamples;
        }

        int32_t riseSamples = mapControl(tiltControl_, kRiseMinSamples, riseMaxSamples);
        int32_t fallSamples = widthSamples - riseSamples;
        if (fallSamples < kFallMinSamples) {
            fallSamples = kFallMinSamples;
            riseSamples = widthSamples - fallSamples;
        }

        riseStep_ = (riseSamples > 0) ? (kEnvMax + riseSamples - 1) / riseSamples : kEnvMax;
        fallStep_ = (fallSamples > 0) ? (kEnvMax + fallSamples - 1) / fallSamples : kEnvMax;
    }

    static int32_t mapControl(uint16_t control, int32_t minVal, int32_t maxVal) {
        int32_t span = maxVal - minVal;
        return minVal + static_cast<int32_t>((static_cast<int64_t>(span) * control + 2047) / 4095);
    }

    int32_t computeWidthSamples() {
        Switch sw = SwitchVal();
        bool dutyCycleMode = (sw == Switch::Middle);
        int32_t widthSamples = kWidthMinSamples;

        if (dutyCycleMode) {
            uint64_t periodSamples = (phaseInc_ > 0) ? ((1ull << 32) / phaseInc_) : 0;
            int32_t dutyQ15 = kDutyMinQ15
                + static_cast<int32_t>((static_cast<int64_t>(kDutyMaxQ15 - kDutyMinQ15) * widthControl_ + 2047) / 4095);
            uint64_t computedWidth = (periodSamples * static_cast<uint32_t>(dutyQ15)) >> 15;
            if (computedWidth > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) {
                computedWidth = static_cast<uint64_t>(std::numeric_limits<int32_t>::max());
            }
            widthSamples = static_cast<int32_t>(computedWidth);
        } else {
            widthSamples = mapControl(widthControl_, kWidthMinSamples, kWidthMaxSamples);
        }

        if (widthSamples < kWidthMinSamples) {
            widthSamples = kWidthMinSamples;
        }
        return widthSamples;
    }

    uint32_t computePhaseInc() {
        int32_t knobCv = static_cast<int32_t>((static_cast<uint32_t>(KnobVal(Knob::Main)) * 2048u + 2047u) / 4095u);
        int32_t audioCv = Connected(Input::Audio2) ? static_cast<int32_t>(AudioIn2()) : 0;
        int32_t pitchCv = knobCv + audioCv;

        int32_t volts = pitchCv / 256;
        int32_t remainder = pitchCv % 256;
        if (remainder < 0) {
            remainder += 256;
            --volts;
        }

        if (volts > kMaxOctaveShift) volts = kMaxOctaveShift;
        if (volts < -kMaxOctaveShift) volts = -kMaxOctaveShift;

        uint32_t factor = kExp2Table[static_cast<uint8_t>(remainder)];
        uint32_t inc = static_cast<uint32_t>((static_cast<uint64_t>(kBasePhaseInc) * factor) >> 16);

        if (volts >= 0) {
            inc <<= volts;
        } else {
            inc >>= -volts;
        }

        if (inc == 0) inc = 1;
        return inc;
    }
};

int main() {
    CypressCard card;
    card.EnableNormalisationProbe();
    card.Run();
}
