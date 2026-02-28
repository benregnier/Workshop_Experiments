#include "ComputerCard.h"

#include <array>
#include <cstdint>

#include "bsp/board.h"
#include "pico/multicore.h"
#include "tusb.h"

namespace {

constexpr uint32_t kSampleRate = 48000;
constexpr uint32_t kGateSamples = kSampleRate / 100; // 10 ms
constexpr size_t kChannels = 2;
constexpr size_t kMaxSteps = 128;
constexpr size_t kMinSteps = 4; // one beat
constexpr uint16_t kKnobMid = 2048;
constexpr uint16_t kPickupWindow = 48;

struct StepEvent {
    bool active = false;
    uint8_t note = 60;
    uint8_t velocity = 100;
    uint16_t levelQ15 = 32767;
};

struct MIDIMessage {
    enum Command : uint8_t { Unknown = 0x00, NoteOff = 0x80, NoteOn = 0x90 };

    explicit MIDIMessage(const uint8_t* packet) {
        command = static_cast<Command>(packet[0] & 0xF0);
        channel = packet[0] & 0x0F;
        data1 = packet[1];
        data2 = packet[2];
    }

    Command command = Unknown;
    uint8_t channel = 0;
    uint8_t data1 = 0;
    uint8_t data2 = 0;
};

struct SharedNoteIn {
    volatile bool pending = false;
    volatile uint8_t note = 60;
    volatile uint8_t velocity = 100;
};

struct SharedNoteOut {
    volatile bool pending = false;
    volatile bool on = false;
    volatile uint8_t channel = 0;
    volatile uint8_t note = 60;
    volatile uint8_t velocity = 100;
};

enum KnobIndex : uint8_t { Speed = 0, Length = 1, Decay = 2 };

struct LoopControl {
    uint16_t speedKnob = kKnobMid;
    uint16_t lengthKnob = kKnobMid;
    uint16_t decayKnob = kKnobMid;
    bool speedPicked = false;
    bool lengthPicked = false;
    bool decayPicked = false;
};

class PocketDimension : public ComputerCard {
public:
    PocketDimension() { multicore_launch_core1(Core1Entry); }

    void ProcessSample() override {
        UpdateControlSelection();
        UpdateLoopControls();
        HandleIncomingMIDI();
        TickLoop(0);
        TickLoop(1);
        RenderOutputs();
        RenderLeds();
    }

private:
    static void Core1Entry() { static_cast<PocketDimension*>(ThisPtr())->USBCore(); }

    void USBCore() {
        uint8_t packet[4] = {};
        board_init();
        tusb_init();

        while (true) {
            tud_task();
            while (tud_midi_available()) {
                if (tud_midi_stream_read(packet, sizeof(packet)) < 3) {
                    continue;
                }
                MIDIMessage message(packet);
                if (message.channel >= kChannels) {
                    continue;
                }
                if ((message.command == MIDIMessage::NoteOn) && (message.data2 > 0)) {
                    auto& in = midiIn_[message.channel];
                    in.note = message.data1;
                    in.velocity = message.data2;
                    in.pending = true;
                }
            }

            for (size_t ch = 0; ch < kChannels; ++ch) {
                auto& out = midiOut_[ch];
                if (!out.pending) {
                    continue;
                }

                uint8_t msg[3] = {
                    static_cast<uint8_t>((out.on ? 0x90 : 0x80) | (out.channel & 0x0F)),
                    out.note,
                    out.velocity,
                };
                tud_midi_stream_write(0, msg, sizeof(msg));
                out.pending = false;
            }
        }
    }

    void UpdateControlSelection() {
        const Switch sw = SwitchVal();
        if (sw == Switch::Down) {
            controlledLoop_ = 0;
        } else if (sw == Switch::Up) {
            controlledLoop_ = 1;
        }
    }

    static bool TryPickup(uint16_t physical, uint16_t stored, bool alreadyPicked) {
        if (alreadyPicked) {
            return true;
        }
        const uint16_t diff = (physical > stored) ? (physical - stored) : (stored - physical);
        return diff <= kPickupWindow;
    }

    void UpdateLoopControls() {
        LoopControl& c = controls_[controlledLoop_];
        const uint16_t speedRaw = KnobVal(Knob::Main);
        const uint16_t lengthRaw = KnobVal(Knob::X);
        const uint16_t decayRaw = KnobVal(Knob::Y);

        c.speedPicked = TryPickup(speedRaw, c.speedKnob, c.speedPicked);
        c.lengthPicked = TryPickup(lengthRaw, c.lengthKnob, c.lengthPicked);
        c.decayPicked = TryPickup(decayRaw, c.decayKnob, c.decayPicked);

        if (c.speedPicked) c.speedKnob = speedRaw;
        if (c.lengthPicked) c.lengthKnob = lengthRaw;
        if (c.decayPicked) c.decayKnob = decayRaw;
    }

    void HandleIncomingMIDI() {
        for (size_t ch = 0; ch < kChannels; ++ch) {
            auto& in = midiIn_[ch];
            if (!in.pending) continue;
            WriteEventAtPlayhead(static_cast<uint8_t>(ch), in.note, in.velocity);
            in.pending = false;
        }

        if (PulseIn1RisingEdge()) CaptureFromCV(0);
        if (PulseIn2RisingEdge()) CaptureFromCV(1);
    }

    void CaptureFromCV(size_t channel) {
        const int16_t pitch = (channel == 0) ? CVIn1() : CVIn2();
        const int16_t velIn = (channel == 0) ? AudioIn1() : AudioIn2();

        int32_t note = 60 + pitch / 32;
        if (note < 0) note = 0;
        if (note > 127) note = 127;

        int32_t v = velIn + 2048;
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        const uint8_t velocity = static_cast<uint8_t>((v * 127) / 4095);

        WriteEventAtPlayhead(static_cast<uint8_t>(channel), static_cast<uint8_t>(note), velocity);
    }

    void WriteEventAtPlayhead(uint8_t ch, uint8_t note, uint8_t velocity) {
        const size_t step = transportStep_[ch] % GetLoopLength(ch);
        auto& ev = loops_[ch][step];
        ev.active = true;
        ev.note = note;
        ev.velocity = velocity;
        ev.levelQ15 = 32767;
    }

    uint32_t GetLoop1StepSamples() const {
        // 500ms..50ms per step
        const uint16_t knob = controls_[0].speedKnob;
        const uint32_t ms = 500u - (static_cast<uint32_t>(knob) * 450u) / 4095u;
        return (kSampleRate * ms) / 1000u;
    }

    uint32_t GetLoop2StepSamples() const {
        // Loop 2 speed is a multiple of loop 1: 0.25x..4x
        const uint16_t knob = controls_[1].speedKnob;
        const uint32_t base = GetLoop1StepSamples();
        const uint32_t ratioMinQ12 = 1024;  // 0.25
        const uint32_t ratioMaxQ12 = 16384; // 4.0
        const uint32_t ratioQ12 = ratioMinQ12 + ((ratioMaxQ12 - ratioMinQ12) * knob) / 4095u;
        return (base * 4096u) / ratioQ12;
    }

    uint32_t GetStepSamples(size_t ch) const { return ch == 0 ? GetLoop1StepSamples() : GetLoop2StepSamples(); }

    size_t GetLoopLength(size_t ch) const {
        // Quantized to multiples of 4 steps
        const uint16_t knob = controls_[ch].lengthKnob;
        const size_t beats = 1 + (static_cast<size_t>(knob) * (kMaxSteps / 4u)) / 4096u;
        const size_t len = beats * 4u;
        return len < kMinSteps ? kMinSteps : (len > kMaxSteps ? kMaxSteps : len);
    }

    uint16_t GetDecayQ15(size_t ch) const {
        const uint16_t knob = controls_[ch].decayKnob;
        const uint32_t minQ15 = 4096;
        return static_cast<uint16_t>(32767u - ((32767u - minQ15) * knob) / 4095u);
    }

    void TickLoop(size_t ch) {
        sampleCounter_[ch]++;
        const uint32_t stepSamples = GetStepSamples(ch);
        if (sampleCounter_[ch] < stepSamples) {
            return;
        }
        sampleCounter_[ch] -= stepSamples;
        AdvanceStep(ch);
    }

    void AdvanceStep(size_t ch) {
        const size_t length = GetLoopLength(ch);
        const size_t step = transportStep_[ch] % length;
        auto& state = playback_[ch];
        auto& ev = loops_[ch][step];

        if (ev.active && ev.velocity > 0 && ev.levelQ15 > 256) {
            state.note = ev.note;
            state.amplitude = static_cast<uint8_t>((static_cast<uint32_t>(ev.velocity) * ev.levelQ15) >> 15);
            state.gateCountdown = kGateSamples;
            state.envCountdown = GetStepSamples(ch);
            state.active = true;
            QueueMidi(static_cast<uint8_t>(ch), true, ev.note, state.amplitude);
        } else {
            state.active = false;
            state.gateCountdown = 0;
            state.envCountdown = 0;
        }

        ++transportStep_[ch];
        if (transportStep_[ch] >= length) {
            transportStep_[ch] = 0;
            ApplyDecay(ch, length);
        }
    }

    void ApplyDecay(size_t ch, size_t length) {
        const uint16_t decayQ15 = GetDecayQ15(ch);
        for (size_t i = 0; i < length; ++i) {
            auto& ev = loops_[ch][i];
            if (!ev.active) continue;
            ev.levelQ15 = static_cast<uint16_t>((static_cast<uint32_t>(ev.levelQ15) * decayQ15) >> 15);
            if (ev.levelQ15 < 128) ev.active = false;
        }
    }

    void QueueMidi(uint8_t channel, bool on, uint8_t note, uint8_t velocity) {
        auto& out = midiOut_[channel];
        out.channel = channel;
        out.on = on;
        out.note = note;
        out.velocity = velocity;
        out.pending = true;
    }

    void RenderOutputs() {
        for (size_t ch = 0; ch < kChannels; ++ch) {
            auto& state = playback_[ch];
            if (state.gateCountdown > 0) {
                --state.gateCountdown;
                (ch == 0) ? PulseOut1(true) : PulseOut2(true);
            } else {
                (ch == 0) ? PulseOut1(false) : PulseOut2(false);
            }

            if (state.envCountdown > 0 && state.active) {
                --state.envCountdown;
                const int32_t centered = static_cast<int32_t>(state.amplitude) * 16 - 1024;
                const int16_t env = centered > 2047 ? 2047 : (centered < -2048 ? -2048 : centered);
                if (ch == 0) {
                    AudioOut1(env);
                    CVOut1MIDINote(state.note);
                } else {
                    AudioOut2(env);
                    CVOut2MIDINote(state.note);
                }
                if (state.envCountdown == 0) {
                    QueueMidi(static_cast<uint8_t>(ch), false, state.note, 0);
                    state.active = false;
                }
            } else {
                (ch == 0) ? AudioOut1(0) : AudioOut2(0);
            }
        }
    }

    void RenderLeds() {
        LedBrightness(0, controlledLoop_ == 0 ? 4095 : 800);
        LedBrightness(1, controlledLoop_ == 1 ? 4095 : 800);
        LedBrightness(2, static_cast<uint16_t>((GetLoopLength(controlledLoop_) * 4095u) / kMaxSteps));
        LedBrightness(3, static_cast<uint16_t>((GetDecayQ15(controlledLoop_) * 4095u) / 32767u));
        const uint32_t speed = GetStepSamples(controlledLoop_);
        const uint32_t minS = (kSampleRate * 50u) / 1000u;
        const uint32_t maxS = (kSampleRate * 500u) / 1000u;
        const uint32_t span = (maxS > minS) ? (maxS - minS) : 1;
        const uint32_t norm = (speed > minS) ? (speed - minS) : 0;
        LedBrightness(4, static_cast<uint16_t>((norm * 4095u) / span));
        LedOn(5, ((frame_++ >> 13) & 1) != 0);
    }

    struct PlaybackState {
        bool active = false;
        uint8_t note = 60;
        uint8_t amplitude = 0;
        uint32_t gateCountdown = 0;
        uint32_t envCountdown = 0;
    };

    std::array<std::array<StepEvent, kMaxSteps>, kChannels> loops_{};
    std::array<PlaybackState, kChannels> playback_{};
    std::array<SharedNoteIn, kChannels> midiIn_{};
    std::array<SharedNoteOut, kChannels> midiOut_{};
    std::array<LoopControl, kChannels> controls_{};

    std::array<uint32_t, kChannels> sampleCounter_{};
    std::array<size_t, kChannels> transportStep_{};

    size_t controlledLoop_ = 0;
    uint32_t frame_ = 0;
};

} // namespace

int main() {
    PocketDimension card;
    card.Run();
}
