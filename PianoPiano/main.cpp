#include "ComputerCard.h"
#include <cstring>

class CombFilter {
public:
    CombFilter(int delaySamples, int tableSize, int16_t feedback, int16_t impulseAmp)
        : size(delaySamples), fb(feedback), writeIdx(0), prev(0),
          tableSize(tableSize), position(0), impulse(impulseAmp) {
        buffer = new int16_t[size];
        memset(buffer, 0, size * sizeof(int16_t));
    }

    ~CombFilter() { delete[] buffer; }

    // Generate one sample; restart when position reaches tableSize.
    int16_t play() {
        // Trigger the impulse at the start of each cycle.
        int16_t input = (position == 0) ? impulse : 0;

        int16_t output = process(input);

        // Advance cycle position.
        position++;
        if (position >= tableSize) position = 0;

        return output;
    }

private:
    // Process one sample using the comb filter and return a 12‑bit output sample.
    int16_t process(int16_t input) {
        int16_t delayed = buffer[writeIdx];

        // Fixed‑point multiply: feedback in Q15
        int32_t fbComp = (static_cast<int32_t>(delayed) * fb) >> 15;
        int32_t sum = static_cast<int32_t>(input) + fbComp;

        // option 1: damping
        // Gentle damping: weight current sum more heavily than previous value
        // Here 7/8 current + 1/8 previous; adjust weights to taste
        //int32_t damped = (sum * 7 + prev) >> 4;

        //Option 2: very gentle damping (15/16 new + 1/16 old)
        int32_t damped = (sum * 15 + prev) >> 4;

         // Option 3: no damping (maximum sustain)
        // int32_t damped = sum;

    // Option 2: very gentle damping (15/16 new + 1/16 old)
    // int32_t damped = (sum * 15 + prev) >> 4;

        // Saturate damped value to 16‑bit range
        if (damped > 32767) damped = 32767;
        else if (damped < -32768) damped = -32768;

        prev = static_cast<int16_t>(damped);
        buffer[writeIdx] = prev;

        // Move the delay index forward
        if (++writeIdx >= size) writeIdx = 0;

        // Scale to 12‑bit output by shifting right by 4 (divide by 16)
        // and clamp to [-2048, 2047]
        int32_t scaled = prev >> 4;
        if (scaled > 2047) scaled = 2047;
        else if (scaled < -2048) scaled = -2048;

        return static_cast<int16_t>(scaled);
    }

    int16_t *buffer;
    int size;
    int16_t fb;
    int writeIdx;
    int16_t prev;
    int tableSize;
    int position;
    int16_t impulse;
};

class PianoPiano : public ComputerCard{
    public:

        const int sampleRate  = 48000;                      // 48 kHz sample rate
        const int targetFreq  = 240;                        // pitch
        const int delaySamples = sampleRate / targetFreq;   // ≈ samples for 440 Hz
        const int tableSize   = sampleRate * 1;             // play for 1 seconds

        // Feedback of 0.98 (~32113 in Q15) for longer sustain
        const int16_t feedbackQ15 = (32768 * 97) / 100;
        // Full‑scale 16‑bit impulse for maximum loudness
        const int16_t impulseAmp  = 8192;

        CombFilter comb;

        PianoPiano():comb(delaySamples, tableSize, feedbackQ15, impulseAmp){

        }

        virtual void ProcessSample() {
            int16_t sample = comb.play();
            AudioOut1(sample);
            AudioOut2(sample);
        }
};

int main(){
    PianoPiano pp;
    pp.Run();
}
