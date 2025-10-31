#include "ComputerCard.h"

/// Passthrough
/// Connects audio, CV and pulse inputs directly to corresponding outputs
/// Also displays switch position and knob values on LEDs
class Passthrough : public ComputerCard
{
public:
	
	virtual void ProcessSample()
	{
		// Let's play with bringing in values and then bringing them back out as a mixer
		//AudioOut1(AudioIn1());
		//AudioOut2(AudioIn2());

		int16_t in1 = AudioIn1();
		int16_t in2 = AudioIn2();

		//int16_t out = (in1/2 + in2/2); //mixer
		//int32_t mult = (in1 * in2) >> 15; //ring mod

		//int16_t out = mult << 4; //convert and amplify?

		int32_t delayed = buffer[writeIdx];

		int32_t fbComp = (static_cast<int32_t>(delayed) * fb) >> 15;

        int32_t sum = static_cast<int32_t>(in1) + delayed;


		if (sum > 32767) sum = 32767;
        else if (sum < -32768) sum = -32768;

        prev = static_cast<int16_t>(sum);

        buffer[writeIdx] = prev;

		// Move the delay index forward
        if (++writeIdx >= size) writeIdx = 0;
		
		// Scale to 12‑bit output by shifting right by 4 (divide by 16)
        // and clamp to [-2048, 2047]
        int32_t scaled = prev >> 4;
        if (scaled > 2047) scaled = 2047;
        else if (scaled < -2048) scaled = -2048;
		int16_t out = scaled << 4; //amplify?

		AudioOut1(out);

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

	int16_t *buffer;
    int size;
    int16_t fb;
    int writeIdx;
    int16_t prev;
    int tableSize;
    int position;
    int16_t impulse;
};


int main()
{
	Passthrough pt;
	
	pt.Run();
}

  
