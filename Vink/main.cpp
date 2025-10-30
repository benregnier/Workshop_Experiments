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
		// Todo: Implement ring modulation
		// Todo: Implement limiter
		
		// Transfer audio/CV/Pulse inputs directly to outputs
		AudioOut1(AudioIn1());
		AudioOut2(AudioIn2());

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
	Vink v;
	v.Run();
}

  
