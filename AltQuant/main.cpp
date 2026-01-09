#include "ComputerCard.h"
#include <stdint.h>

// Quantizer that provides alternative 12TET tunings
// LOG(LOG(4 + 12 * $E12, 2), 2) - 1


int32_t log2_note[12] = {0, 215, 370, 489, 585, 664, 732, 791, 842, 888, 929, 966}; // log2 values for 12TET in millivolts

struct QuantizedNote {
  uint8_t octave;   // 0..(octaves-1)
  uint8_t note;     // 0..(notesPerOctave-1)
  uint16_t step;    // 0..(octaves*notesPerOctave-1)
};

// Maps adc in [0..4091] to step in [0..totalSteps-1] with no overflow and
// ensures adc=4091 -> step=totalSteps-1.
static inline QuantizedNote Quantize_12v(uint16_t adc,
                                            uint8_t octaves = 12,
                                            uint8_t notesPerOctave = 12)
{
  // Clamp just in case
  if (adc > 4091) adc = 4091;

  const uint16_t totalSteps = (uint16_t)octaves * (uint16_t)notesPerOctave; // e.g. 144
  // Use 4092 as the span so the top code lands on the last step.
  const uint16_t step = (uint16_t)(((uint32_t)adc * totalSteps) / 4092u);

  QuantizedNote q;
  q.step   = step;
  q.octave = (uint8_t)(step / notesPerOctave);
  q.note   = (uint8_t)(step % notesPerOctave);
  return q;
}

	

class AltQuant : public ComputerCard
{
public:

  

	virtual void ProcessSample()
	{
	    int32_t qin1 = CVIn1();
	    int32_t qin2 = CVIn2();
	
	    QuantizedNote q1 = Quantize_12v(qin1);
		QuantizedNote q2 = Quantize_12v(qin2);

		int32_t cv1mv = q1.octave * 1000 + log2_note[q1.note];
		int32_t cv2mv = q2.octave * 1000 + log2_note[q2.note];
    
		// Output desired voltages on CV out
		bool cv1limited = CVOut1Millivolts(cv1mv);
		bool cv2limited = CVOut2Millivolts(cv2mv);

		// Light top left/right LED, if requested voltage
		// on corresponding channel is not possible
		LedOn(0, cv1limited);
		LedOn(1, cv2limited);

		// Light bottom right LED if CV outputs have not been calibrated
		LedOn(5, !CVOutsCalibrated());
	}
};


int main()
{
	AltQuant aq;

	aq.Run();
}
