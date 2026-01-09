#include "ComputerCard.h"

// Quantizer that provides alternative 12TET tunings
// LOG(LOG(4 + 12 * $E12, 2), 2) - 1

class AltQuant : public ComputerCard
{
public:

  int32_t log2[] = {0, 215, 370, 489, 585, 664, 732, 791, 842, 888, 929, 966}; // log2 values for 12TET in millivolts

	virtual void ProcessSample()
	{
    int32_t qin1 = CVIn1()
    int32_t qin2 = CVIn2()

    //TODO:convert qin1 and 2 to voltages
    //TODO:separate octave and quantize remainder to 12TET
    //TODO: get log2 mv and combine with octave, write to cv1mv and cv2mv
    
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

