#include "ComputerCard.h"
#include <cstdint>
#include <cmath>
#include "pico/multicore.h"
#include "bsp/board.h"
#include "tusb.h"

// Minimal MIDI message reading struct
struct MIDIMessage
{
	// Parse a MIDI message from the raw packet data
	MIDIMessage(uint8_t* packet)
	{
		channel = packet[0]&0x0F;
		command = static_cast<Command>(packet[0] & 0xF0);
		switch (packet[0]&0xF0)
		{
		case Command::NoteOn: // two separate data bytes
		case Command::NoteOff:
		case Command::Aftertouch:
		case Command::CC:
			note = packet[1];
			velocity = packet[2];
			break;
			
		case Command::PatchChange: // single data bytes
			instrument = packet[1];
			break;
			
		case Command::PitchBend: // pitch bend combines msb/lsb
			pitchbend = packet[1] + (packet[2]<<7) - 8192;
			break;
			
		default:
			command = Command::Unknown;
			break;
		}
	}
	
	enum Command {Unknown = 0x00, NoteOn = 0x90, NoteOff = 0x80,
		PitchBend = 0xE0, Aftertouch = 0xA0, CC = 0xB0,	PatchChange = 0xC0};
	Command command;
	uint8_t channel;
	union // Store MIDI data bytes in two-byte union
	{
		struct
		{
			union // data byte 1
			{
				uint8_t note; // note on/off/aftertouch
				uint8_t cc; // CC
				uint8_t instrument; // patch change
			};
			union // data byte 2
			{
				uint8_t velocity; // note on/off
				uint8_t touch; // aftertouch
				uint8_t value; // CC 
			};
		};
		int16_t pitchbend;
	};
};

int32_t log2_note[12];

struct QuantizedNote {
  int32_t octave;   // 0..(octaves-1)
  int32_t note;     // 0..(notesPerOctave-1)
  int32_t step;    // 0..(octaves*notesPerOctave-1)
};

void InitializeNotes()
{
	for (int i=0; i<12; i++)
	{
		log2_note[i] = (log2f(log2f(4.0f + i)) - 1.0f)*1000.0f;
	}
}

// Maps adc in [0..4091] to step in [0..totalSteps-1] with no overflow and
// ensures adc=4091 -> step=totalSteps-1.
static inline QuantizedNote Quantize_12v(int32_t adc,
										 int32_t octaves = 12,
										 int32_t notesPerOctave = 12)
{
	adc += 2048;
	// Clamp just in case
	if (adc > 4091) adc = 4091;

	int32_t totalSteps = octaves * notesPerOctave; // e.g. 144
	// Use 4092 as the span so the top code lands on the last step.
	int32_t step = (adc * totalSteps) / 4092;

	QuantizedNote q;
	q.step   = step;
	q.octave = step / notesPerOctave;
	q.note   = step % notesPerOctave;
	return q;
}

static inline QuantizedNote Quantize_MidiNote(int32_t midiNote,
										 int32_t octaves = 12,
										 int32_t notesPerOctave = 12)
{
	if (midiNote < 0) midiNote = 0;
	if (midiNote > 127) midiNote = 127;

	int32_t totalSteps = octaves * notesPerOctave; // e.g. 144
	// Map MIDI note [0..127] to step [0..totalSteps-1]
	int32_t step = (midiNote * totalSteps) / 128;

	QuantizedNote q;
	q.step   = step;
	q.octave = step / notesPerOctave;
	q.note   = step % notesPerOctave;
	return q;
}



class AltQuant : public ComputerCard
{
public:
	AltQuant()
	{
		// Start the second core
		multicore_launch_core1(core1);
	}

	// Boilerplate to call member function as second core
	static void core1()
	{
		((AltQuant *)ThisPtr())->USBCore();
	}

inline bool CVOut1MIDINote(int32_t midiNote)
{
	QuantizedNote q = Quantize_MidiNote(midiNote);
	int32_t cvmv = -6000 + q.octave * 1000 + log2_note[q.note];
	return CVOut1Millivolts(cvmv);
}
	// Code for second RP2040 core, blocking
	// Handles MIDI in/out messages
	void USBCore()
	{
		uint8_t lastCCOut = 0;
		uint8_t packet[32];

		// Initialise TinyUSB
		board_init();
		tusb_init();
		
		while (1)
		{
			tud_task();
			while (tud_midi_available())
			{
				// Read MIDI input
				tud_midi_stream_read(packet, sizeof(packet));
				MIDIMessage m(packet);

				// Handle MIDI note on/off and mod wheel
				switch (m.command)
				{
				case MIDIMessage::NoteOn:
					if (m.velocity > 0 && !Connected(Input::CV1)) // real note on
					{
						CVOut1MIDINote(m.note);
						LedOn(4);
						PulseOut1(true);
					}
					else // note on with velocity 0 = note off
					{
						LedOff(4);
						PulseOut1(false);
					}
					break;
					
				case MIDIMessage::NoteOff:
					LedOff(4);
					PulseOut1(false);
					break;
					
				case MIDIMessage::CC:
					// Mod wheel -> CV out 2
					if (m.cc == 1 && !Connected(Input::CV2))
					{
					
						CVOut2(m.value << 4);
						LedBrightness(3, m.value << 4);
					}
					break;
					
				default:
					break;
				}
			}

			// Read main knob, and if value has changed, send 
			// MIDI CC 1 (Mod wheel) on channel 1 with knob position
			uint8_t ccOut = KnobVal(Knob::Main) >> 5;
			if (ccOut != lastCCOut)
			{
				uint8_t ccmesg[3] = {0xB0, 0x01, ccOut};
				tud_midi_stream_write(0, ccmesg, 3);
				lastCCOut = ccOut;
			}
		}
	}  

	virtual void ProcessSample()
	{
	    int32_t qin1 = CVIn1();
	    int32_t qin2 = CVIn2();
		
	    QuantizedNote q1 = Quantize_12v(qin1);
		QuantizedNote q2 = Quantize_12v(qin2);

		int32_t cv1mv = -6000 + q1.octave * 1000 + log2_note[q1.note];
		int32_t cv2mv = -6000 + q2.octave * 1000 + log2_note[q2.note];
    
		// Output desired voltages on CV out
		if (Connected(Input::CV1))
		{
			bool cv1limited = CVOut1Millivolts(cv1mv);
			LedOn(0, cv1limited);
		}
		if (Connected(Input::CV2))
		{
			bool cv2limited = CVOut2Millivolts(cv2mv);
			LedOn(1, cv2limited);
		}

		// Light top left/right LED, if requested voltage
		// on corresponding channel is not possible

		// Light bottom right LED if CV outputs have not been calibrated
		LedOn(5, !CVOutsCalibrated());
	}
};


int main()
{
	InitializeNotes();
	AltQuant aq;
	aq.EnableNormalisationProbe();
	aq.Run();
}
