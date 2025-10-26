#include "ComputerCard.h"

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



class MIDIDevice : public ComputerCard
{
public:
	MIDIDevice()
	{
		// Start the second core
		multicore_launch_core1(core1);
	}

	// Boilerplate to call member function as second core
	static void core1()
	{
		((MIDIDevice *)ThisPtr())->USBCore();
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
					if (m.velocity > 0) // real note on
					{
						CVOut1MIDINote(m.note);
						LedOn(1);
						PulseOut1(true);
					}
					else // note on with velocity 0 = note off
					{
						LedOff(1);
						PulseOut1(false);
					}
					break;
					
				case MIDIMessage::NoteOff:
					LedOff(1);
					PulseOut1(false);
					break;
					
				case MIDIMessage::CC:
					// Mod wheel -> CV out 2
					if (m.cc == 1)
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

	
	// 48kHz audio processing function
	virtual void ProcessSample()
	{
		// No audio I/O, so just flash an LED
		// to indicate that the card is running
		static int32_t frame=0;
		LedOn(5,(frame>>13)&1);
		frame++;
	}
};


int main()
{
	MIDIDevice md;
	md.Run();
}

  
