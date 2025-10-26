#include "ComputerCard.h"

#include "pico/multicore.h"
#include "bsp/board.h"
#include "tusb.h"
#include "usb_midi_host.h"

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

class MIDIDeviceHost : public ComputerCard
{
public:
	MIDIDeviceHost()
	{
		midi_dev_addr = 0;
		device_connected = 0;
		
		counter = 0;
		powerState = Unsupported;
		
		// Start the second core
		multicore_launch_core1(core1);
	}

	// Boilerplate to call member function as second core
	static void core1()
	{
		((MIDIDeviceHost *)ThisPtr())->USBCore();
	}


	// Send alternate note-on and note-off messages
	void SendNextNote(bool isHost)
	{
		static uint8_t noteOn[3] = {0x90, 0x5f, 0x7f};
		static uint8_t noteOff[3] = {0x80, 0x5f, 0x00};
		static bool noteOnNext = true;

		// Transmit the note message on the highest cable number
		uint8_t cable = tuh_midih_get_num_tx_cables(midi_dev_addr) - 1;


		// Toggle between sending note-on and note-off messages
		if (noteOnNext)
		{
			if (isHost)
			{
				tuh_midi_stream_write(midi_dev_addr, cable, noteOn, sizeof(noteOn));
			}
			else
			{
				tud_midi_stream_write(0, noteOn, sizeof(noteOn));
			}
		}
		else
		{
			if (isHost)
			{
				tuh_midi_stream_write(midi_dev_addr, cable, noteOff, sizeof(noteOff));
			}
			else
			{
				tud_midi_stream_write(0, noteOff, sizeof(noteOff));
			}
		}

		// Toggle between note on / note off
		noteOnNext = !noteOnNext;
	}

	
	// Code for second RP2040 core, blocking
	// Handles MIDI in/out messages
	void USBCore()
	{
		uint8_t packet[32];

		powerState = USBPowerState();

		// On reboot, choose between USB device or USB host mode
		if (powerState == UFP)
		{
			tud_init(TUD_OPT_RHPORT);
		}
		else if (powerState == DFP)
		{
			tuh_init(TUD_OPT_RHPORT);
		}

		board_init();
		
		while (1)
		{
			if (powerState == UFP) // Computer is USB device
			{
				tud_task();
				while (tud_midi_available())
				{
					// Read and discard MIDI input
					tud_midi_stream_read(packet, sizeof(packet));
			
				}
				
				if (counter >= 20000)
				{
					SendNextNote(false);
					counter -= 20000;
				}
			}
			else if (powerState == DFP) // Computer is USB host
			{
				tuh_task();
		
				bool connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);

				// device must be attached and have at least one endpoint ready to receive a message
				if (connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
				{

					if (counter >= 20000)
					{
						SendNextNote(true);
						counter -= 20000;
					}

					// Send a USB packet immediately (even though in this case,
					// we are not close to the 64-byte maximum payload)
					tuh_midi_stream_flush(midi_dev_addr);
				}
			}
		}

	}

	
	// 48kHz audio processing function
	virtual void ProcessSample()
	{
		// No audio I/O, so just flash an LED
		// to indicate that the card is running
		static int32_t frame=0;

		// Top two LEDS held on if we can't determine USB power state
		if (powerState == Unsupported)
		{
			LedOn(0);
			LedOn(1);
		
		}
		else // otherwise, flash LED 5 if DFP, LED 4 if UFP
		{
			LedOff(0);
			LedOff(1);
			int activeLED = (powerState==DFP)?5:4;
			int inactiveLED = (powerState==DFP)?4:5;
			LedOn(activeLED, (frame>>13)&1);
			LedOff(inactiveLED);
		}
		frame++;

		
		// Counter is reset by other core
		if (counter <= 30000)
			counter++;
	}

	
	static uint8_t device_connected;
	static uint8_t midi_dev_addr;
	
private:
	USBPowerState_t powerState;
	volatile uint32_t counter;
};

uint8_t MIDIDeviceHost::device_connected;
uint8_t MIDIDeviceHost::midi_dev_addr;


// Four callback functions that rppicomidi/usb_midi_host uses

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
	(void)in_ep; (void)out_ep; (void)num_cables_rx; (void)num_cables_tx; // avoid unused variable warnings

	
	if (MIDIDeviceHost::midi_dev_addr == 0)
	{
		MIDIDeviceHost::midi_dev_addr = dev_addr;
		MIDIDeviceHost::device_connected = 1;
	}
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
	(void)instance;
	
	if (dev_addr == MIDIDeviceHost::midi_dev_addr)
	{
		MIDIDeviceHost::midi_dev_addr = 0;
		MIDIDeviceHost::device_connected = 0;
	}
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
	if (MIDIDeviceHost::midi_dev_addr != dev_addr)
		return;

	if (num_packets == 0)
		return;
	
	uint8_t cable_num;
	uint8_t buffer[48];
	while (1)
	{
		uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));

		if (bytes_read == 0)
			return;
		
		for (uint32_t idx = 0; idx < bytes_read; idx++)
		{
			//buffer[idx]
		}
	}

}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}






int main()
{
	MIDIDeviceHost mdh;
	mdh.Run();
}

  
