## One liners for uploading firmware to multiple cards: 

```

///MIDI 

while true; do 
    if mount | grep -q "/Volumes/RPI-RP2"; then 
        cp Simple_Midi_0_2.ino.ino.uf2 /Volumes/RPI-RP2 && diskutil unmount /Volumes/RPI-RP2
        echo "File copied and disk unmounted, waiting for re-mount..."
    fi
    sleep 1
done


while true; do 
    if mount | grep -q "/Volumes/RPI-RP2"; then 
        cp 8mu_card.ino.uf2 /Volumes/RPI-RP2 && diskutil unmount /Volumes/RPI-RP2
        echo "File copied and disk unmounted, waiting for re-mount..."
    fi
    sleep 1
done


while true; do 
    if mount | grep -q "/Volumes/RPI-RP2"; then 
        cp reverb.uf2 /Volumes/RPI-RP2 && diskutil unmount /Volumes/RPI-RP2
        echo "File copied and disk unmounted, waiting for re-mount..."
    fi
    sleep 1
done


while true; do 
    if mount | grep -q "/Volumes/RPI-RP2"; then 
        cp 03_TuringMachine.ino.uf2 /Volumes/RPI-RP2 && diskutil unmount /Volumes/RPI-RP2
        echo "File copied and disk unmounted, waiting for re-mount..."
    fi
    sleep 1
done

```
