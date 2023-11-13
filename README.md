### Functionality
- Pot sends CC10
- Slider sends CC11
- Slider has a deadzone in the middle where it outputs value 64 constantly. So it can safely be used with the DJFX Filter of M8.
- First 8 buttons act as toggles to mute/unmute tracks 1 to 8. Last 4 buttons act as momentary switches that mute tracks 1&2, 3&4, 5&6, 7&8.

### Flashing Instructions (thanks <a href=https://github.com/roge-rm/NMCode> roge-rm </a>)
1. Download <a href=https://www.arduino.cc/en/software>Arduino</a>
2. <a href=https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/>Install ESP32</a> in Arduino
3. Load sketch, install required libraries (<a href=https://github.com/FortySevenEffects/arduino_midi_library>MIDI</a>, <a href=https://github.com/thomasfredericks/Bounce2>Bounce2</a>)
4. Select board Firebeetle-ESP32
5. Plug in USB to serial FTDI adapter, select port in Arduino
6. Connect adapter to NMSVE - see <a href=https://github.com/roge-rm/NMCode/blob/main/images/pinout.png>pinout</a>
- Black/GND to GND on NMSVE
- Green/TX to RX on NMSVE
- White/RX to TX on NMSVE
7. Turn on NMSVE while holding boot pin to ground wire - POWER and CONNECT LEDs should be solid
8. Flash in Arduino
