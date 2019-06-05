# VoiceGrowler - 2019 - STM32
It's voice modification project with some fancy features like predefined audio and volume knob.
## How to run
Open `VoiceGrolwer.ioc` in `STM32CubeMX` and then Generate Project. Next just compile it and upload it to your STM32.
## Device Connections to STM32
### SD Card Reader
- `PB5` - `MOSI`
- `PB4` - `MISO`
- `PB3` - `SCK`
- `GND` - `CS`
### Microphone
- `PA1` - `Aout`
- `5V` - `Vcc`
### Volume Knob - Potentiometer
- `PB1` - `Uout`
- `5V` - `Vcc`
### Buttons
- `PE3`, `PE4`, `PE5`, `PE6` - Voice Effects (Echo, Overdrive, Low Voice, High Voice)
- `PE7`, `PE8`, `PE9`, `PE10` - Predefined Audio Files from SD Card

## How to use SD Card
Copy files from SDcard folder onto your SD Card and connect it to the Card Reader Module.
In case you would like to use your own files convert them into .wav having:
  - Bit resolution: 8 bit
  - Sampling rate: 16000 Hz
  - Audio channels: mono
### Converter website
https://audio.online-convert.com/convert-to-wav

### Used Tools
CubeMX
STM32Studio
Ecllipse

### Future Improvements
Volume meter based on WS2812 Diodes 

### Credits
Wojciech Kasperski\
Michał Kalinowski