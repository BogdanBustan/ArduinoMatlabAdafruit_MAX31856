# Arduino Matlab Adafruit MAX31856 Thermocouple Breakout Library

## General Information
Adapted Arduino library of Adafruit MAX31856 Thermocouple Breakout for use in MATLAB.
https://www.adafruit.com/product/3263  
Written in MATLAB R2018b.

## Software Prerequisites
1. MATLAB R2014a to R2019a: https://www.mathworks.com/products/matlab.html
2. MATLAB Support Package for Arduino Hardware: https://www.mathworks.com/matlabcentral/fileexchange/47522-matlab-support-package-for-arduino-hardware

## Basic Usage
1. Creating an Adafruit_MAX31856 object: `ada_ard = Adafruit_MAX31856('COM7','Leonardo','D10')`
2. Reading the thermocouple temperature: `ada_ard.readThermocoupleTemperature`
3. Reading the cold junction temperature: `ada_ard.readCJTemperature`

## Wiring
These sensors use SPI to communicate, 4 pins are required to interface: SCLK, MISO, MOSI, CS. Use the hardware defined SPI pins. CS pin
can be chosen arbitrary.  
More at: https://learn.adafruit.com/adafruit-max31856-thermocouple-amplifier/wiring-and-test  
Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

## License and Credits
Original Arduino library written by Limor Fried/Ladyada for Adafruit Industries under BSD license. For more information check Adafruit
Github repository:  
https://github.com/adafruit/Adafruit_MAX31856 
