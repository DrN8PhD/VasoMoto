# VasoMoto
## Monitor and control cannulated vessel pressure in one easy-to-build unit

This arduino-based code is used to program the VasoMoto controller.
Paired with [VasoTracker-2 software](https://github.com/VasoTracker/VasoTracker-2), intravascular pressure can be monitored and controlled without the need for costly interface units.

This system is also capable of simulating pulsatile changes in pressure at rates approaching normal murine heart rate (~400 bpm). A few notes:

* While adaptable to other Arduino versions, this code is specifically written to utilize SAMD51-based microcontrollers. The [ItsyBitsy by Adafruit Inc.](https://www.adafruit.com/product/3800) is recommended.
* A myriad of libraries are required in addition to the custom ones included here.
* The parts list and assembly manual are also included in the release packages.
* This system requires a custom PCB. This is available at a nominal cost through the [Cubi<sup>3</sup>C Core Facility](https://phmtox.msu.edu/facilities/cubi3c) at Michigan State University. Costs will vary depending on the level of assembly you want, but I assure you -- it's far less than any other system on the market.

## v4.2
* First major update. I noticed that the sampling rate was too high relative to the pressure sensing resolution, resulting in weird pressure oscillations that I couldn't notice until we were tracking pressure with VasoTracker 2. This should all be fixed, resulting in better, more accurate pressure regulation. It's a bit slower for the line filling at first, but this is a small price to pay for healthier vessels.

## v4.1.1
* Added slow pressure ramp. This is different than the step protocol in VasoTracker. This slowly increases the set pressure at a given rate in mmHg/minute.
  
## v4.1.0
* Multiple bug fixes.
* Additional code to allow VasoTracker to control pressure and do timed pressure steps without the need for NI interface.
* Additoonal commenting throughout for clarity in the Arduino code.
