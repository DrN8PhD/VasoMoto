# VasoMoto
## Monitor and control cannulated vessel pressure in one easy-to-build unit

This arduino-based code is used to program the VasoMoto controller.
Paired with [VasoTracker-2 software](https://github.com/VasoTracker/VasoTracker-2), intravascular pressure can be monitored and controlled without the need for costly interface units.

This system is also capable of simulating pulsatile changes in pressure at rates approaching normal murine heart rate (~400 bpm). A few notes:

* While adaptable to other Arduino versions, this code is specifically written to utilize SAMD51-based microcontrollers. The [ItsyBitsy by Adafruit Inc.](https://www.adafruit.com/product/3800) is recommended.
* A myriad of libraries are required in addition to the custom ones included here. They include:
  * FlashStorage library
  * Adafruit GFX library
  * Adafruit ST7735 library
* This URL (https://adafruit.github.io/arduino-board-index/package_adafruit_index.json) must be added to the Arduino IDE External Board Manager in order to write to the ItsyBitsy.
* This system requires a custom PCB. This is available at a nominal cost through the [Cubi<sup>3</sup>C Core Facility](https://phmtox.msu.edu/facilities/cubi3c) at Michigan State University. Costs will vary depending on the level of assembly you want, but I assure you -- it's far less than any other system on the market.

***Use at your own peril with other microcontrollers. I tried to clearly label what pins should be changed, but it's possible to incinerate your microcontroller if pin assignments are incorrect.***
--
## v4.1.4
* Bug fixes and integration into VasoTracker.
  
## v4.1.1
* Added slow pressure ramp. This is different than the step protocol in VasoTracker. This slowly increases the set pressure at a given rate in mmHg/minute.
  
## v4.1.0
* Multiple bug fixes.
* Additional code to allow VasoTracker to control pressure and do timed pressure steps without the need for NI interface.
* Additoonal commenting throughout for clarity in the Arduino code.
