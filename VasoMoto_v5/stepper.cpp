#include "stepper.h"
#include <Arduino.h>
#include <Wire.h>
//constructor
Stepper::Stepper(int stepPin, int dirPin, int enPin, int vHiPin, int m0Pin, int m1Pin)
{
  _stepPin = stepPin;
  _dirPin = dirPin;
  _enPin = enPin;
  _vHiPin = vHiPin;
  _m0Pin = m0Pin;
  _m1Pin = m1Pin;

  _previous_us = 0;
  _current_us = 0;
}

void Stepper::begin()
{
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  pinMode(_m0Pin, OUTPUT);
  pinMode(_m1Pin, OUTPUT);
  pinMode(_vHiPin, OUTPUT);
  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);  //LOW is CW. HIGH is CCW.
  digitalWrite(_enPin, HIGH);
  digitalWrite(_vHiPin, HIGH);
}

void Stepper::disable() {
  if (_isEnabled) {
    digitalWrite(_enPin, HIGH); // Disable motor output
    _isEnabled = false;
  }
}

void Stepper::move(int enable, int stepDelay, int dir) 
{
  // 1. Handle enabling/disabling the motor
  if (enable == 0) {
    disable();
    return;
  }
  
  if (!_isEnabled) {
    digitalWrite(_enPin, LOW); // Active-low enable
    _isEnabled = true;
  }

  // 2. Set the direction (Map the 1 or -1 from the INO to HIGH/LOW)
  int dirState = (dir == 1) ? HIGH : LOW; 
  if (dirState != _currentDir) {
    digitalWrite(_dirPin, dirState);
    _currentDir = dirState;
  }

  // 3. Generate a non-blocking step pulse
  _current_us = micros();
  if (_current_us - _previous_us >= stepDelay) {
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(2); // A tiny 2-microsecond HIGH pulse is enough for most drivers
    digitalWrite(_stepPin, LOW);
    
    _previous_us = _current_us;
  }
}

void Stepper::setStepFrac(int fraction)
{
  _fraction = fraction;
  //m0-m1:  LOW-LOW is 1/8; LOW-HI is 1/32; HI-LOW is 1/64; HI-HI is 1/16. speed is in rotations per second.
  switch (_fraction) {
    case 8:
      digitalWrite(_m0Pin, LOW);
      digitalWrite(_m1Pin, LOW);
      break;
    case 16:
      digitalWrite(_m0Pin, HIGH);
      digitalWrite(_m1Pin, HIGH);
      break;
    case 32:
      digitalWrite(_m0Pin, LOW);
      digitalWrite(_m1Pin, HIGH);
      break;
    case 64:
      digitalWrite(_m0Pin, HIGH);
      digitalWrite(_m1Pin, LOW);
      break;
  }
}

int Stepper::setStepFracSpeed(int fraction, int speed) 
{
  _fraction = fraction;
  _speed = speed;
//m0-m1:  LOW-LOW is 1/8; LOW-HI is 1/32; HI-LOW is 1/64; HI-HI is 1/16. speed is in rotations per second.
  if (_fraction == 8) {
    digitalWrite(_m0Pin, LOW);
    digitalWrite(_m1Pin, LOW);
    if (_speed == 0) {
      stepperDelay = 0;
    }
    else if (_speed >= 1) {
      stepperDelay = 1000000 / (_fraction * _speed);
    }
  }
  if (_fraction == 16) {
    digitalWrite(_m0Pin, HIGH);
    digitalWrite(_m1Pin, HIGH);
    if (_speed == 0) {
      stepperDelay = 0;
    }
    else if (_speed >= 1) {
      stepperDelay = 1000000 / (_fraction * _speed);
    }
  }
  if (_fraction == 32) {
    digitalWrite(_m0Pin, LOW);
    digitalWrite(_m1Pin, HIGH);
    if (_speed == 0) {
      stepperDelay = 0;
    }
    else if (_speed >= 1) {
      stepperDelay = 1000000 / (_fraction * _speed);
    }
  }
  if (fraction == 64) {
    digitalWrite(_m0Pin, HIGH);
    digitalWrite(_m1Pin, LOW);
    if (_speed == 0) {
      stepperDelay = 0;
    }
    else if (_speed >= 1) {
      stepperDelay = 1000000 / (_fraction * _speed);
    }
  }
  return stepperDelay;
}

double ADS1115::measure(int source)
{
  _source = source;
  if (_source == 0) {
    readADC = readADC_Differential_0_1();
  }
  else if (_source == 1) {
    readADC = readADC_Differential_2_3();
  }
  converted = ((_outputMax - _outputMin) / (_inputMax - _inputMin)) * (readADC - _inputMin) + _outputMin;
  return (converted);
}

void ADS1115::linearCal(int inputMin, int inputMax, double outputMin, double outputMax)
{
  _inputMin = inputMin;
  _inputMax = inputMax;
  _outputMin = outputMin;
  _outputMax = outputMax;
}

ADS1115::ADS1115() 
{
  m_bitShift = 0;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  m_dataRate = RATE_ADS1115_128SPS;
}

bool ADS1115::begin(uint8_t i2c_addr, TwoWire *wire) {
  m_i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  return m_i2c_dev->begin();
}

void ADS1115::setGain(adsGain_t gain) { m_gain = gain; }
adsGain_t ADS1115::getGain() { return m_gain; }
void ADS1115::setDataRate(uint16_t rate) { m_dataRate = rate; }

void ADS1115::startADCReading(uint16_t mux, bool continuous) {
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
      ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

  if (continuous) {
    config |= ADS1X15_REG_CONFIG_MODE_CONTIN;
  } else {
    config |= ADS1X15_REG_CONFIG_MODE_SINGLE;
  }
  config |= m_gain;       // Set PGA/voltage range
  config |= m_dataRate;   // Set data rate
  config |= mux;          // Set channels
  config |= ADS1X15_REG_CONFIG_OS_SINGLE; // Set 'start single-conversion' bit
  writeRegister(ADS1X15_REG_POINTER_CONFIG, config); // Write config register to the ADC
  writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000); // Set ALERT/RDY to RDY mode.
  writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}

int16_t ADS1115::readADC_Differential_0_1() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  
  unsigned long startWait = millis();
  while (!conversionComplete()) {
    if (millis() - startWait > 10) {
      break; // Timeout after 10ms to prevent hardware freeze
    }
    yield(); // Let the SAMD51 handle background tasks (like USB/Serial)
  }
  return getLastConversionResults();  
}

int16_t ADS1115::readADC_Differential_2_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/false);
  
  unsigned long startWait = millis();
  while (!conversionComplete()) {
    if (millis() - startWait > 10) {
      break; // Timeout after 10ms to prevent hardware freeze
    }
    yield(); 
  }
  return getLastConversionResults(); 
}

int16_t ADS1115::getLastConversionResults() {
  uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT) >> m_bitShift; // Read the conversion results
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    if (res > 0x07FF) { // Shift 12-bit results right 4 bits for the ADS1015, making sure we keep the sign bit intact
      res |= 0xF000;  // negative number - extend the sign to 16th bit
    }
    return (int16_t)res;
  }
}

void ADS1115::requestADC_Differential_0_1() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
}

void ADS1115::requestADC_Differential_2_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/false);
}

double ADS1115::getMeasureResult() {
  readADC = getLastConversionResults();
  converted = ((_outputMax - _outputMin) / (_inputMax - _inputMin)) * (readADC - _inputMin) + _outputMin;
  return converted;
}

bool ADS1115::conversionComplete() {
  return (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
}

void ADS1115::writeRegister(uint8_t reg, uint16_t value) {
  buffer[0] = reg;
  buffer[1] = value >> 8;
  buffer[2] = value & 0xFF;
  m_i2c_dev->write(buffer, 3);
}

uint16_t ADS1115::readRegister(uint8_t reg) {
  buffer[0] = reg;
  m_i2c_dev->write(buffer, 1);
  m_i2c_dev->read(buffer, 2);
  return ((buffer[0] << 8) | buffer[1]);
}