/*
 * VasoMoto v4.1.0 — Brief Commented Edition
 * Purpose: Add concise, high-level explanations above major functions and logic blocks,
 *          without changing any behavior. For lab collaborators and future maintainers.
 *
 * Key concepts:
 *  - Host protocol: 'SET P=<mmHg>' from PC, ACK back, and 25 Hz telemetry ('DATA T=... P=... P_SET=...').
 *  - Arbitration: while PC is active, manual encoder writes are gated; after 3s idle, manual resumes.
 *  - State machine: RUN / STOP / SIM paths with shared control/telemetry.
 */
/* This version includes external input to change pressure. All tension recording functions are disabled. Can be re-enabled for a second pressure
   transducer, but name should be changed to prevent confusion. */

/* NOTE: This version (4.1.0) is also specifically adapted to do slow pressure increases over time. The fast pulse functionality is removed
   in favor of this. For rapid pulse pressure shifting, see version 4.0.1.*/

#include <Wire.h>
#include "bitmaps.h"
#include "FlashStorage.h"
#include "avr/dtostrf.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "FreeSansBold7pt7b.h"
#include "FreeSansBold8pt7b.h"
#include "FreeSansBold9pt7b.h"
#include <Adafruit_ST7735.h>  // Hardware-specific library
#include "stepper.h"

/*ADC Setup*/
  ADS1115 ads;
  byte calibrationOrder;

/*TFT Setup*/
  #define TFT_CS A4
  #define TFT_DC A3
  #define TFT_RST 13  // Or set to -1 and connect to Arduino RESET pin. This only needs to be 13 when using TensoMoto board v2.0 (May 2023)
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
  int rainbow[34]{ 0x001F, 0x011F, 0x021F, 0x031F, 0x041F, 0x051F, 0x061F, 0x071F, 0x07FF, 0x07FC, 0x07F8, 0x07F4, 0x07F0, 0x07EC, 0x07E8, 0x07E4, 0x07E0, 0x27E0, 0x47E0, 0x67E0, 0x87E0, 0xA7E0, 0xC7E0, 0xE7E0, 0xFFE0, 0xFF00, 0xFE00, 0xFD00, 0xFC00, 0xFB00, 0xFA00, 0xF900, 0xF800 };

/*Rotary Encoder Setup and interrups*/
  #define pinDC 10
  #define pinCS 11
  #define enSW 12 
  volatile byte aFlag = 0;        //  let's us know when we're expecting a rising edge on pinDT to signal that the encoder has arrived at a detent
  volatile byte bFlag = 0;        //  let's us know when we're expecting a rising edge on pinCLK to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
  int encoderPos;                 //  this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
  volatile uint32_t reading = 0;  //  somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
  uint32_t maskA;
  uint32_t maskB;
  uint32_t maskAB;
  volatile uint32_t *port;
  bool box;
  int runStateRamp = 0;
  int runStateSim = 0;
  int runStateMoto = 0;
  bool fillLines = false;
  bool printSettings = true;
  int choice;
  bool beatDir = true;

  void fpinDC() {
    noInterrupts();
    reading = *port & maskAB;
    if ((reading == maskAB) && aFlag) {  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
      encoderPos--;                      //decrement the encoder's position count
      box = !box;
      bFlag = 0;  //reset flags for the next turn
      aFlag = 0;  //reset flags for the next turn
    } else if (reading == maskB) bFlag = 1;
    interrupts();  //signal that we're expecting pinCS to signal the transition to detent from free rotation
  }
  void fpinCS() {
    noInterrupts();
    reading = *port & maskAB;
    if (reading == maskAB && bFlag) {  //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
      encoderPos++;                    //increment the encoder's position count
      box = !box;
      bFlag = 0;  //reset flags for the next turn
      aFlag = 0;  //reset flags for the next turn
    } else if (reading == maskA) aFlag = 1;
    interrupts();  //signal that we're expecting pinDC to signal the transition to detent from free rotation
  }

/*Stepper Setup*/
  Stepper stepper(0, 9, 7, 5, 2, 3);
  unsigned long currentMicros;
  unsigned long previousMicros = 0;
  int pulseCounter = 0;
  float tempRate = 0.0;
  float actualRate = 0.0;
  int maxDelay = 10000;   //  Used in pressureControl to easily alter how slow the stepper starts, so I dont have to change in 12 places.
  int numSteps = 0;
  int stepCounter = 0;
  int stepsPerSec = 200;  //This is WHOLE steps per sec; the fractionation is accounted for later.
  int goUp = maxDelay;
  int goDown = maxDelay;
  int x = 1;
  int y = 1;

/*Pressure sensor calibration and setup*/
  int txdx1 = 0;                      //identity of first transducer
  int txdx2 = 1;                      //identity of second transducer
  int iMinP = 0;           // Raw value calibration lower point
  int iMaxP = 65535;       // Raw value calibration upper point
  float avgPressure = 0.0;
  float avgTension = 0.0;
  float avgSample = 0.0;
  float oMinP = 0.0;      // Pressure calibration lower point
  float oMaxP = 200.0;    // Pressure calibration upper point. This is in mmHg. This is about the max it can do.
  float PRESSURE_CAL_MIN = 0.00;
  float PRESSURE_CAL_MAX = 80.0;
  int iMinT = 0;            // Raw value calibration lower point
  int iMaxT = 65535;        // Raw value calibration upper point
  float oMinT = 0.0;       // Pressure calibration lower point
  float oMaxT = 1000.0;    // Pressure calibration upper point. This is in µN. This is about the max it can do.
  float TENSION_CAL_MIN = 0.00;
  float TENSION_CAL_MAX = 100.0;

/*Flash storage definitions and matrices*/
   struct cal_matrix {       //place to store all the values needed for linear calibration of sensors.
    bool valid_cal;
    bool valid_init;
    bool valid_ramp;
    bool valid_sim;
    float _pLowSel;
    float _pHiSel;
    float _tLowSel;
    float _tHiSel;
    float _multiplier;      //Use this to set the velocity multiplier to get the right shape of the pressure curve.             
    int _pLowADC;
    int _pHiADC;
    int _tLowADC;
    int _tHiADC;
    int _timeDelay;         //the time delay for wheatstone bridge output.
    int _filterWeight;      //the weighting of the averaging filter.
    int _acceleration;      //Use this to alter how fast the pump accelerates and to what speed.
    int _numSamples;        //the number of samples averaged. Was set to 5 in v200 for some reason?
    int _lowmmHg;
    int _highmmHg;
    int _pressRate;
    int _minmmHg;
    int _maxmmHg;
    int _pulseRate;
  } calib;

    float pLowSel;
    float pHiSel;
    float tLowSel;
    float tHiSel;
    int pLowADC;
    int pHiADC;
    int tLowADC;
    int tHiADC;
    float multiplier;          
    int timeDelay;       
    int filterWeight;    
    int acceleration;    
    int numSamples;
    int lowmmHg;
    int highmmHg;
    int pressRate;
    int minmmHg;
    int maxmmHg;
    int pulseRate;
  
  FlashStorage (calibrate, cal_matrix);

/*Serial receive vairables*/
  const byte numChars = 16;
  char receivedChars[numChars];
  bool newData = false;

/*Miscellaneous global variables*/
  unsigned long previousMillis = 0;
  unsigned long prevmillis = 0;
  unsigned long currmillis;
  unsigned long startMillis;
  unsigned long currentMillis;
  unsigned long newTime;
  double currentTime;
  int expType;
  int initMin = 60;     //Change this to alter min pressure starting point
  int initMax = 100;    //Change this to alter max pressure starting point
  int initSteps = 60;  //Change this to alter the pulse rate starting point
  int beatTime;
  int pulseInt;
  int ticker;
  float coloring;
  int sel_pressure = 0;
  unsigned long prevTiming;
  int direction = 1;
  bool UseStartTime = true;
  int mode;
  // bool moto = false;
  char number[8];
  char selected[4];
  char pressure[6];
  char tension[6];
  char range[8];
  char rate[6];
  char time[12];
  char RunningOutputMoto[28];
  char StoppingOutputMoto[28];
  char RunningOutputSim[32];
  char RunningOutputRamp[32];
  char StoppingOutputRamp[32];
  char StoppingOutputSim[32];
  char pressureBufferLCA[32];  //Pressure calib.pLowADC string
  char pressureBufferHCA[32];  //Pressurecalib.pHiADC string
  char pressureBufferLCS[32];  //Pressure calib.pLowSel string
  char pressureBufferHCS[32];  //Pressure calib.pHiSel string
  char tensionBufferLCA[6];  //Tension low_cal_ADC string
  char tensionBufferHCA[6];  //Tension high_cal_ADC string
  char tensionBufferLCS[4];  //Tension low_cal_sel string
  char tensionBufferHCS[4];  //Tension high_cal_sel string
  char bufferLP[32];   //low pressure string
  char bufferHP[32];   //high pressure string
  char bufferPR[32];   //pulse rate string
  char debugging[72]; //for ease of reading serial output stuff

/*VasoTracker PC control variables*/
// Host -> Device:  "SET P=<mmHg>". Device -> Host:  "ACK SET P=<mmHg> T=<ms>"
// Telemetry @ 25 Hz: "DATA T=<ms> P=<avgPressure> P_SET=<sel_pressure>"
static const uint16_t VM_TELEMETRY_HZ = 25;      // DATA lines per second
static const unsigned long VM_PC_TIMEOUT_MS = 3000;  // allow a longer grace period for PC control
static unsigned long _vm_nextDataMs = 0;
static bool _vm_pcActive = false;
static unsigned long _vm_lastSetMs = 0;                // becomes true when a PC SET is received
static String _vm_line;                          // tiny line buffer

//acknowledge a host SET command with the current millis() timestamp.
static void _vm_send_ack(int p_int) { 
  Serial.print(F("ACK SET P=")); Serial.print(p_int);       
  Serial.print(F(" T="));        Serial.println(millis());
}

// Expect exactly: "SET P=80.0" (whitespace-insensitive on either side)
// Parse a host line (e.g., 'SET P=…'), update target pressure, sync encoder, force RUN, and mark PC control active.
static void _vm_handle_line(const String& s) {
  if (s.startsWith("SET")) {
    int idx = s.indexOf('P');
    if (idx >= 0) {
      int eq = s.indexOf('=', idx);
      if (eq > 0) {
        float p = s.substring(eq + 1).toFloat();
        int p_int = (int)(p + 0.5f);
        sel_pressure = p_int;            // Update the existing target variable your sketch already uses:
        encoderPos = sel_pressure;       // keep the front-panel encoder in sync with PC commands
        _vm_pcActive = true;
        mode = 1;                     // Ensure device is in Motor RUN state when commanded from PC
        runStateMoto = 1;
        startMillis = millis();          // PC is actively driving now
        _vm_lastSetMs = millis();
        _vm_send_ack(p_int);
      }
    }
  }
}

void setup() {
  _vm_nextDataMs = millis();    // Init VasoTracker telemetry scheduler
  Serial.begin(115200);
  ads.begin();
  ads.setGain(GAIN_ONE);
  pinMode(enSW, INPUT_PULLUP);
  pinMode(pinCS, INPUT_PULLUP);
  pinMode(pinDC, INPUT_PULLUP);
  attachInterrupt(pinDC, fpinDC, CHANGE);  //Sets interrupt for rotary encoder so that it works with above functions;
  attachInterrupt(pinCS, fpinCS, CHANGE);
  maskA = digitalPinToBitMask(pinDC);
  maskB = digitalPinToBitMask(pinCS);
  maskAB = maskA | maskB;
  port = portInputRegister(digitalPinToPort(pinDC));
  stepper.begin();
  tft.initR(INITR_GREENTAB);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(false);
  
  calib = calibrate.read();
  
  loadingFromFlash();

  bootup();
  chooseMode();
  calibration();
  ads.linearCal(pLowADC, pHiADC, pLowSel, pHiSel);
  // ads.linearCal(tLowADC, tHiADC, tLowSel, tHiSel); //this is solely for using second transducer. 
  if(mode == 1) {
    stepper.setStepFrac(8);
    delay(500);
  }
  if(mode == 2) {
    stepper.setStepFrac(8);
    stepper.setStepFracSpeed(8, stepsPerSec);
    bootRamp();
    delay(500);
  }
  if(mode == 3) {
    stepper.setStepFrac(8);
    stepper.setStepFracSpeed(8, stepsPerSec);
    bootSim();
    testType();
    delay(500);
  }
  while (fillLines == true) {
    lineFilling();
    tft.fillScreen(ST7735_BLACK);
    initScreenSim();
    printWords(9, 1, 102, 39, 0xfb2c, "Done");
    printNumber(2, 110, 47, 0xfb2c, ST77XX_BLACK, minmmHg);
    printNumber(2, 110, 67, 0xfb2c, ST77XX_BLACK, maxmmHg);
    printNumber(2, 110, 87, 0xfb2c, ST77XX_BLACK, pulseRate);
    testType();
  }
  clickBegin();
  stepper.setStepFracSpeed(8, stepsPerSec);
  writingToFlash();
  calibrate.write(calib);
  delay(500);
}

void loop() {
  currentMillis = millis();
  //parse incoming PC commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (_vm_line.length() > 0) {
        _vm_handle_line(_vm_line);
        _vm_line = "";
      }
    } 
    else {
      if (_vm_line.length() < 120) {
        _vm_line += c;
      }
    }
  }
  // Front-panel takeover if PC inactive for longer than the grace period.
  if (_vm_pcActive) {
    if (currentMillis - _vm_lastSetMs > VM_PC_TIMEOUT_MS) {
      _vm_pcActive = false;
    }
  }
  if(mode == 2) {
    if (runStateRamp == 0) {
      isStoppingRamp();
    } else if (runStateRamp == 1) {
      isRunningRamp();                 //runs as pulse simulator
    } else if (runStateRamp == 2) {
      isPausedRamp();                  //pauses pulse simulator
    }
  }
  else if(mode == 3) {
    if (runStateSim == 0) {
      isStoppingSim();
    } else if (runStateSim == 1) {
      isRunningSim();                 //runs as pulse simulator
    } else if (runStateSim == 2) {
      isPausedSim();                  //pauses pulse simulator
    }
  }
  else if(mode == 1) {
    if (runStateMoto == 0) {
      isStoppingMoto();               //runs as pressure controller
    } else if (runStateMoto == 1) {
      isRunningMoto();
    }
  }
}

//************************************************************************************************************************//
// Load configuration/calibration from non-volatile memory at calib.
void loadingFromFlash() {
  timeDelay = calib._timeDelay;
  filterWeight = calib._filterWeight;
  multiplier = calib._multiplier;
  acceleration = calib._acceleration;
  numSamples = calib._numSamples;
  pLowSel = calib._pLowSel;
  pHiSel = calib._pHiSel;
  tLowSel = calib._tLowSel;
  tHiSel = calib._tHiSel;
  pLowADC = calib._pLowADC;
  pHiADC = calib._pHiADC;
  tLowADC = calib._tLowADC;
  tHiADC = calib._tHiADC;
  lowmmHg = calib._lowmmHg;
  highmmHg = calib._highmmHg;
  pressRate = calib._pressRate;
  minmmHg = calib._minmmHg;
  maxmmHg = calib._maxmmHg;
  pulseRate = calib._pulseRate;
}


// Write configuration/calibration to non-volatile memory.
void writingToFlash() {
  calib._timeDelay = timeDelay;
  calib._filterWeight = filterWeight;
  calib._multiplier = multiplier;
  calib._acceleration = acceleration;
  calib._numSamples = numSamples;
  calib._pLowSel = pLowSel;
  calib._pHiSel = pHiSel;
  calib._tLowSel = tLowSel;
  calib._tHiSel = tHiSel;
  calib._pLowADC = pLowADC;
  calib._pHiADC = pHiADC;
  calib._tLowADC = tLowADC;
  calib._tHiADC = tHiADC;
  calib._lowmmHg = lowmmHg;
  calib._highmmHg = highmmHg;
  calib._pressRate = pressRate;
  calib._minmmHg = minmmHg;
  calib._maxmmHg = maxmmHg;
  calib._pulseRate = pulseRate;
}

//Extra setup functions.
void advancedSettings() {
  tft.fillScreen(ST7735_BLACK);
  initScreenAdv();
  selectNumSamples();
  selectTimeDelay();
  selectFilterWeight();
  selectMultiplier();
  selectAcceleration();
  calib.valid_init = true;
  delay(100);
  bootup();
}

// Acquire and filter pressure ADC samples, producing a smoothed 'avgPressure' reading.
void averagingPressure(int q) {
  // avgPressure = ads.measure(txdx1);
  float Pressure = ads.measure(txdx1);
  for (int i = 0; i < q; i++) {
    avgPressure = avgPressure + (Pressure - avgPressure) / filterWeight;
  }
}

// Acquire and filter tension ADC samples, producing a smoothed 'avgTension' reading.
void averagingTension(int q) {
  float Tension = ads.measure(txdx2);
  for (int i = 0; i < q; i++) {
    avgTension = avgTension + (Tension - avgTension) / filterWeight;
  }
}

//Load image for splash screen.
void bootup() {
  int h = 128, w = 160, row, col, buffidx = 0;
  for (row = 0; row < h; row++) {
    for (col = 0; col < w; col++) {
      tft.drawPixel(col, row, pgm_read_word(logo + buffidx));
      buffidx++;
    }
  }
  printWords(0, 1, 80, 120, ST77XX_RED, "v4.1.4");
    if (calib.valid_init == false) {
    delay(1000);
    tft.fillRect(0, 100, 160, 28, ST77XX_BLACK);
    printWords(8, 1, 1, 111, ST77XX_YELLOW, "First Complete Setup");
    delay(1000);
    numSamples = 10;
    timeDelay = 50;
    multiplier = 5;
    filterWeight = 3;
    acceleration = 50;
    advancedSettings();  
  }
}

void bootRamp() {
  if (calib.valid_ramp == false) {
    lowmmHg = 20;
    highmmHg = 120;
    pressRate = 300;
  }
    rampSetup();
}

//setting a reasonable starting pressure and pulse for simulator.
void bootSim() {
  if (calib.valid_sim == false) {
    minmmHg = 20;
    maxmmHg = 120;
    pulseRate = 100;
  }
    simSetup();
}

// Interactive calibration routine for sensors; guides through points and stores coefficients.
void calibration() {
  const char *calMenu[] = { "Load", "New", "N/A" };
  encoderPos = 0;
  if(mode == 1) {
    initScreenMoto();
  }
  else if(mode == 2) {
    initScreenRamp();
  }
  else if(mode == 3) {
    initScreenSim();
  }
  while (digitalRead(enSW)) {
    choice = encoderPos;
    encoderLimit(0, 2);
    listBox(89, 27, 72, 15, ST77XX_BLACK);
    printWords(9, 1, 90, 39, ST77XX_WHITE, calMenu[choice]);
  }
  while (digitalRead(enSW) == 0) {
    printWords(9, 1, 90, 39, 0xfb2c, calMenu[choice]);
  }
  if (choice == 0) {
    sprintf(debugging, "%d;%d;%0.1f;%d;%d;%0.1f;%0.1f;%d;%d;%d;%d;%d;%d;%d;%d", timeDelay,filterWeight,multiplier,acceleration,numSamples,pLowSel,pHiSel, pLowADC, pHiADC,lowmmHg, highmmHg, pressRate, minmmHg, maxmmHg, pulseRate); Serial.println(debugging);
    if (calib.valid_cal == true) {
      calibrationOrder = 0;
      calWords();
      calNums();
      // delay(200);
      offsetPressure();
      // calibrationOrder = 1;
      // calWords();
      // calNums();
      // offsetTension();     //add or remove, depending on # of sensors
      tft.fillScreen(ST77XX_BLACK);
      if (mode == 1) {
        initScreenMoto();
        printWords(9, 1, 102, 39, 0xfb2c, "Done");
      }
      else if (mode == 2) {
        initScreenRamp();
        printWords(9, 1, 102, 39, 0xfb2c, "Done");
      }
      else if (mode == 3) {
        initScreenSim();
        printWords(9, 1, 102, 39, 0xfb2c, "Done");
      }
    }
    if (calib.valid_cal == false) {
      calibration();
    }
  }
  if (choice == 1) {
    delay(250);
    calibrationOrder = 0;    //add or remove, depending on # of sensors
    calWords();
    PressureADCLow();
    PressureSelLow();
    PressureADCHigh();
    PressureSelHigh();
    delay(200);
    offsetPressure();
    // calibrationOrder = 1;    //add or remove, depending on # of sensors
    // calWords();              //add or remove, depending on # of sensors
    // TensionADCLow();         //add or remove, depending on # of sensors
    // TensionSelLow();         //add or remove, depending on # of sensors
    // TensionADCHigh();        //add or remove, depending on # of sensors
    // TensionSelHigh();        //add or remove, depending on # of sensors
    // offsetTension();         //add or remove, depending on # of sensors
    tft.fillScreen(ST77XX_BLACK);
    calib.valid_cal = true;
    if (mode == 1) {
      initScreenMoto();
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(200);
    }
    else if(mode == 2) {
      initScreenRamp();
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(200);
    }
    else if(mode == 3) {
      initScreenSim();
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(200);
    }
  }
  if (choice == 2) {
    // tft.fillScreen(ST77XX_BLACK);
    calibrationOrder = 0;
    pLowADC = 487; //261 or 711 for andres
    pLowSel = 0;
    pHiADC =  20638; //7930 or 7877 for andres
    pHiSel = 200; //82 or 78 for andres
    calWords();
    calNums();
    offsetPressure();
    // calibrationOrder = 1;
    // tLowADC = 0;
    // tLowSel = 0;
    // tHiADC = 0;
    // tHiSel = 0;
    // calWords();
    // calNums();
    // offsetTension();   //add or remove, depending on # of sensors
    tft.fillScreen(ST77XX_BLACK);
    if (mode == 1) {
      initScreenMoto();
      calib.valid_cal = true;
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(200);
    }
    else if(mode == 2) {
      initScreenRamp();
      calib.valid_cal = true;
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
    }
    else if(mode == 3) {
      initScreenSim();
      calib.valid_cal = true;
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
    }
  }
  writingToFlash();
  // calibrate.write(calib);
  delay(200);
}

// Calibration numeric prompt helper: capture/display numbers.
void calNums() {
  if (calibrationOrder == 0) {
    printCalNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, pLowADC, 6);
    printCalNumber(2, 90, 46, ST77XX_WHITE, ST77XX_BLACK, pLowSel, 6);
    printCalNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, pHiADC, 6);
    printCalNumber(2, 90, 86, ST77XX_WHITE, ST77XX_BLACK, pHiSel, 6);
  }
  else if (calibrationOrder == 1) {
    printCalNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, tLowADC, 6);
    printCalNumber(2, 90, 46, ST77XX_WHITE, ST77XX_BLACK, tLowSel, 6);
    printCalNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, tHiADC, 6);
    printCalNumber(2, 90, 86, ST77XX_WHITE, ST77XX_BLACK, tHiSel, 6);
  }
}

// Calibration text prompt helper: capture/display strings.
void calWords() {
  tft.fillScreen(ST77XX_BLACK);
  if (calibrationOrder == 0) {
    printWords(9, 0, 2, 19, 0x64df, "Calibrate Pressure");
  }
  else if (calibrationOrder == 1) {
    printWords(9, 1, 2, 19, 0x64df, "Calibrate Tension");
  }
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Min ADC:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Sel Min:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max ADC:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Sel Max:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Offset:");
}

// Mode selection UI: lets user pick RUN/SIM/ADV paths.
void chooseMode() {
  const char *modeMenu[] = {"Turn to Select Mode","Pressure Control","Pressure Ramp","Pulse Simulator","Advanced Settings" };
  encoderPos = 0;
  while (digitalRead(enSW)) {
    encoderLimit(0, 4);
    listBox(0, 100, 160, 28, ST77XX_BLACK);
    printWords(8, 1, 7, 111, ST77XX_WHITE, modeMenu[encoderPos]);  
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      bootup();
      chooseMode();
    }
    if (switchChoice == 1) {
      printWords(8, 1, 7, 111, 0xfb2c, modeMenu[encoderPos]);  
      mode = 1;
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 2) {
      printWords(8, 1, 7, 111, 0xfb2c, modeMenu[encoderPos]);  
      mode = 2;
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 3) {
      printWords(8, 1, 7, 111, 0xfb2c, modeMenu[encoderPos]);  
      mode = 3;
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 4) {
      printWords(8, 1, 7, 111, 0xfb2c, modeMenu[encoderPos]);  
      delay(500);
      advancedSettings();
      tft.fillScreen(ST77XX_BLACK);
    }
  }
}

void clickBegin () {
  printWords(8, 1, 30, 121, 0x64df, "Click to Begin");
  while (digitalRead(enSW)) {
  }
  while (digitalRead(enSW) == 0) {
    encoderPos = 0;
    tft.fillScreen(ST7735_BLACK);
    if (mode == 2 && runStateRamp == 0) {
      RampScreen(ST77XX_RED, "STOPPED");
    }
  }
}

// Purpose: High-level behavior of `drawColorBar`.
void drawColorBar(int ctrl, int spotx, int spoty, int height, int pix) {
  int value = abs(ctrl);
  for (int i = 0; i < 33; i++) {
    if (i <= value) {
      tft.fillRect(spotx + (i * pix), spoty, pix / 2, height, rainbow[i]);
    }
    else {
      tft.fillRect(spotx + (i * pix), spoty, pix / 2, height, ST77XX_BLACK);
    }
  }
}

// Easy way to keep encoder from going too far in menus.
void encoderLimit(int min, int max) {
  if (encoderPos < min) { 
    encoderPos = min; 
  }
  if (encoderPos > max) {
    encoderPos = max;
  }
}

//for filling. fast.
void fillFast() {
  int stepDelay = 800;
  stepper.setStepFrac(8);
  float filler;
  filler = (encoderPos - avgPressure);
  if (filler <= -1) {
    stepper.move(1, stepDelay, 1);
  }
  else if (filler >= 1) {
    stepper.move(1, stepDelay, -1);
  } 
  else {
    stepper.move(0, stepDelay, -1);
  }
}

void initScreenAccel() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calibrate:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Min mmHg:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max mmHg:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Rate:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Accel:");
}

void initScreenAdjust() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calibrate:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Min mmHg:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max mmHg:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Rate:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Multiplier:");
}

void initScreenMoto() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calibrate:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
}

void initScreenRamp() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calibrate:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Min mmHg:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max mmHg:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "mmHg/min:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
}

void initScreenSim() {
  printWords(9, 1, 30, 19, 0x64df, "Initialization");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Calibrate:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Min mmHg:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Max mmHg:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "BPM:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Type:");
}

void initScreenAdv() {
  printWords(9, 1, 6, 19, 0x64df, "Advanced Setup");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "Samples:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Delay (ms):");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Avg Weight:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Multiplier:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Accel (ms):");
}

void lineFilling() {
  tft.fillScreen(ST77XX_BLACK);
  printWords(9, 1, 2, 16, ST77XX_WHITE, "Actual");
  printWords(9, 1, 2, 55, ST77XX_WHITE, "Select");
  printWords(7, 1, 6, 122, ST77XX_BLUE, "FILLING");
  tft.drawRect(2, 108, 158, 20, ST77XX_BLUE);
  tft.drawFastHLine(0, 30, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 100, 160, ST77XX_CYAN);
  tft.drawFastVLine(1, 95, 5, ST77XX_CYAN);
  tft.drawFastVLine(20, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(40, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(60, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(80, 95, 5, ST77XX_CYAN);
  tft.drawFastVLine(100, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(120, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(140, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(159, 95, 5, ST77XX_CYAN);
  int mmHg = minmmHg;
  if (mmHg > 0) {
    encoderPos = mmHg;
  }
  while (digitalRead(enSW)) {
    stepper.setStepFracSpeed(8, stepsPerSec);
    fillFast();
    if (!_vm_pcActive) { sel_pressure = encoderPos; }
      currentMillis = millis();
    if (currentMillis - previousMillis >= timeDelay) {
      averagingPressure(numSamples);
      sprintf(selected, " %d ", sel_pressure);
      sprintf(pressure, "%.0f ", avgPressure);
      printWords(0, 3, 80, 2, ST77XX_WHITE, pressure);
      printWords(0, 3, 80, 39, ST77XX_WHITE, selected);
      previousMillis = currentMillis;
    }
  }
  while (digitalRead(enSW) == 0) {
  delay(100);
  }
}

//A black box that keeps menus from overlapping in an unreadable way.
void listBox(uint8_t posX, uint8_t posY, uint8_t wide, uint8_t high, uint16_t fontColor) {
  if (box == true) {
    tft.fillRect(posX, posY, wide, high, fontColor);
    box = false;
  }
}

void MotoScreen(uint16_t color, const char *state) {
  tft.drawRect(2, 108, 158, 20, color);
  printWords(7, 1, 6, 122, color, state);
  printWords(7, 1, 2, 16, ST77XX_WHITE, "Pressure");
  printWords(7, 1, 96, 16, ST77XX_WHITE, "Tension");
  printWords(8, 1, 12, 56, 0xfb2c, "Select");
  printWords(8, 1, 2, 74, 0xfb2c, "Pressure:");
  tft.drawFastHLine(0, 20, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 40, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 80, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 100, 160, ST77XX_CYAN);
  tft.drawFastVLine(1, 95, 5, ST77XX_CYAN);
  tft.drawFastVLine(20, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(40, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(60, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(80, 95, 5, ST77XX_CYAN);
  tft.drawFastVLine(100, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(120, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(140, 97, 3, ST77XX_CYAN);
  tft.drawFastVLine(159, 95, 5, ST77XX_CYAN);
  printWords(0, 3, 106, 48, ST77XX_BLUE, selected);  
}

void offsetPressure() {
  float samplesOffset[numSamples];
  float avgSample;
  uint8_t i;
  float tempAvg;
  encoderPos = 0;
  int tempLowADC = pLowADC;
  int tempHighADC = pHiADC;
  float tempLowSel = pLowSel;
  float tempHighSel = pHiSel;
  while (digitalRead(enSW)) {
    tempLowADC = (pLowADC - (encoderPos * 10));
    tempHighADC = (pHiADC - (encoderPos * 10));
    for (i = 0; i < numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, tempLowSel, tempHighSel);
      samplesOffset[i] = ads.measure(txdx1);
      delay(10);
    }
    avgSample = 0;
    for (i = 0; i < numSamples; i++) {
      avgSample += samplesOffset[i];
    }
    avgSample /= numSamples;
    tempAvg = avgSample;
    char tempOffset[10];
    sprintf(tempOffset, "%.1f ", tempAvg);
    printWords(0, 2, 102, 106, ST77XX_WHITE, tempOffset);
  }
  while (digitalRead(enSW) == 0) {
    pLowADC = tempLowADC;
    pHiADC = tempHighADC;
    pLowSel = tempLowSel;
    pHiSel = tempHighSel;
  }
}

void offsetTension() {
  float samplesOffset[numSamples];
  float avgSample;
  uint8_t i;
  float tempAvg;
  encoderPos = 0;
  int tempLowADC = tLowADC;
  int tempHighADC = tHiADC;
  while (digitalRead(enSW)) {
    tempLowADC = (tLowADC - (encoderPos * 10));
    tempHighADC = (tHiADC - (encoderPos * 10));
    for (i = 0; i < numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, tLowSel, tHiSel);
      samplesOffset[i] = ads.measure(txdx2);
      delay(10);
    }
    avgSample = 0;
    for (i = 0; i < numSamples; i++) {
      avgSample += samplesOffset[i];
    }
    avgSample /= numSamples;
    tempAvg = avgSample;
    char tempOffset[10];
    sprintf(tempOffset, "%.1f ", tempAvg);
    printWords(0, 2, 102, 106, ST77XX_WHITE, tempOffset);
  }
  while (digitalRead(enSW) == 0) {
    tLowADC = tempLowADC;
    tHiADC = tempHighADC;
  }
}

void oscillate(float z) {
  stepper.setStepFrac(8);
  float motors;
  motors = (z - avgPressure);
  if (motors <= -2) {
    stepper.move(1, pulseInt, 1);
  } else if (motors >= 2) {
    stepper.move(1, pulseInt, -1);
  } else {  
    stepper.move(0, pulseInt, 1);
  }
}

// Closed-loop control toward 'sel_pressure' using 'avgPressure'; applies clamps/rate limits and drives the actuator.
void pressureControl(int accel) {
  float motors;
  int minDelay = 500;
  currentMicros = millis();                     //I know the variable is named weird. I didnt want to go back and change it again for no reason.
  motors = (sel_pressure - avgPressure);
  if (motors <= -0.3) {
    if (currentMicros - previousMicros >= accel) {
      goDown = maxDelay;
      if (goUp > minDelay){
      x = x + 10;
      goUp = goUp - x;    
    }
    else {
      goUp = minDelay; 
    }
    previousMicros = currentMicros;
    }
    stepper.move(1, goUp, 1);
  }
   else if (motors >= 0.3) {
    if (currentMicros - previousMicros >= accel) {
      goUp = maxDelay;
      if (goDown > minDelay) {
        y = y + 10;
        goDown = goDown - y;
      }
      else {
        goDown = minDelay; 
      }
      previousMicros = currentMicros;
    }
    stepper.move(1, goDown, -1);
  } 
    else {
      goUp = maxDelay;
      goDown = maxDelay;
      x = 1;
      y = 1;
      stepper.move(0, 0, -1);
  }
}

void pressureRamp() {
  unsigned long currentTiming = millis();                     //I know the variable is named weird. I didnt want to go back and change it again for no reason.
  int pressureDelay = (60000 / pressRate);
  if (currentTiming - prevTiming >= pressureDelay) {
    sel_pressure += direction;
    prevTiming = currentTiming;
  }
  if (sel_pressure >= highmmHg) {
    direction = -1;       // Start counting down
  } 
  else if (sel_pressure <= lowmmHg) {
    direction = 1;        // Start counting up
  }
}

void PressureADCHigh() {
  int firstADC = 0;
  int avgHigh;
  uint8_t i;
  int samplesHigh[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesHigh[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgHigh = 0;
    for (i = 0; i < numSamples; i++) {
      avgHigh += samplesHigh[i];
    }
    avgHigh /= numSamples;
    firstADC = avgHigh;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    pHiADC = firstADC;
    delay(200);
  }
}

void PressureADCLow() {
  calWords();
  int firstADC = 0;
  int avgLow;
  uint8_t i;
  int samplesLow[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesLow[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgLow = 0;
    for (i = 0; i < numSamples; i++) {
      avgLow += samplesLow[i];
    }
    avgLow /= numSamples;
    firstADC = avgLow;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    pLowADC = firstADC;
    delay(200);
  }
}

void PressureSelHigh() {
  encoderPos = PRESSURE_CAL_MAX;
  while (digitalRead(enSW)) {
    char buffer[10];
    sprintf(buffer, "%d ", encoderPos);
    printWords(0, 2, 90, 86, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    pHiSel = encoderPos;
    delay(200);
  }
}

void PressureSelLow() {
  encoderPos = PRESSURE_CAL_MIN;
  while (digitalRead(enSW)) {
    char buffer2[10]; 
    sprintf(buffer2, "%d ", encoderPos);
    printWords(0, 2, 90, 46, ST77XX_WHITE, buffer2);
  }
  while (digitalRead(enSW) == 0) {
    pLowSel = encoderPos;
    delay(200);
  }
}

void rampAdjust() {
  initScreenRamp();
  delay(200);
  selectLowPressure();
  delay(200);
  selectHighPressure();
  delay(200);
  selectRampRate();
  delay(200);
  calib.valid_ramp = true;
  encoderPos = 0;
  tft.fillScreen(ST7735_BLACK);
  RampScreen(ST77XX_RED, "STOPPED");
}

void rampSetup() {
  initScreenRamp();
  delay(100);
  selectLowPressure();
  selectHighPressure();
  selectRampRate();
  delay(100);
  calib.valid_ramp = true;
  encoderPos = 0;
}


void simAdjust() {
  initScreenAccel();
  delay(200);
  selectMinPressure();
  delay(200);
  selectMaxPressure();
  delay(200);
  selectSimRate();
  delay(200);
  calib.valid_sim = true;
  encoderPos = 0;
}

void simSetup() {
  initScreenSim();
  delay(100);
  selectMinPressure();
  selectMaxPressure();
  selectSimRate();
  delay(100);
  calib.valid_sim = true;
}

void TensionADCHigh() {
  int firstADC = 0;
  int avgHigh;
  uint8_t i;
  int samplesHigh[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesHigh[i] = ads.readADC_Differential_2_3();
      delay(1);
    }
    avgHigh = 0;
    for (i = 0; i < numSamples; i++) {
      avgHigh += samplesHigh[i];
    }
    avgHigh /= numSamples;
    firstADC = avgHigh;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    tHiADC = firstADC;
    delay(200);
  }
}

void TensionADCLow() {
  calWords();
  int firstADC = 0;
  int avgLow;
  uint8_t i;
  int samplesLow[numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < numSamples; i++) {
      samplesLow[i] = ads.readADC_Differential_2_3();
      delay(1);
    }
    avgLow = 0;
    for (i = 0; i < numSamples; i++) {
      avgLow += samplesLow[i];
    }
    avgLow /= numSamples;
    firstADC = avgLow;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    tLowADC = firstADC;
    delay(200);
  }
}

void TensionSelHigh() {
  encoderPos = TENSION_CAL_MAX;
  while (digitalRead(enSW)) {
    char buffer[10];
    sprintf(buffer, "%d ", encoderPos);
    printWords(0, 2, 90, 86, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    tHiSel = encoderPos;
    delay(200);
  }
}

void TensionSelLow() {
  encoderPos = TENSION_CAL_MIN;
  while (digitalRead(enSW)) {
    char buffer[10]; 
    sprintf(buffer, "%d ", encoderPos);
    printWords(0, 2, 90, 46, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    tLowSel = encoderPos;
    delay(200);
  }
}

void printHeader() {
  Serial.println("----------------------------------------");
  sprintf(pressureBufferLCA, "Pressure ADC Min: %i | ", pLowADC); Serial.print(pressureBufferLCA); 
  // sprintf(tensionBufferLCA, "Tension ADC Min: %i", tLowADC); Serial.println(tensionBufferLCA);
  sprintf(pressureBufferHCA, "Pressure ADC Max: %i | ", pHiADC); Serial.print(pressureBufferHCA);
  // sprintf(tensionBufferHCA, "Tension ADC Max: %i", tHiADC); Serial.println(tensionBufferHCA);
  sprintf(pressureBufferLCS, "Pressure Cal Min: %.0f mmHg | ", pLowSel); Serial.print(pressureBufferLCS);
  // sprintf(tensionBufferLCS, "Tension Cal Min: %.0f uN", tLowSel); Serial.println(tensionBufferLCS);
  sprintf(pressureBufferHCS, "Pressure Cal Max: %.0f mmHg | ", pHiSel); Serial.print(pressureBufferHCS);
  // sprintf(pressureBufferHCS, "Tension Cal Max: %.0f uN", tHiSel); Serial.println(tensionBufferHCS);
  sprintf(bufferLP, "Min Pressure: %d mmHg", minmmHg); Serial.println(bufferLP);
  sprintf(bufferHP, "Max Pressure: %d mmHg", maxmmHg); Serial.println(bufferHP);
  sprintf(bufferPR, "Ramp Rate: %d mmHg/min", pulseRate); Serial.println(bufferPR);
  Serial.println("----------------------------------------");
}

void printNumber(int fontSize, int posX, int posY, uint16_t fontColor, uint16_t fontBkg, double num) {
  tft.setFont();
  tft.setTextSize(fontSize);
  tft.setTextColor(fontColor, fontBkg);
  tft.setCursor(posX, posY);
  dtostrf(num, 6, 1, number);
  tft.print(number);
}

void printCalNumber(int fontSize, int posX, int posY, uint16_t fontColor, uint16_t fontBkg, double num, int width) {
  tft.setFont();
  tft.setTextSize(fontSize);
  tft.setTextColor(fontColor, fontBkg);
  tft.setCursor(posX, posY);
  dtostrf(num, width, 0, number);
  tft.print(number);
}

void printWords(byte font, int fontSize, int posX, int posY, uint16_t fontColor, const char *words) {
  if (font == 9) {
    tft.setFont(&FreeSansBold9pt7b);
  } else if (font == 7) {
    tft.setFont(&FreeSansBold7pt7b);
  } else if (font == 8) {
    tft.setFont(&FreeSansBold8pt7b);
  } else if (font == 0) {
    tft.setFont();
  }
  tft.setTextSize(fontSize);
  tft.setCursor(posX, posY);
  tft.setTextColor(fontColor, ST77XX_BLACK);
  tft.print(words);
}

void selectAcceleration() {
  encoderPos = acceleration;
  char buffer[3];
  int accelCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    accelCounter = encoderPos;
    sprintf(buffer, " %3d", accelCounter);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    acceleration = accelCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectAccelAdjust() {
  int tempAccel = acceleration;
  encoderPos = 0;
  char buffer[3];
  int accelCounter;
  while (digitalRead(enSW)) {
    accelCounter = tempAccel + encoderPos;
    sprintf(buffer, "%3d", accelCounter);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    acceleration = accelCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectFilterWeight() {
  encoderPos = (filterWeight / 3);
  char buffer[3];
  int filterCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    filterCounter = (encoderPos * 3);
    sprintf(buffer, " %3d", filterCounter);
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    filterWeight = filterCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectMaxPressure() {
  encoderPos = 0;
  char buffer[3];
  int maxCounter;
  while (digitalRead(enSW)) {
    maxCounter = maxmmHg + encoderPos;
    sprintf(buffer, " %3d", maxCounter);
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    maxmmHg = maxCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectMinPressure() {
  encoderPos = 0;
  char buffer[3];
  int minCounter;
  while (digitalRead(enSW)) {
    minCounter = minmmHg + encoderPos;
    sprintf(buffer, " %3d", minCounter);
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    minmmHg = minCounter;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
  }
}

void selectHighPressure() {
  encoderPos = 0;
  char buffer[3];
  int maxCounter;
  while (digitalRead(enSW)) {
    maxCounter = highmmHg + encoderPos;
    sprintf(buffer, " %3d", maxCounter);
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    highmmHg = maxCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectLowPressure() {
  encoderPos = 0;
  char buffer[3];
  int minCounter;
  while (digitalRead(enSW)) {
    minCounter = lowmmHg + encoderPos;
    sprintf(buffer, " %3d", minCounter);
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    lowmmHg = minCounter;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
  }
}


void selectMultiplier() {
  float tempMult = multiplier;
  encoderPos = 0;
  char buffer[3];
  float multCounter;
  while (digitalRead(enSW)) {
    multCounter = tempMult + encoderPos;
    sprintf(buffer, "%.0f ", multCounter);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    multiplier = multCounter;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void simMultiplierAdjust() {
  float tempMult = multiplier;
  encoderPos = 0;
  char buffer[3];
  float multCounter;
  while (digitalRead(enSW)) {
    multCounter = tempMult + encoderPos;
    sprintf(buffer, "%.0f ", multCounter);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    multiplier = multCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectNumSamples() {
  encoderPos = numSamples;
  char buffer[3];
  int sampleCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    sampleCounter = encoderPos;
    sprintf(buffer, " %3d", sampleCounter);
    printWords(0, 2, 110, 27, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    numSamples = sampleCounter;
    printWords(0, 2, 110, 27, 0xfb2c, buffer);
  }
}

void selectRampRate() {
  encoderPos = pressRate;
  char buffer[3];
  int steps;
  while (digitalRead(enSW)) {
    steps = encoderPos;
    if (steps >= 600) {
      steps = 600;
    }
    sprintf(buffer, " %3d", steps);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer); 
  }
  while (digitalRead(enSW) == 0) {
    pressRate = steps;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void selectSimRate() {
  encoderPos = pulseRate;
  char buffer[3];
  int steps;
  while (digitalRead(enSW)) {
    steps = encoderPos;
    if (steps >= 600) {
      steps = 600;
    }
    sprintf(buffer, " %3d", steps);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer); 
  }
  while (digitalRead(enSW) == 0) {
    pulseRate = steps;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void selectTimeDelay() {
  encoderPos = (timeDelay / 5);
  char buffer[4];
  int delayCounter;
  while (digitalRead(enSW)) {
    if (encoderPos < 1) {
      encoderPos = 1;
    }
    delayCounter = (encoderPos * 5);
    sprintf(buffer, "% 4d", delayCounter);
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    timeDelay = delayCounter;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
  }
}

void SimScreen(uint16_t color, const char *state) {
  tft.drawRect(2, 108, 158, 20, color);
  printWords(7, 1, 6, 122, color, state);
  printWords(7, 1, 2, 16, ST77XX_WHITE, "Actual");
  printWords(7, 1, 96, 16, ST77XX_WHITE, "Select");
  printWords(8, 1, 1, 54, 0xfb2c, "Range:");
  printWords(8, 1, 1, 75, 0xfb2c, "Rate:");
  printWords(8, 1, 1, 95, 0xfb2c, "Time:");
  tft.drawFastHLine(0, 20, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 41, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 61, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 81, 160, ST77XX_WHITE);
}

void RampScreen(uint16_t color, const char *state) {
  tft.drawRect(2, 108, 158, 20, color);
  printWords(7, 1, 6, 122, color, state);
  printWords(7, 1, 2, 16, ST77XX_WHITE, "Actual");
  printWords(7, 1, 96, 16, ST77XX_WHITE, "Select");
  printWords(8, 1, 1, 54, 0xfb2c, "Range:");
  printWords(8, 1, 1, 75, 0xfb2c, "mmHg/min:");
  printWords(8, 1, 1, 95, 0xfb2c, "Time:");
  tft.drawFastHLine(0, 20, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 41, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 61, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 81, 160, ST77XX_WHITE);
}

void testType() {
  encoderPos = 0;
  const char *testMenu[] = {"line Fill","Triangle", "Reset?" };
  while (digitalRead(enSW)) {
    encoderLimit(0,2);
    listBox(69, 105, 90, 23, ST77XX_BLACK);
    printWords(9, 1, 70, 118, ST77XX_WHITE, testMenu[encoderPos]);
  }
  while (digitalRead(enSW) == 0) {
    expType = encoderPos;
    printWords(9, 1, 70, 118, 0xfb2c, testMenu[encoderPos]);
  }
  if (expType == 0) {
    fillLines = true;
    Serial.println("Line Filling");
    delay(100);
  }
  if (expType == 1) {
    fillLines = false;
    Serial.println("Triangular Pulse");
    tft.fillScreen(ST7735_BLACK);
    SimScreen(ST77XX_ORANGE, "PAUSED");
    runStateSim = 2;
    runStateMoto = 0;
    UseStartTime = true;
    encoderPos = 0;
    delay(100);
  }
  if (expType == 2) {
      NVIC_SystemReset();
  }
}

void triangle() {
  currmillis = millis();
  beatTime = 30000 / pulseRate;
  if (currmillis - prevmillis > beatTime) {
    beatDir = !beatDir;
    pulseCounter++;
    tempRate = currmillis - prevmillis;
    prevmillis = currmillis;
  }
  if (beatDir == true) {
    oscillate(minmmHg);
  } else if (beatDir == false) {
    oscillate(maxmmHg);
  }
  actualRate = 30000 / tempRate;
}

/******************************** Core Functions ********************************/

// RUN path UI/logic: handle manual input (when allowed), update setpoint, and execute control each cycle.
void isRunningMoto() {
  if (!_vm_pcActive) {
    sel_pressure = encoderPos; 
  }
  currentMillis = millis() - startMillis;
  pressureControl(acceleration);
  if (currentMillis - previousMillis >= timeDelay) {
    currentTime = (currentMillis / 1000.00);
    averagingPressure(numSamples);
    averagingTension(numSamples);
    sprintf(pressure, "%.1f  ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    sprintf(time, "%.2f", currentTime);
    // printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(RunningOutputMoto, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputMoto);
    sprintf(selected, "%d ", sel_pressure);
    printWords(0, 3, 106, 48, ST77XX_BLUE, selected);  
    coloring = ((encoderPos - avgPressure) * 0.5);
    drawColorBar(coloring, 0, 84, 8, 5);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    runStateMoto = 0;
    encoderPos = 0;
    tft.fillScreen(ST77XX_BLACK);
  }
}

void isStoppingMoto() {
  const char *stopMenu[] = { "UNPAUSE", "RESET?" };
  encoderLimit(0, 1);
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  sprintf(selected, "%d ", sel_pressure);
  MotoScreen(ST77XX_RED, "STOPPED");
  if (currentMillis - previousMillis >= timeDelay) {
    averagingPressure(numSamples);
    averagingTension(numSamples);
    sprintf(pressure, "%.1f  ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(StoppingOutputMoto, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputMoto);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      runStateMoto = 1;
      tft.fillScreen(ST7735_BLACK);
      MotoScreen(ST77XX_GREEN, "RUNNING");
      encoderPos = sel_pressure;
      goUp = maxDelay;
      goDown = maxDelay;
    }
    if (switchChoice == 1) {
      NVIC_SystemReset();
    }
  }
}

// SIM pause path: allow safe parameter tweaks while simulation is paused; manual writes gated if PC active.
void isPausedRamp() {
  if (!_vm_pcActive) { 
    sel_pressure = encoderPos; 
  }
  currentMillis = millis() - startMillis;
  pressureControl(acceleration);
  if (currentMillis - previousMillis >= timeDelay) {
    // currentTime = (currentMillis / 1000.00);
    averagingPressure(numSamples);
    // averagingTension(numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(selected, "%d ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    sprintf(range, "%d/%d", lowmmHg, highmmHg);
    dtostrf(pressRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    sprintf(RunningOutputRamp, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputRamp);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
      runStateRamp = 1;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      RampScreen(ST77XX_GREEN, "RUNNING");
      prevmillis = currentMillis;
      UseStartTime = true;
  }
}

void isRunningRamp() {
  pressureRamp();
  pressureControl(acceleration);
  if (UseStartTime == true) {
    startMillis = millis();
    UseStartTime = false;
  }
  currentMillis = millis() - startMillis;
    if (currentMillis - previousMillis >= timeDelay) {
    currentTime = (currentMillis / 1000.00);
    averagingPressure(numSamples);
    // averagingTension(numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(selected, "%d ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    sprintf(range, "%d/%d", lowmmHg, highmmHg);
    dtostrf(pressRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    sprintf(time, "%.2f", currentTime);
    printWords(0, 2, 70, 84, ST77XX_WHITE, time);
    sprintf(RunningOutputRamp, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputRamp);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    runStateRamp = 0;
    encoderPos = 0;
    tft.fillScreen(ST77XX_BLACK);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    RampScreen(ST77XX_RED, "STOPPED");
  }
}

void isStoppingRamp() {
  const char *stopMenu[] = { "PAUSE", "ADJUST", "RESET?" };
  encoderLimit(0, 2);
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  SimScreen(ST77XX_RED, "STOPPED");
  if (currentMillis - previousMillis >= timeDelay) {
    averagingPressure(numSamples);
    averagingTension(numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(selected, "%d ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    sprintf(range, "%d/%d", lowmmHg, highmmHg);
    dtostrf(pressRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
   sprintf(StoppingOutputRamp, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputRamp);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      runStateRamp = 2;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      RampScreen(ST77XX_ORANGE, "PAUSED");
      encoderPos = sel_pressure;
      prevmillis = currentMillis;
    }
    if (switchChoice == 1) {
      tft.fillScreen(ST7735_BLACK);
      delay(100);
      rampAdjust();
      // selectAccelAdjust();
      tft.fillScreen(ST7735_BLACK);
      encoderPos = 0;
    }
    if (switchChoice == 2) {
      writingToFlash();
      delay(200);
      calibrate.write(calib);
      delay(200);
      NVIC_SystemReset();
    }
  }
}

// SIM pause path: allow safe parameter tweaks while simulation is paused; manual writes gated if PC active.
void isPausedSim() {
  currentMillis = millis() - startMillis;
  pressureControl(acceleration);
  if (currentMillis - previousMillis >= timeDelay) {
    // currentTime = (currentMillis / 1000.00);
    averagingPressure(numSamples);
    // averagingTension(numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(selected, "%d ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    sprintf(range, "%d/%d", minmmHg, maxmmHg);
    dtostrf(actualRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    sprintf(RunningOutputSim, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputSim);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
      runStateSim = 1;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      SimScreen(ST77XX_GREEN, "RUNNING");
      prevmillis = currentMillis;
      UseStartTime = true;
  }
}

void isRunningSim() {
  if (expType == 1) {
    triangle();
  }
  if (UseStartTime == true) {
    startMillis = millis();
    UseStartTime = false;
  }
  currentMillis = millis() - startMillis;
  if (currentMillis - previousMillis >= timeDelay) {
    averagingPressure(numSamples);
    // averagingTension(numSamples);
    currentTime = (currentMillis / 1000.00);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    // sprintf(tension, "%.1f  ", avgTension);
    // printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(rate, "%-.2f", pulseRate);
    sprintf(time, "%-.1f", currentTime);
    printWords(0, 2, 80, 64, ST77XX_CYAN, rate);
    printWords(0, 2, 80, 84, ST77XX_CYAN, time);
    sprintf(RunningOutputSim, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputSim);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    runStateSim = 0;
    encoderPos = 0;
    tft.fillScreen(ST77XX_BLACK);
  }
}

void isStoppingSim() {
  int switchChoice;
  const char *stopMenu[] = { "UNPAUSE", "ADJUST", "RESET?" };
  encoderLimit(0, 2);
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  SimScreen(ST77XX_RED, "STOPPING");
  if (currentMillis - previousMillis >= timeDelay) {
    averagingPressure(numSamples);
    // averagingTension(numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(range, "%d/%d", minmmHg, maxmmHg);
    dtostrf(pulseRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 80, 64, ST77XX_CYAN, rate);
    sprintf(StoppingOutputSim, "DATA T=%lu P=%.2f P_SET=%d", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputSim);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    switchChoice = encoderPos;
    if (switchChoice == 0) {
      numSteps = (maxmmHg - minmmHg) * multiplier;
      pulseInt = (60000000 / pulseRate) / (2 * numSteps);
      runStateSim = 1;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      SimScreen(ST77XX_GREEN, "RUNNING");
      printHeader();
      prevmillis = currentMillis;
    }
    if (switchChoice == 1) {
      tft.fillScreen(ST7735_BLACK);
      delay(100);
      simAdjust();
      simMultiplierAdjust();
      tft.fillScreen(ST7735_BLACK);
      encoderPos = 0;
    }
    if (switchChoice == 2) {
      writingToFlash();
      delay(200);
      calibrate.write(calib);
      delay(200); 
      NVIC_SystemReset();
    }
  }
}
