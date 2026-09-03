/*
 VasoMoto v5.0

- All parts are fully functional now. No need to reset to start over; logic state engine overhauled.
- Included menu update in "advanced" to select 1 or 2 transducers. Renamed "txDx1 and "txDx2" to prevent confsion. */

#include <Wire.h>
#include "bitmaps.h"
#include "FlashStorage.h"
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
  #define TFT_RST -1  // Or set to -1 and connect to Arduino RESET pin. This only needs to be 13 when using TensoMoto board v2.0 (May 2023)
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
  const int rainbow[34]{ 0x001F, 0x011F, 0x021F, 0x031F, 0x041F, 0x051F, 0x061F, 0x071F, 0x07FF, 0x07FC, 0x07F8, 0x07F4, 0x07F0, 0x07EC, 0x07E8, 0x07E4, 0x07E0, 0x27E0, 0x47E0, 0x67E0, 0x87E0, 0xA7E0, 0xC7E0, 0xE7E0, 0xFFE0, 0xFF00, 0xFE00, 0xFD00, 0xFC00, 0xFB00, 0xFA00, 0xF900, 0xF800 };

/*Rotary Encoder Setup */
  #define pinDC 10
  #define pinCS 11
  #define enSW 12
  #define exSW 1
  volatile byte aFlag = 0;        //  let's us know when we're expecting a rising edge on pinDT to signal that the encoder has arrived at a detent
  volatile byte bFlag = 0;        //  let's us know when we're expecting a rising edge on pinCLK to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
  volatile int encoderPos;                 //  this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
  volatile uint32_t reading = 0;  //  somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
  bool box;
  bool fillLines = false;
  bool printSettings = true;
  bool beatDir = true;
  bool lastEnSWState = HIGH; // Default to HIGH for INPUT_PULLUP
  bool lastExSWState = HIGH;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

void updateEncoderPolled() {
  static unsigned long lastEncoderRead = 0;
  static uint8_t oldState = 3;
  static int internalCount = 0; // Tracks the 4 micro-steps per physical click
  if (millis() - lastEncoderRead >= 2) {
    lastEncoderRead = millis();
    uint8_t newState = 0;
    if (digitalRead(pinDC)) newState |= 1;
    if (digitalRead(pinCS)) newState |= 2;
    if (newState != oldState) {
      if (oldState == 3 && newState == 1) internalCount--; 
      else if (oldState == 1 && newState == 0) internalCount--; 
      else if (oldState == 0 && newState == 2) internalCount--; 
      else if (oldState == 2 && newState == 3) internalCount--; 
      else if (oldState == 3 && newState == 2) internalCount++; 
      else if (oldState == 2 && newState == 0) internalCount++;
      else if (oldState == 0 && newState == 1) internalCount++;
      else if (oldState == 1 && newState == 3) internalCount++;
      oldState = newState;
      if (internalCount >= 4) {
        encoderPos++;
        internalCount = 0;
        box = !box; 
      } else if (internalCount <= -4) {
        encoderPos--;
        internalCount = 0;
        box = !box;
      }
    }
  }
}

bool isButtonPressed(uint8_t pin, bool &lastState) {
  bool currentState = digitalRead(pin);
  bool triggered = false;
  if (currentState != lastState) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();
      if (currentState == LOW) {
        triggered = true;
      }
      lastState = currentState;
    }
  }
  return triggered;
}

/*Stepper Setup*/
  Stepper stepper(0, 9, 7, 5, 2, 3);
  unsigned long currentMicros;
  unsigned long previousMicros = 0;
  int pulseCounter = 0;
  float tempRate = 0.0;
  float actualRate = 0.0;
  int minDelay = 500;
  int maxDelay = 30000;   //  Used in pressureControl to easily alter how slow the stepper starts, so I dont have to change in 12 places.
  int numSteps = 0;
  int stepsPerSec = 200;  //This is WHOLE steps per sec; the fractionation is accounted for later.

/*Pressure sensor calibration and setup*/
  int outputDelay = 50;               //time for serial output, not reading pressure.
  int txdx1 = 0;                      //identity of first transducer
  int txdx2 = 1;                      //identity of second transducer
  float avgPressure = 0.0;
  float avgTension = 0.0;
  float avgSample = 0.0;
  float PRESSURE_CAL_MIN = 0.00;
  float PRESSURE_CAL_MAX = 80.0;
  float TENSION_CAL_MIN = 0.00;
  float TENSION_CAL_MAX = 100.0;
  int numSamples = 20;
  byte adcState = 0;

/* Flash storage definitions and matrices */
   struct cal_matrix {       //place to store all the values needed for linear calibration of sensors.
    bool redo_init;
    bool valid_cal;
    bool valid_init;
    bool valid_ramp;
    bool valid_sim;
    float pLowSel;
    float pHiSel;
    float tLowSel;
    float tHiSel;
    float multiplier;      //Use this to set the velocity multiplier to get the right shape of the pressure curve.             
    int pLowADC;
    int pHiADC;
    int tLowADC;
    int tHiADC;
    int timeDelay;         //the time delay for wheatstone bridge output.
    int filterWeight;      //the weighting of the averaging filter.
    int acceleration;      //Use this to alter how fast the pump accelerates and to what speed.
    int numTxdx;
    int lowmmHg;
    int highmmHg;
    int pressRate;
    int minmmHg;
    int maxmmHg;
    int pulseRate;
  } calib; 
  
  FlashStorage (calibrate, cal_matrix);

/* Serial receive vairables */
  const byte numChars = 16;
  char receivedChars[numChars];

/* Miscellaneous global variables */
  unsigned long previousMillis = 0;
  unsigned long prevmillis = 0;
  unsigned long currmillis;
  unsigned long startMillis;
  unsigned long currentMillis;
  float currentTime;
  int expType;
    int pulseInt;
  float sel_pressure = 0;
  unsigned long prevTiming;
  float direction = 0.1;
  bool UseStartTime = true;
  int mode;
  // bool moto = false;
  char number[8];
  char selected[8];
  char pressure[16];
  char tension[16];
  char range[8];
  char rate[16];
  char time[16];
  char RunningOutputMoto[64];
  char StoppingOutputMoto[64];
  char RunningOutputSim[64];
  char RunningOutputRamp[64];
  char StoppingOutputRamp[64];
  char StoppingOutputSim[64];
  char debugging[72]; //for ease of reading serial output stuff

/* VasoTracker PC control variables */
  static const uint16_t VM_TELEMETRY_HZ = 25;      // DATA lines per second
  static const unsigned long VM_PC_TIMEOUT_MS = 3000;  // allow a longer grace period for PC control
  static unsigned long _vm_nextDataMs = 0;
  static bool _vm_pcActive = false;
  static unsigned long _vm_lastSetMs = 0;                // becomes true when a PC SET is received
  static String _vm_line;                          // tiny line buffer

/* acknowledge a host SET command with the current millis() timestamp. */
  static void _vm_send_ack(int p_int) { 
    Serial.print(F("ACK SET P=")); Serial.print(p_int);       
    Serial.print(F(" T="));        Serial.println(millis());
  }

/* State Machine */
  enum SystemState {
    SYS_BOOTUP,
    SYS_ADVANCED_SETTINGS,
    SYS_CHOOSE_MODE,
    SYS_CALIBRATION,
    SYS_MODE_SETUP,
    SYS_LINE_FILL,
    SYS_ACTIVE_MODE,
    SYS_SIM_SETUP,
    SYS_RAMP_SETUP    
  };
  enum MotoState { 
    MOTO_STOPPED, 
    MOTO_RUNNING
  };
  enum RampState {
    RAMP_STOPPED,
    RAMP_RUNNING,
    RAMP_PAUSED
  };
  enum RampSetup {
    RAMP_SEL_MIN,
    RAMP_SEL_MAX,
    RAMP_SEL_RATE
  };
  enum SimState {
    SIM_STOPPED,
    SIM_RUNNING,
    SIM_PAUSED
  };
  enum SimSetup {
    SIM_SEL_MIN,
    SIM_SEL_MAX,
    SIM_SEL_RATE
  };
  enum AdvancedState {
    ADV_TXDX,
    ADV_DELAY,
    ADV_WEIGHT,
    ADV_MULTIPLIER,
    ADV_KP
  };

  enum CalibrationState {
    OFFSET_PRESSURE,
    OFFSET_TENSION,
    CAL_PRESSURE,
    CAL_TENSION,
    CAL_NEW_P_LOW_ADC,
    CAL_NEW_P_LOW_SEL,
    CAL_NEW_P_HIGH_ADC,
    CAL_NEW_P_HIGH_SEL,
    CAL_NEW_T_LOW_ADC,
    CAL_NEW_T_LOW_SEL,
    CAL_NEW_T_HIGH_ADC,
    CAL_NEW_T_HIGH_SEL
  };

  SystemState sysState = SYS_BOOTUP;
  AdvancedState advState = ADV_TXDX;
  MotoState runStateMoto = MOTO_STOPPED;
  RampState runStateRamp = RAMP_STOPPED;
  SimState runStateSim = SIM_STOPPED;
  RampSetup setupRamp = RAMP_SEL_MIN;
  SimSetup setupSim = SIM_SEL_MIN;
  CalibrationState setupCal = CAL_PRESSURE;

  bool stateInit = true;

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
          runStateMoto = MOTO_RUNNING;     // Ensure device is in Motor RUN state when commanded from PC
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
  Wire.setClock(400000);
  ads.setGain(GAIN_ONE);
  pinMode(enSW, INPUT_PULLUP);
  pinMode(exSW, INPUT_PULLUP);
  pinMode(pinCS, INPUT_PULLUP);
  pinMode(pinDC, INPUT_PULLUP);
  stepper.begin();
  tft.initR(INITR_GREENTAB);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.setTextWrap(false);
  calib = calibrate.read();
  sysState = SYS_BOOTUP;
  stateInit = true;
}

void loop() {
  telemetry();
  updateEncoderPolled();
  switch (sysState) {
    case SYS_ADVANCED_SETTINGS:
      advancedSettings();
      break;

    case SYS_BOOTUP:
      bootup();
      break;
    
    case SYS_CHOOSE_MODE:
      chooseMode();
      break;

    case SYS_CALIBRATION:
      calibrationSettings();
      break;

    case SYS_MODE_SETUP:
      modeSetup();
      break;
    
    case SYS_RAMP_SETUP:
      bootRamp();
      break;

    case SYS_SIM_SETUP:
      bootSim();
      break;

    case SYS_LINE_FILL:
      lineFilling();
      break;
    
    case SYS_ACTIVE_MODE:
      if(mode == 2) {
        if (runStateMoto == MOTO_STOPPED) { isStoppingMoto(); }
        else if (runStateMoto == MOTO_RUNNING) { isRunningMoto(); }
      }
      else if(mode == 3) {
        if (runStateRamp == RAMP_STOPPED) { isStoppingRamp(); } 
        else if (runStateRamp == RAMP_RUNNING) { isRunningRamp(); }
        else if (runStateRamp == RAMP_PAUSED) { isPausedRamp(); }
      }
      else if(mode == 4) {
        if (runStateSim == SIM_STOPPED) { isStoppingSim(); }
        else if (runStateSim == SIM_RUNNING) { isRunningSim(); }
        else if (runStateSim == SIM_PAUSED) { isPausedSim(); }
      }
      break;
  }
}

//************************************************************************************************************************//

void modeSetup() {
  if(mode == 2) {
    stepper.setStepFrac(16);
    stateInit = true;
    sysState = SYS_ACTIVE_MODE;
  }
  if(mode == 3) {
    stepper.setStepFrac(8);
    if (calib.valid_ramp == false) {
      calib.lowmmHg = 20;
      calib.highmmHg = 120;
      calib.pressRate = 60;
    }
     stateInit = true;
     sysState = SYS_RAMP_SETUP;
  }
  if(mode == 4) {
    stepper.setStepFrac(8);
    if (calib.valid_sim == false) {
      calib.minmmHg = 20;
      calib.maxmmHg = 120;
      calib.pulseRate = 100;
    }
    stateInit = true;
    sysState = SYS_SIM_SETUP;
  }
  ads.linearCal(calib.pLowADC, calib.pHiADC, calib.pLowSel, calib.pHiSel);
  if (calib.numTxdx == 2) {
    ads.linearCal(calib.tLowADC, calib.tHiADC, calib.tLowSel, calib.tHiSel); //this is solely for using second transducer. 
  }
  calibrate.write(calib);
}

void telemetry() {
  currentMillis = millis();
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
  if (_vm_pcActive) {
    if (currentMillis - _vm_lastSetMs > VM_PC_TIMEOUT_MS) {
      _vm_pcActive = false;
    }
  }
}

void calibrationSettings() {
  switch (setupCal) {
    case CAL_PRESSURE:
      calibrationPressure();
      break;

    case CAL_NEW_P_LOW_ADC:
      PressureADCLow();
      break;
    
    case CAL_NEW_P_LOW_SEL:
      PressureSelLow();
      break;
    
    case CAL_NEW_P_HIGH_ADC:
      PressureADCHigh();
      break;
    
    case CAL_NEW_P_HIGH_SEL:
      PressureSelHigh();
      break;
      
    case CAL_TENSION:
      calibrationTension();
      break;

    case CAL_NEW_T_LOW_ADC:
      TensionADCLow();
      break;
    
    case CAL_NEW_T_LOW_SEL:
      TensionSelLow();
      break;
    
    case CAL_NEW_T_HIGH_ADC:
      TensionADCHigh();
      break;
    
    case CAL_NEW_T_HIGH_SEL:
      TensionSelHigh();
      break;
    
    case OFFSET_PRESSURE:
      offsetPressure();
      break;
    
    case OFFSET_TENSION:
      offsetTension();
      break;
  }
}

void advancedSettings() {
  switch (advState) {
  case ADV_TXDX:
    selectNumTxdx();
    break;
  
  case ADV_DELAY:
    selectTimeDelay();
    break;
  
  case ADV_WEIGHT:
    selectFilterWeight();
    break;

  case ADV_MULTIPLIER:
    selectMultiplier();
    break;

  case ADV_KP:
    selectAcceleration();
    break;
  }
}

void updateSensors (int filterWeight) {
  static byte sensorState = 0;
  static unsigned long sensorPrevMillis = 0;
  unsigned long currentMillis = millis();
  switch (sensorState) {
    case 0:
      if (currentMillis - sensorPrevMillis >= calib.timeDelay) {
        ads.requestADC_Differential_0_1();
        sensorState = 1;
      }
    break;
    case 1:
      if (ads.conversionComplete()) {
        float rawPressure = ads.getMeasureResult();
        avgPressure = avgPressure + (rawPressure - avgPressure) / filterWeight;
        ads.requestADC_Differential_2_3();
        sensorState = 2;
      }
    break;
    case 2: 
      if (ads.conversionComplete()) {
        float rawTension = ads.getMeasureResult();
        avgTension = avgTension + (rawTension - avgTension) / filterWeight;  
        sensorPrevMillis = currentMillis;
        sensorState = 0;
      }
    break;
  }
}

void averagingPressure(int q) {
  for (int i = 0; i < q; i++) {
    float Pressure = ads.measure(txdx1);
    avgPressure = avgPressure + (Pressure - avgPressure) / calib.filterWeight;
  }
}

void bootup() {
  int h = 128, w = 160, row, col, buffidx = 0;
  for (row = 0; row < h; row++) {
    for (col = 0; col < w; col++) {
      tft.drawPixel(col, row, pgm_read_word(logo + buffidx));
      buffidx++;
    }
  }
  drawCentreString("v5.0",80,120,ST77XX_GREEN);
  delay(1000);
  if (calib.valid_init == false) {
    delay(1000);
    tft.fillRect(0, 100, 160, 28, ST77XX_BLACK);
    printWords(8, 1, 1, 111, ST77XX_YELLOW, "First Complete Setup");
    delay(1000);
    numSamples = 30;
    calib.timeDelay = 100;
    calib.multiplier = 5;
    calib.filterWeight = 3;
    calib.acceleration = 500;
    sysState = SYS_ADVANCED_SETTINGS;
  }
  else if (calib.valid_init == true) {
    sysState = SYS_CHOOSE_MODE;
  }
}

void bootRamp() {
  switch(setupRamp) {
   case RAMP_SEL_MIN:
  rampMinPressure();
    break;

   case RAMP_SEL_MAX:
    rampMaxPressure();
    break;

   case RAMP_SEL_RATE:
    rampSelectRate();
    break;
  }
}

void bootSim() {
  switch(setupSim) {
   case SIM_SEL_MIN:
    simMinPressure();
    break;

   case SIM_SEL_MAX:
    simMaxPressure();
    break;

   case SIM_SEL_RATE:
    simSelectRate();
    break;
  }
}

void calibrationPressure() {
  calib.valid_init = true;
  static int lastEncoderPos = -1;
  const char *calMenu[] = { "Load", "New", "N/A" };
  encoderLimit(0, 2);
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    initScreenCal();
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  if (encoderPos != lastEncoderPos) {
    listBox(89, 27, 72, 15, ST77XX_BLACK);
    printWords(9, 1, 90, 39, ST77XX_WHITE, calMenu[encoderPos]);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(9, 1, 90, 39, 0xfb2c, calMenu[encoderPos]);
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      if (calib.valid_cal == true) {
        calibrationOrder = 0;
        setupCal = OFFSET_PRESSURE;
      }
      else { 
        calibrationOrder = 0;
        setupCal = CAL_PRESSURE; 
      }
    }
    else if (switchChoice == 1) {
      calibrationOrder = 0;
      setupCal = CAL_NEW_P_LOW_ADC;
    }
    else if (switchChoice == 2) {
      calibrationOrder = 0;
      calib.pLowADC = 536;
      calib.pLowSel = 0;
      calib.pHiADC =  13887;
      calib.pHiSel = 140;
      setupCal = OFFSET_PRESSURE;
    }
    stateInit = true;
  }
}

void calibrationTension() {
  static int lastEncoderPos = -1;
  const char *calMenu[] = { "Load", "New", "N/A" };
  encoderLimit(0, 2);
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    initScreenCal();
    printWords(9, 1, 90, 39, 0xfb2c, "Done");
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  if (encoderPos != lastEncoderPos) {
    listBox(89, 47, 72, 15, ST77XX_BLACK);
    printWords(9, 1, 90, 59, ST77XX_WHITE, calMenu[encoderPos]);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(9, 1, 90, 47, 0xfb2c, calMenu[encoderPos]);
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      if (calib.valid_cal == true) {
        calibrationOrder = 1;
        setupCal = OFFSET_TENSION;
      }
      else { 
        calibrationOrder = 1;
        calibrationTension();
      }
    }
    else if (switchChoice == 1) {
      calibrationOrder = 1;
      setupCal = CAL_NEW_T_LOW_ADC;
    }
    else if (switchChoice == 2) {
      calibrationOrder = 1;
      calib.tLowADC = 1057;
      calib.tLowSel = 0;
      calib.tHiADC =  2654;
      calib.tHiSel = 5*9.81;
      setupCal = OFFSET_TENSION;
    }
    stateInit = true;
  }
}

void calNums() {
  if (calibrationOrder == 0) {
    printCalNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, calib.pLowADC, 6);
    printCalNumber(2, 90, 46, ST77XX_WHITE, ST77XX_BLACK, calib.pLowSel, 6);
    printCalNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, calib.pHiADC, 6);
    printCalNumber(2, 90, 86, ST77XX_WHITE, ST77XX_BLACK, calib.pHiSel, 6);
  }
  else if (calibrationOrder == 1) {
    printCalNumber(2, 90, 26, ST77XX_WHITE, ST77XX_BLACK, calib.tLowADC, 6);
    printCalNumber(2, 90, 46, ST77XX_WHITE, ST77XX_BLACK, calib.tLowSel, 6);
    printCalNumber(2, 90, 66, ST77XX_WHITE, ST77XX_BLACK, calib.tHiADC, 6);
    printCalNumber(2, 90, 86, ST77XX_WHITE, ST77XX_BLACK, calib.tHiSel, 6);
  }
}

void calWords() {
  tft.fillScreen(ST77XX_BLACK);
  if (calibrationOrder == 0) {
    printWords(9, 0, 2, 19, 0x64df, "Calibrate TxDx1");
  }
  else if (calibrationOrder == 1) {
    printWords(9, 1, 2, 19, 0x64df, "Calibrate TxDx2");
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

void drawCentreString(const char *buf, int x, int y, uint16_t fontColor) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(1);
  tft.setFont(&FreeSansBold8pt7b);
  tft.setTextColor(fontColor, ST77XX_BLACK);
  tft.getTextBounds(buf, x, y, &x1, &y1, &w, &h); //calc width of new string
  tft.setCursor(x - w / 2, y);
  tft.print(buf);
}

void chooseMode() {
  const char *modeMenu[] = {"Turn to Select Mode","Line Filling","Pressure Control","Pressure Ramp","Pulse Simulator","Calibration"," Change Advanced","Hard Reset?" };
  static int lastEncoderPos = -1;
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    int h = 128, w = 160, row, col, buffidx = 0;
    for (row = 0; row < h; row++) {
      for (col = 0; col < w; col++) {
        tft.drawPixel(col, row, pgm_read_word(logo + buffidx));
        buffidx++;
      }
    }
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  encoderLimit(0, 7);
  if (encoderPos != lastEncoderPos) {
    listBox(0, 100, 160, 28, ST77XX_BLACK);
    if (encoderPos == 0) {
      drawCentreString(modeMenu[encoderPos],80,111,ST77XX_WHITE);
    }
    else if (encoderPos >= 1 && encoderPos <= 4) {
      drawCentreString(modeMenu[encoderPos],80,111,ST77XX_YELLOW);
    }
    else if (encoderPos >= 5 && encoderPos <= 6) {
      drawCentreString(modeMenu[encoderPos],80,111,ST77XX_GREEN);
    }
    else if (encoderPos == 7) {
      drawCentreString(modeMenu[encoderPos],80,111,ST77XX_RED);
    }
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    drawCentreString(modeMenu[encoderPos],80,111,ST77XX_WHITE);
    if (encoderPos == 0) {
      sysState = SYS_CHOOSE_MODE;
    }
    else if (encoderPos == 1) {
      mode = encoderPos;
      sysState = SYS_LINE_FILL;
    }
    else if (encoderPos >= 2 && encoderPos <= 4) {  
      mode = encoderPos;
      sysState = SYS_MODE_SETUP;
    }
    else if (encoderPos == 5) {
      sysState = SYS_CALIBRATION;
    }
    else if (encoderPos == 6) {
      sysState = SYS_ADVANCED_SETTINGS;
    }
    else if (encoderPos == 7) {
      cal_matrix savedCalib = calibrate.read();
      if (memcmp(&savedCalib, &calib, sizeof(calib)) != 0) {
        calibrate.write(calib);
      }
      NVIC_SystemReset();
    }
  stateInit = true;
  }
}

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

void encoderLimit(int min, int max) {
  if (encoderPos < min) { 
    encoderPos = min; 
  }
  if (encoderPos > max) {
    encoderPos = max;
  }
}

void fillFast() {
  int stepDelay = 1000;
  stepper.setStepFrac(8);
  float filler;
  filler = (encoderPos - avgPressure);
  if (filler <= -1) {
    stepper.move(1, stepDelay, -1);
  }
  else if (filler >= 1) {
    stepper.move(1, stepDelay, 1);
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

void initScreenCal() {
  printWords(9, 1, 30, 19, 0x64df, "Calibration");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "TxDx1:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "TxDx2:");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
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
}

void initScreenAdv() {
  printWords(9, 1, 6, 19, 0x64df, "Advanced Setup");
  tft.drawFastHLine(2, 24, 158, 0xfe31);
  printWords(9, 1, 2, 39, 0x04d3, "#TxDx:");
  tft.drawFastHLine(2, 44, 158, 0xfe31);
  printWords(9, 1, 2, 59, 0x04d3, "Delay (ms):");
  tft.drawFastHLine(2, 64, 158, 0xfe31);
  printWords(9, 1, 2, 79, 0x04d3, "Avg Weight:");
  tft.drawFastHLine(2, 84, 158, 0xfe31);
  printWords(9, 1, 2, 99, 0x04d3, "Multiplier:");
  tft.drawFastHLine(2, 104, 158, 0xfe31);
  printWords(9, 1, 2, 119, 0x04d3, "Kp:");
}

void initLineFilling() {
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
}

void lineFilling() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    ads.linearCal(calib.pLowADC, calib.pHiADC, calib.pLowSel, calib.pHiSel);
    lastEncoderPos = -1;
    initLineFilling();
    encoderPos = 0;
    stateInit = false;
  }
    if (encoderPos != lastEncoderPos) {
      if (!_vm_pcActive) { 
        sel_pressure = encoderPos; 
      }
      lastEncoderPos = encoderPos;
    }
    currentMillis = millis();
    fillFast();
    if (currentMillis - previousMillis >= outputDelay) {
      avgPressure = ads.measure(txdx1);
      sprintf(selected, " %.0f ", sel_pressure);
      sprintf(pressure, "%.0f ", avgPressure);
      printWords(0, 3, 80, 2, ST77XX_WHITE, pressure);
      printWords(0, 3, 80, 39, ST77XX_WHITE, selected);
      previousMillis = currentMillis;
    }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    sysState = SYS_CHOOSE_MODE;
    stateInit = true;
  }
}

void listBox(uint8_t posX, uint8_t posY, uint8_t wide, uint8_t high, uint16_t fontColor) {
  if (box == true) {
    tft.fillRect(posX, posY, wide, high, fontColor);
    box = false;
  }
}

void MotoScreen(uint16_t color, const char *state) {
  tft.drawRect(2, 108, 158, 20, color);
  printWords(7, 1, 6, 122, color, state);
  printWords(7, 1, 2, 16, ST77XX_WHITE, "TxDx1");
  printWords(7, 1, 96, 16, ST77XX_WHITE, "TxDx2");
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
  static int lastEncoderPos = -1;
  float avgSample = 0;
  static int tempLowADC = 0;
  static int tempHighADC = 0;
  if (stateInit) {
    tft.fillScreen(ST7735_BLACK);
    calWords();
    calNums();
    lastEncoderPos = -1;
    encoderPos = 0;
    tempLowADC = calib.pLowADC;
    tempHighADC = calib.pHiADC; 
    stateInit = false;
  }
  char buffer[10];
  if (encoderPos != lastEncoderPos) {
    tempLowADC -= ((encoderPos - lastEncoderPos) * 10);
    tempHighADC -= ((encoderPos - lastEncoderPos) * 10);
    lastEncoderPos = encoderPos;
  }
  if (currentMillis - previousMillis >= calib.timeDelay) {
    for (uint8_t i = 0; i < numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, calib.pLowSel, calib.pHiSel);
      avgSample += ads.measure(txdx1);
    }
    avgSample /= numSamples;
    snprintf(buffer, sizeof(buffer), "%-3.1f  ", avgSample);
    printWords(0, 2, 102, 106, ST77XX_WHITE, buffer);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.pLowADC = tempLowADC;
    calib.pHiADC = tempHighADC;
    if (calib.numTxdx == 1) {
      calib.valid_cal = true;
    }
    if (calib.numTxdx == 2) {
      calibrationOrder = 1;
      setupCal = CAL_TENSION;
    }
    else { 
      sysState = SYS_CHOOSE_MODE;
    }
      stateInit = true;
}
}

void offsetTension() {
  static int lastEncoderPos = -1;
  float avgSample = 0;
  static int tempLowADC = 0;
  static int tempHighADC = 0;
  if (stateInit) {
    tft.fillScreen(ST7735_BLACK);
    calWords();
    calNums();
    lastEncoderPos = -1;
    encoderPos = 0;
    tempLowADC = calib.tLowADC;
    tempHighADC = calib.tHiADC; 
    stateInit = false;
  }
  char buffer[10];
  if (encoderPos != lastEncoderPos) {
    tempLowADC -= (encoderPos - lastEncoderPos);
    tempHighADC -= (encoderPos - lastEncoderPos);
    lastEncoderPos = encoderPos;
  }
  if (currentMillis - previousMillis >= calib.timeDelay) {
    for (uint8_t i = 0; i < numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, calib.tLowSel, calib.tHiSel);
      avgSample += ads.measure(txdx2);
    }
    avgSample /= numSamples;
    snprintf(buffer, sizeof(buffer), "%-3.1f  ", avgSample);
    printWords(0, 2, 102, 106, ST77XX_WHITE, buffer);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.tLowADC = tempLowADC;
    calib.tHiADC = tempHighADC;
    calib.valid_cal = true;
    sysState = SYS_CHOOSE_MODE;
    setupCal = CAL_PRESSURE;
    stateInit = true;
  }
}

void oscillate(float target) {
  stepper.setStepFrac(8);
  float error = target - avgPressure;
  if (error <= -1) {
    stepper.move(1, pulseInt, -1);
  } else if (error >= 1) {
    stepper.move(1, pulseInt, 1);
  } else {  
    stepper.move(0, pulseInt, 0);
  }
}

void pressureControl(int accel) {
  float error = sel_pressure - avgPressure;
  if (abs(error) > 0.5) {
    float calcDelay = maxDelay - (abs(error) * accel);
    int stepDelay = constrain((int)calcDelay, minDelay, maxDelay);
    int dir = (error <= -0.5) ? -1 : 1;
    stepper.move(1, stepDelay, dir);
  } 
  else {
    stepper.move(0, 0, 0);
  }
}

void pressureRamp() {
  uint32_t pressureDelayUs = 6000000 / calib.pressRate;
  if (micros() - prevTiming >= pressureDelayUs) {
    sel_pressure += direction;
    prevTiming = micros();
  }
  if (sel_pressure >= calib.highmmHg) {
    direction = -0.1;
  } 
  else if (sel_pressure <= calib.lowmmHg) {
    direction = 0.1;
  }
}

void PressureADCLow() {
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    calWords();
    encoderPos = 0;
    stateInit = false;
  }
  int avgLow = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    avgLow += ads.readADC_Differential_0_1();
  }
  avgLow /= numSamples;
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%-6d", avgLow);
  printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.pLowADC = avgLow;
    setupCal = CAL_NEW_P_LOW_SEL;
    stateInit = true;
  }
}

void PressureSelLow() {
  if(stateInit) {
    encoderPos = PRESSURE_CAL_MIN;
    stateInit = false;
  }
  char buffer[10]; 
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  printWords(0, 2, 90, 46, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.pLowSel = encoderPos;
    setupCal = CAL_NEW_P_HIGH_ADC;
    stateInit = true;
  }
}

void PressureADCHigh() {
  if (stateInit) {
    encoderPos = 0;
    stateInit = false;
  }
  int avgHigh = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    avgHigh += ads.readADC_Differential_0_1();
  }
  avgHigh /= numSamples;
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%-6d", avgHigh);
  printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.pHiADC = avgHigh;
    setupCal = CAL_NEW_P_HIGH_SEL;
    stateInit = true;
  }
}

void PressureSelHigh() {
    if(stateInit) {
    encoderPos = PRESSURE_CAL_MAX;
    stateInit = false;
  }
  char buffer[10]; 
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  printWords(0, 2, 90, 86, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.pHiSel = encoderPos;
    setupCal = OFFSET_PRESSURE;
    stateInit = true;
  }
}

void TensionADCLow() {
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    calWords();
    encoderPos = 0;
    stateInit = false;
  }
  int avgLow = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    avgLow += ads.readADC_Differential_2_3();
  }
  avgLow /= numSamples;
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%-6d", avgLow);
  printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.tLowADC = avgLow;
    setupCal = CAL_NEW_T_LOW_SEL;
    stateInit = true;
  }
}

void TensionSelLow() {
  if(stateInit) {
    encoderPos = TENSION_CAL_MIN;
    stateInit = false;
  }
  char buffer[10]; 
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  printWords(0, 2, 90, 46, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.tLowSel = encoderPos;
    setupCal = CAL_NEW_T_HIGH_ADC;
    stateInit = true;
  }
}

void TensionADCHigh() {
  if (stateInit) {
    encoderPos = 0;
    stateInit = false;
  }
  int avgHigh = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    avgHigh += ads.readADC_Differential_2_3();
  }
  avgHigh /= numSamples;
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%-6d", avgHigh);
  printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.tHiADC = avgHigh;
    setupCal = CAL_NEW_T_HIGH_SEL;
    stateInit = true;
  }
}

void TensionSelHigh() {
    if(stateInit) {
    encoderPos = TENSION_CAL_MAX;
    stateInit = false;
  }
  char buffer[10]; 
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  printWords(0, 2, 90, 86, ST77XX_WHITE, buffer);
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.tHiSel = encoderPos;
    setupCal = OFFSET_TENSION;
    stateInit = true;
  }
}

void printHeader() {
  Serial.println("----------------------------------------");
  Serial.print(F("Transducer 1 ADC Min: ")); Serial.println(calib.pLowADC);
  Serial.print(F("Transducer 2 ADC Min: ")); Serial.println(calib.tLowADC);
  Serial.print(F("Transducer 1 ADC Max: ")); Serial.println(calib.pHiADC);
  Serial.print(F("Transducer 2 ADC Max: ")); Serial.println(calib.tHiADC);
  Serial.print(F("Pressure Cal Min (mmHg): ")); Serial.println(calib.pLowSel);
  Serial.print(F("Tension Cal Min (µN): ")); Serial.println(calib.tLowSel);
  Serial.print(F("Pressure Cal Max (mmHg): ")); Serial.println(calib.pHiSel);
  Serial.print(F("Tension Cal Max (µN): ")); Serial.println(calib.tHiSel);
  Serial.print(F("Min Pressure (mmHg): ")); Serial.println(calib.minmmHg);
  Serial.print(F("Max Pressure (mmHg): ")); Serial.println(calib.maxmmHg);
  Serial.print(F("Ramp Rate (mmHg/min): ")); Serial.println(calib.pulseRate);
  Serial.println("----------------------------------------");
}

void printCalNumber(int fontSize, int posX, int posY, uint16_t fontColor, uint16_t fontBkg, float num, int width) {
  tft.setFont();
  tft.setTextSize(fontSize);
  tft.setTextColor(fontColor, fontBkg);
  tft.setCursor(posX, posY);
  snprintf(number, sizeof(number), "%-*.0f", width, num);
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

void simMinPressure() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    tft.fillScreen(ST77XX_BLACK);
    initScreenSim();
    encoderPos = calib.minmmHg;
    printWords(9, 1, 90, 39, 0xfb2c, "Done");
    stateInit = false;
  }
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
    calib.minmmHg = encoderPos;
    setupSim = SIM_SEL_MAX;
    stateInit = true;
  }
}

void rampMinPressure() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    tft.fillScreen(ST77XX_BLACK);
    initScreenRamp();
    encoderPos = calib.lowmmHg;
    printWords(9, 1, 90, 39, 0xfb2c, "Done");
    stateInit = false;
  }
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    calib.lowmmHg = encoderPos;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
    setupRamp = RAMP_SEL_MAX;
    stateInit = true;
  }
}

void simMaxPressure() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.maxmmHg;
    stateInit = false;
  }
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
    calib.maxmmHg = encoderPos;
    setupSim = SIM_SEL_RATE;
    stateInit = true;
  }
}

void rampMaxPressure() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.highmmHg;
    stateInit = false;
  }
  char buffer[10];
  int maxCounter;
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
    calib.highmmHg = encoderPos;
    setupRamp = RAMP_SEL_RATE;
    stateInit = true;
  }
}

void rampSelectRate() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.pressRate;
    stateInit = false;
  }
  encoderLimit(0, 600);
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
    calib.pressRate = encoderPos;
    calib.valid_ramp = true;
    setupRamp = RAMP_SEL_MIN;
    runStateRamp = RAMP_STOPPED;
    sysState = SYS_ACTIVE_MODE;
    encoderPos = 0;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
    stateInit = true;
  }
}

void simSelectRate() {
  static int lastEncoderPos = -1;
  if(stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.pulseRate;
    stateInit = false;
  }
  encoderLimit(0, 600);
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
    calib.pulseRate = encoderPos;
    calib.valid_sim = true;
    runStateSim = SIM_PAUSED;
    setupSim = SIM_SEL_MIN;
    sysState = SYS_ACTIVE_MODE;
    stateInit = true;
  }
}

void simMultiplierAdjust() {
  encoderPos = calib.multiplier;
  char buffer[8];
  while ((digitalRead(enSW)) && (digitalRead(exSW))) {
    sprintf(buffer, "%.0f ", encoderPos);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while ((digitalRead(enSW) == 0) || (digitalRead(exSW) == 0)) {
    calib.multiplier = encoderPos;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectNumTxdx() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    lastEncoderPos = -1;
    tft.fillScreen(ST7735_BLACK);
    initScreenAdv();
    encoderPos = calib.numTxdx;
    stateInit = false;
  }
  encoderLimit(1, 2);
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 27, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
    printWords(0, 2, 110, 27, 0xfb2c, buffer);
    calib.numTxdx = encoderPos;
    encoderPos = 0;
    stateInit = true;
    advState = ADV_DELAY;
  }
}

void selectTimeDelay() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    lastEncoderPos = -1;
    encoderPos = (calib.timeDelay * 0.2);
    stateInit = false;
  }
  int delayCounter = (encoderPos * 5);
  if (encoderPos < 1) {
    encoderPos = 1;
  }
  char buffer[10];
  snprintf(buffer, sizeof(buffer),  "%-4d", delayCounter);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    snprintf(buffer, sizeof(buffer),  "%-4d", delayCounter);
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
    calib.timeDelay = delayCounter;
    encoderPos = 0;
    stateInit = true;
    advState = ADV_WEIGHT;
  }
}

void selectFilterWeight() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.filterWeight;
    stateInit = false;
  }
  encoderLimit(1, 100);
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
    calib.filterWeight = encoderPos;
    encoderPos = 0;
    stateInit = true;
    advState = ADV_MULTIPLIER;
  }
}

void selectMultiplier() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.multiplier;
    stateInit = false;
  }
  encoderLimit(1, 10);
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    snprintf(buffer, sizeof(buffer), "%-4d", encoderPos);
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
    calib.multiplier = encoderPos;
    encoderPos = 0;
    stateInit = true;
    advState = ADV_KP;
  }
}

void selectAcceleration() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    lastEncoderPos = -1;
    encoderPos = calib.acceleration * 0.2;
    stateInit = false;
  }
  int accelCounter = encoderPos * 5;
  encoderLimit(1, 1000);
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%-4d", accelCounter);
  if (encoderPos != lastEncoderPos) {
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
    lastEncoderPos = encoderPos;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    snprintf(buffer, sizeof(buffer), "%-4d", accelCounter);
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
    calib.acceleration = accelCounter;
    if (calib.valid_cal == false) {
      sysState = SYS_CALIBRATION;
    }
    else if (calib.valid_cal == true){
      if (mode <= 1 || mode >=5) {
        encoderPos = 0;
        tft.fillScreen(ST7735_BLACK);
        sysState = SYS_CHOOSE_MODE;
      }
      else if (mode >= 2 && mode <= 4) {
        tft.fillScreen(ST7735_BLACK);
        sysState = SYS_ACTIVE_MODE;
        encoderPos = 0;
      }
    }
    advState = ADV_TXDX;
    stateInit = true;
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

void triangle() {
  if (beatDir == true) {
    if (avgPressure <= (calib.minmmHg + 0.5)) {
      beatDir = false; 
      pulseCounter++;
      tempRate = micros() - prevTiming;
      prevTiming = micros();
    }
  } else {
    if (avgPressure >= (calib.maxmmHg - 0.5)) { 
      beatDir = true; 
      pulseCounter++;
      tempRate = micros() - prevTiming;
      prevTiming = micros();
    }
  }
  if (beatDir == true) {
    oscillate(calib.minmmHg);
  } else {
    oscillate(calib.maxmmHg);
  }
  if (tempRate > 1) {
    actualRate = 30000000.0 / tempRate; 
  } else {
    actualRate = 0;
  }
}

/******************************** Core Functions ********************************/

void isRunningMoto() {
  if (!_vm_pcActive) {
    sel_pressure = encoderPos; 
  }
  unsigned long elapsedMillis = currentMillis - startMillis;
  static int lastColoring = -999;
  pressureControl(calib.acceleration);
  updateSensors(calib.filterWeight);
  if (currentMillis - previousMillis >= outputDelay) {
    currentTime = (elapsedMillis / 1000.00);
    snprintf(pressure, sizeof(pressure), "%-4.0f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    if (calib.numTxdx == 2) {
      snprintf(tension, sizeof(tension), "%-4.1f", avgTension);
    } else {
      snprintf(tension, sizeof(tension), "0   ");
    }
    snprintf(time, sizeof(time), "%.2f", currentTime);
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    snprintf(RunningOutputMoto, sizeof(RunningOutputMoto), "DATA T=%lu P=%.2f P_SET=%.0f", elapsedMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputMoto);
    sprintf(selected, "%.0f ", sel_pressure);
    printWords(0, 3, 106, 48, ST77XX_BLUE, selected);  
    lastColoring = -999;
    int currentColoring = (int)((encoderPos - avgPressure) * 0.5);
    if (currentColoring != lastColoring) {
      drawColorBar(currentColoring, 0, 84, 8, 5);
      lastColoring = currentColoring;
    }
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    runStateMoto = MOTO_STOPPED;
    encoderPos = 0;
    lastColoring = -999;
    stepper.move(1, 0, 0); // Lock motor
    tft.fillScreen(ST77XX_BLACK);
    stateInit = true;
  }
}

void isStoppingMoto() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    MotoScreen(ST77XX_RED, "STOPPED");
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  const char *stopMenu[] = { "UNPAUSE", "SETUP", "MAIN MENU" };
  encoderLimit(0, 2);
  listBox(79, 109, 78, 18, ST77XX_BLACK);
  printWords(7, 1, 79, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  sprintf(selected, "%.0f ", sel_pressure);
  updateSensors(calib.filterWeight);
  if (currentMillis - previousMillis >= outputDelay) {
    snprintf(pressure, sizeof(pressure), "%-4.0f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    if (calib.numTxdx == 2) {
      snprintf(tension, sizeof(tension), "%-4.1f", avgTension);
    } else {
      snprintf(tension, sizeof(tension), "0  ");
    }
    snprintf(time, sizeof(time), "%.2f", currentTime);
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    snprintf(StoppingOutputMoto, sizeof(StoppingOutputMoto), "DATA T=%lu P=%.2f P_SET=%.0f", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputMoto);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      runStateMoto = MOTO_RUNNING;
      tft.fillScreen(ST7735_BLACK);
      MotoScreen(ST77XX_GREEN, "RUNNING");
      encoderPos = sel_pressure;
    }
    else if (switchChoice == 1) {
      tft.fillScreen(ST7735_BLACK);
      stateInit = true;
      sysState = SYS_ADVANCED_SETTINGS;
      encoderPos = 0;
    }
    else if (switchChoice == 2) {
      cal_matrix savedCalib = calibrate.read();
      if (memcmp(&savedCalib, &calib, sizeof(calib)) != 0) {
        calibrate.write(calib);
      }
      stateInit = true;
      sysState = SYS_CHOOSE_MODE;
    }
  }
}

void isRunningRamp() {
  if (stateInit) {
    tft.fillScreen(ST7735_BLACK);
    RampScreen(ST77XX_GREEN, "RUNNING");
    snprintf(range, sizeof(range), "%d/%d", calib.lowmmHg, calib.highmmHg);
    snprintf(rate, sizeof(rate), "%4.1f", (float)calib.pressRate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    stateInit = false;
  }
  pressureRamp();
  pressureControl(calib.acceleration);
  updateSensors(calib.filterWeight);
  if (UseStartTime) {
    startMillis = millis();
    UseStartTime = false;
  }
  unsigned long elapsedMillis = currentMillis - startMillis;
  if (currentMillis - previousMillis >= outputDelay) {
    previousMillis = currentMillis;
    snprintf(pressure, sizeof(pressure), "%-4.0f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    snprintf(selected, sizeof(selected), "%.0f ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    snprintf(time, sizeof(time), "%6.0f", (elapsedMillis / 1000.0f));
    printWords(0, 2, 90, 84, ST77XX_WHITE, time);
    snprintf(RunningOutputRamp, sizeof(RunningOutputRamp), "DATA T=%lu P=%.2f P_SET=%.0f", elapsedMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputRamp);
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    runStateRamp = RAMP_STOPPED;
    encoderPos = 0;
    stateInit = true;
  }
}

void isStoppingRamp() {
  static int lastEncoderPos = -1;
  const char *stopMenu[] = { "PAUSE", "ADJUST", "MAIN MENU" };
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    RampScreen(ST77XX_RED, "STOPPED");
    snprintf(range, sizeof(range), "%d/%d", calib.lowmmHg, calib.highmmHg);
    snprintf(rate, sizeof(rate), "%4.1f", (float)calib.pressRate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  encoderLimit(0, 2);
  if (encoderPos != lastEncoderPos) {
    listBox(79, 109, 78, 18, ST77XX_BLACK);
    printWords(7, 1, 79, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
    lastEncoderPos = encoderPos;
  }
  updateSensors(calib.filterWeight);
  if (currentMillis - previousMillis >= outputDelay) {
    previousMillis = currentMillis;
    snprintf(pressure, sizeof(pressure), "%-4.1f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    snprintf(selected, sizeof(selected), "%.0f ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    snprintf(StoppingOutputRamp, sizeof(StoppingOutputRamp), "DATA T=%lu P=%.2f P_SET=%.0f", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputRamp);
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      runStateRamp = RAMP_PAUSED;
      encoderPos = sel_pressure;
      prevmillis = currentMillis;
      stateInit = true;
    } 
    else if (switchChoice == 1) {
      stateInit = true;
      sysState = SYS_RAMP_SETUP;
    } 
    else if (switchChoice == 2) {
      cal_matrix savedCalib = calibrate.read();
      if (memcmp(&savedCalib, &calib, sizeof(calib)) != 0) {
        calibrate.write(calib);
      }
      stateInit = true;
      sysState = SYS_CHOOSE_MODE;
    }
  }
}

void isPausedRamp() {
  if (stateInit) {
    tft.fillScreen(ST7735_BLACK);
    RampScreen(ST77XX_ORANGE, "PAUSED");
    snprintf(range, sizeof(range), "%d/%d", calib.lowmmHg, calib.highmmHg);
    snprintf(rate, sizeof(rate), "%4.1f", (float)calib.pressRate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    stateInit = false;
  }
  if (!_vm_pcActive) { 
    sel_pressure = encoderPos; 
  }
  pressureControl(calib.acceleration);
  updateSensors(calib.filterWeight);
  unsigned long elapsedMillis = currentMillis - startMillis;
  if (currentMillis - previousMillis >= outputDelay) {
    previousMillis = currentMillis;
    snprintf(pressure, sizeof(pressure), "%-4.0f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    snprintf(selected, sizeof(selected), "%.0f ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    snprintf(RunningOutputRamp, sizeof(RunningOutputRamp), "DATA T=%lu P=%.2f P_SET=%.0f", elapsedMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputRamp);
  }

  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    runStateRamp = RAMP_RUNNING;
    prevmillis = currentMillis;
    UseStartTime = true;
    stateInit = true;
  }
}

void isPausedSim() {
  if (stateInit) {
    tft.fillScreen(ST7735_BLACK);
    RampScreen(ST77XX_ORANGE, "PAUSED");
    snprintf(range, sizeof(range), "%d/%d", calib.minmmHg,calib.maxmmHg);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    encoderPos = calib.minmmHg;
    stateInit = false;
  }
  if (!_vm_pcActive) { 
    sel_pressure = encoderPos; 
  }
  pressureControl(calib.acceleration);
  updateSensors(calib.filterWeight);
  unsigned long elapsedMillis = currentMillis - startMillis;
  if (currentMillis - previousMillis >= outputDelay) {
    currentTime = (elapsedMillis / 1000.00);
    snprintf(pressure, sizeof(pressure), "%-4.0f", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(selected, "%.0f ", sel_pressure);
    printWords(0, 2, 106, 24, ST77XX_BLUE, selected);
    snprintf(rate, sizeof(rate), "%4.1f", calib.pulseRate);
    snprintf(RunningOutputSim, sizeof(RunningOutputRamp), "DATA T=%lu P=%.2f P_SET=%.0f", elapsedMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputSim);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
      numSteps = (calib.maxmmHg - calib.minmmHg) * calib.multiplier;
      if (numSteps > 0 && calib.pulseRate > 0) {
        pulseInt = (60000000UL / calib.pulseRate) / (2 * numSteps);
      } else {
        pulseInt = minDelay;
      }
      runStateSim = SIM_RUNNING;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      SimScreen(ST77XX_GREEN, "RUNNING");
      prevmillis = currentMillis;
      prevTiming = micros();
      UseStartTime = true;
  }
}

void isRunningSim() {
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    prevmillis = currentMillis;
    prevTiming = micros();
    SimScreen(ST77XX_GREEN, "RUNNING");
    printHeader();
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    snprintf(range, sizeof(range), "%d/%d", calib.minmmHg, calib.maxmmHg);
    encoderPos = 0;
    stateInit = false;
  }
  triangle();
  if (UseStartTime == true) {
    startMillis = millis();
    UseStartTime = false;
  }
  unsigned long elapsedMillis = currentMillis - startMillis;
  if (currentMillis - previousMillis >= outputDelay) {
    avgPressure = ads.measure(txdx1);
    currentTime = (elapsedMillis / 1000.00);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    if (calib.numTxdx == 2) {
      sprintf(tension, "-4.1f", avgTension);
    } else {
      sprintf(tension, "0  ");
    }
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(rate, "%-4.2f  ", actualRate);
    sprintf(time, "%-.1f", currentTime);
    printWords(0, 2, 80, 64, ST77XX_CYAN, rate);
    printWords(0, 2, 80, 84, ST77XX_CYAN, time);
    snprintf(RunningOutputSim, sizeof(RunningOutputRamp), "DATA T=%lu P=%.2f P_SET=%.0f", elapsedMillis, avgPressure, sel_pressure);
    Serial.println(RunningOutputSim);
    // Serial.println(pressure);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    runStateSim = SIM_STOPPED;
    encoderPos = 0;
    stateInit = true;
  }
}

void isStoppingSim() {
  static int lastEncoderPos = -1;
  if (stateInit) {
    tft.fillScreen(ST77XX_BLACK);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    snprintf(range, sizeof(range), "%d/%d", calib.minmmHg, calib.maxmmHg);
    encoderPos = 0;
    lastEncoderPos = -1;
    stateInit = false;
  }
  const char *stopMenu[] = { "PAUSE", "ADJUST", "MAIN MENU" };
  encoderLimit(0, 2);
  listBox(79, 109, 78, 18, ST77XX_BLACK);
  printWords(7, 1, 79, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  SimScreen(ST77XX_RED, "STOPPED");
  updateSensors(calib.filterWeight);
  if (currentMillis - previousMillis >= outputDelay) {
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    if (calib.numTxdx == 2) {
      sprintf(tension, "%-4.1f", avgTension);
    } else {
      sprintf(tension, "  ");
    }
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(range, "%d/%d", calib.minmmHg, calib.maxmmHg);
    snprintf(rate, sizeof(rate), "%4.1f", actualRate);
    printWords(0, 2, 90, 64, ST77XX_CYAN, rate);
    sprintf(StoppingOutputSim, "DATA T=%lu P=%.2f P_SET=%.0f", currentMillis, avgPressure, sel_pressure);
    Serial.println(StoppingOutputSim);
    previousMillis = currentMillis;
  }
  if (isButtonPressed(enSW, lastEnSWState) || isButtonPressed(exSW, lastExSWState)) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      numSteps = (calib.maxmmHg - calib.minmmHg) * calib.multiplier;
      if (numSteps > 0 && calib.pulseRate > 0) {
        pulseInt = (60000000UL / calib.pulseRate) / (2 * numSteps);
      } else {
        pulseInt = minDelay;
      }
      runStateSim = SIM_PAUSED;
      stateInit = true;
    }
    if (switchChoice == 1) {
      stateInit = true;
      encoderPos = sel_pressure;
      sysState = SYS_SIM_SETUP;
      setupSim = SIM_SEL_MIN;
    }
    if (switchChoice == 2) {
      cal_matrix savedCalib = calibrate.read();
      if (memcmp(&savedCalib, &calib, sizeof(calib)) != 0) {
        calibrate.write(calib);
      }
      stateInit = true;
      sysState = SYS_CHOOSE_MODE;
    }
  }
}
