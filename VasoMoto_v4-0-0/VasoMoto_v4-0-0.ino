/* This version includes external inpout to change pressure. All tension recording functions are disabled. Can be re-enabled for a second pressure
   transducer, but name should be changed to prevent confusion. */

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
  int runStateSim = 0;
  int runStateMoto = 0;
  bool fillLines = false;
  bool printSettings = true;
  int choice;
  bool beatDir;

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
  int maxDelay = 30000;   //  Used in pressureControl to easily alter how slow the stepper starts, so I dont have to change in 12 places.
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
    bool valid;
    float pLowSel;
    float pHiSel;
    float tLowSel;
    float tHiSel;
    int pLowADC;
    int pHiADC;
    int tLowADC;
    int tHiADC;
  } calib;
  struct init_matrix {    //place to store all the values from the "advanced" settings menu.
    bool valid;
    int timeDelay;         //the time delay for wheatstone bridge output.
    int filterWeight;      //the weighting of the averaging filter.
    float multiplier;      //Use this to set the velocity multiplier to get the right shape of the pressure curve.             
    int acceleration;      //Use this to alter how fast the pump accelerates and to what speed.
    int numSamples;        //the number of samples averaged. Was set to 5 in v200 for some reason?
  } startup;
  struct sim_matrix {     //place to store values of the pulse simulation.
    bool valid;
    int minmmHg;
    int maxmmHg;
    int pulseRate;
    int stepsPer;
  } sim;
  FlashStorage(calibrate, cal_matrix);
  FlashStorage(initialize, init_matrix);
  FlashStorage(simulation, sim_matrix);

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
  bool UseStartTime = true;
  bool simu = false;
  bool moto = false;
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
  char StoppingOutputSim[32];
  char pressureBufferLCA[6];  //Pressure calib.pLowADC string
  char pressureBufferHCA[6];  //Pressurecalib.pHiADC string
  char pressureBufferLCS[4];  //Pressure calib.pLowSel string
  char pressureBufferHCS[4];  //Pressure calib.pHiSel string
  char tensionBufferLCA[6];  //Tension low_cal_ADC string
  char tensionBufferHCA[6];  //Tension high_cal_ADC string
  char tensionBufferLCS[4];  //Tension low_cal_sel string
  char tensionBufferHCS[4];  //Tension high_cal_sel string
  char bufferLP[4];   //low pressure string
  char bufferHP[4];   //high pressure string
  char bufferPR[4];   //pulse rate string
  char debugging[50]; //for ease of reading serial output stuff

void setup() {
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

  bootup();
  chooseMode();
  calibration();
  ads.linearCal(calib.pLowADC, calib.pHiADC, calib.pLowSel, calib.pHiSel);
  // ads.linearCal(calib.tLowADC, calib.tHiADC, calib.tLowSel, calib.tHiSel);
  if(moto == true && simu == false) {
    stepper.setStepFrac(8);
    clickBegin();
    delay(500);
    
  }
  if(moto == false && simu == true) {
    stepper.setStepFrac(8);
    stepper.setStepFracSpeed(8, stepsPerSec);
    bootSim();
    testType();
    while (fillLines == true) {
      lineFilling();
      tft.fillScreen(ST7735_BLACK);
      initScreenSim();
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      printNumber(2, 110, 47, 0xfb2c, ST77XX_BLACK, sim.minmmHg);
      printNumber(2, 110, 67, 0xfb2c, ST77XX_BLACK, sim.maxmmHg);
      printNumber(2, 110, 87, 0xfb2c, ST77XX_BLACK, sim.pulseRate);
      testType();
      delay(500);
    }
  }
stepper.setStepFracSpeed(8, stepsPerSec);
}

void loop() {
  currentMillis = millis();
  if(moto == false && simu == true) {
    if (runStateSim == 0) {
      isStoppingSim();
    } else if (runStateSim == 1) {
      isRunningSim();
    }
  }
  else if(moto == true && simu == false) {
    if (runStateMoto == 0) {
      isStoppingMoto();
    } else if (runStateMoto == 1) {
      isRunningMoto();
    }
  }
}
//************************************************************************************************************************//

void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    encoderPos = atoi(receivedChars);
    newData = false;
  }
}

void advancedSettings() {
  tft.fillScreen(ST7735_BLACK);
  initScreenAdv();
  selectNumSamples();
  selectTimeDelay();
  selectFilterWeight();
  selectMultiplier();
  selectAcceleration();
  startup.valid = true;
  initialize.write(startup);
  delay(100);
  NVIC_SystemReset();
}

void averagingPressure(int q) {
  float Pressure = ads.measure(txdx1);
  for (int i = 0; i < q; i++) {
    avgPressure = avgPressure + (Pressure - avgPressure) / startup.filterWeight;
  }
}

void averagingTension(int q) {
  double Tension = ads.measure(txdx2);
  for (int i = 0; i < q; i++) {
    avgTension = avgTension + (Tension - avgTension) / startup.filterWeight;
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
  printWords(0, 1, 80, 120, ST77XX_RED, "v4.0.0");
  startup = initialize.read();
  if (startup.valid == false) {
    delay(1000);
    tft.fillRect(0, 100, 160, 28, ST77XX_BLACK);
    printWords(8, 1, 1, 111, ST77XX_YELLOW, "First Complete Setup");
    delay(1000);
    startup.numSamples = 10;
    startup.timeDelay = 20;
    startup.multiplier = 4;
    startup.filterWeight = 3;
    startup.acceleration = 50;
    advancedSettings();  
  }
}

void bootSim() {
  sim = simulation.read();
  if (sim.valid == false) {
    delay(500);
    sim.minmmHg = 60;
    sim.maxmmHg = 100;
    sim.pulseRate = 200;
  }
    simSetup();
}

void calibration() {
  const char *calMenu[] = { "Load", "New", "N/A" };
  encoderPos = 0;
  if(moto == true && simu == false) {
    initScreenMoto();
  }
  else if(moto == false && simu == true) {
    initScreenSim();
  }
  while (digitalRead(enSW)) {
    int position = encoderPos;
    encoderLimit(0, 2);
    if (position == 0) {
      listBox(89, 27, 70, 14, ST77XX_BLACK);
      printWords(9, 1, 90, 39, ST77XX_WHITE, calMenu[position]);
      choice = position;
    } else if (position == 1) {
      listBox(89, 27, 70, 14, ST77XX_BLACK);
      printWords(9, 1, 90, 39, ST77XX_WHITE, calMenu[position]);
      choice = position;
    } else if (position == 2) {
      listBox(89, 27, 70, 14, ST77XX_BLACK);
      printWords(9, 1, 90, 39, ST77XX_WHITE, calMenu[position]);
      choice = position;
    } 
  }
  while (digitalRead(enSW) == 0) {
    printWords(9, 1, 90, 39, 0xfb2c, calMenu[choice]);
  }
  if (choice == 0) {
    calib = calibrate.read();
    if (calib.valid == true) {
      calibrationOrder = 0;
      calWords();
      calNums();
      delay(200);
      offsetPressure();
      calibrationOrder = 1;
      calWords();
      calNums();
      // offsetTension();
      tft.fillScreen(ST77XX_BLACK);
      if(moto == false && simu == true) {
        initScreenSim();
        printWords(9, 1, 102, 39, 0xfb2c, "Done");
      }
      calibrate.write(calib);
    }
    else {
      calibration();
    }
  }
  if (choice == 1) {
    delay(250);
    // calibrationOrder = 0;
    calWords();
    PressureADCLow();
    PressureSelLow();
    PressureADCHigh();
    PressureSelHigh();
    delay(200);
    offsetPressure();
    // calibrationOrder = 1;
    // calWords();
    // TensionADCLow();
    // TensionSelLow();
    // TensionADCHigh();
    // TensionSelHigh();
    // offsetTension();
    tft.fillScreen(ST77XX_BLACK);
    calib.valid = true;
    if (moto == true && simu == false) {
      initScreenMoto();
      calibrate.write(calib);
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(1000);
    }
    else if(moto == false && simu == true) {
      initScreenSim();
      calibrate.write(calib);
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
      delay(1000);
    }
  }
  if (choice == 2) {
    tft.fillScreen(ST77XX_BLACK);
    calibrationOrder = 0;
    calib.pLowADC = 261;
    calib.pLowSel = 0;
    calib.pHiADC =  7930;
    calib.pHiSel = 82;
    calWords();
    calNums();
    offsetPressure();
    calibrationOrder = 1;
    calib.tLowADC = 0;
    calib.tLowSel = 0;
    calib.tHiADC = 0;
    calib.tHiSel = 0;
    calWords();
    calNums();
    // offsetTension();
    tft.fillScreen(ST77XX_BLACK);
    if(moto == false && simu == true) {
      initScreenSim();
      calib.valid = true;
      calibrate.write(calib);
      printWords(9, 1, 102, 39, 0xfb2c, "Done");
    }
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

void chooseMode() {
  const char *modeMenu[] = {"Turn to Select Mode","Pressure Control","Pulse Simulator","Advanced Settings" };
  encoderPos = 0;
  while (digitalRead(enSW)) {
    encoderLimit(0, 3);
    if (encoderPos == 0) {
      listBox(0, 100, 160, 28, ST77XX_BLACK);
      printWords(8, 1, 1, 111, ST77XX_WHITE, modeMenu[encoderPos]);
    }
    if (encoderPos == 1) {
      listBox(0, 100, 160, 28, ST77XX_BLACK);
      printWords(8, 1, 15, 111, ST77XX_WHITE, modeMenu[encoderPos]);
    }
    else if (encoderPos == 2) {
      listBox(0, 100, 160, 28, ST77XX_BLACK);
      printWords(8, 1, 20, 111, ST77XX_WHITE, modeMenu[encoderPos]);
    }
    else if (encoderPos == 3) {
      listBox(0, 100, 160, 28, ST77XX_BLACK);
      printWords(8, 1, 7, 111, ST77XX_WHITE, modeMenu[encoderPos]);
    }  
  }
  while (digitalRead(enSW) == 0) {
    int switchChoice = encoderPos;
    if (switchChoice == 0) {
      bootup();
      chooseMode();
    }
    if (switchChoice == 1) {
      printWords(8, 1, 15, 111, 0xfb2c, modeMenu[encoderPos]);
      simu = false;
      moto = true;
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 2) {
      printWords(8, 1, 20, 111, 0xfb2c, modeMenu[encoderPos]);
      simu = true;
      moto = false;
      delay(500);
      tft.fillScreen(ST77XX_BLACK);
    }
    if (switchChoice == 3) {
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

void initScreenSim() {
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
  int mmHg = sim.minmmHg;
  if (mmHg > 0) {
    encoderPos = mmHg;
  }
  while (digitalRead(enSW)) {
    stepper.setStepFracSpeed(8, stepsPerSec);
    fillFast();
    sel_pressure = encoderPos;
    currentMillis = millis();
    if (currentMillis - previousMillis >= startup.timeDelay) {
      averagingPressure(startup.numSamples);
      sprintf(selected, " %d ", sel_pressure);
      sprintf(pressure, "%.0f ", avgPressure);
      printWords(0, 3, 80, 2, ST77XX_WHITE, pressure);
      printWords(0, 3, 80, 39, ST77XX_WHITE, selected);
      previousMillis = currentMillis;
    }
  }
  while (digitalRead(enSW) == 0) {
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
  float samplesOffset[startup.numSamples];
  float avgSample;
  uint8_t i;
  float tempAvg;
  encoderPos = 0;
  int tempLowADC = calib.pLowADC;
  int tempHighADC = calib.pHiADC;
  float tempLowSel = calib.pLowSel;
  float tempHighSel = calib.pHiSel;
  while (digitalRead(enSW)) {
    tempLowADC = (calib.pLowADC - (encoderPos * 10));
    tempHighADC = (calib.pHiADC - (encoderPos * 10));
    for (i = 0; i < startup.numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, tempLowSel, tempHighSel);
      samplesOffset[i] = ads.measure(txdx1);
      delay(10);
    }
    avgSample = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgSample += samplesOffset[i];
    }
    avgSample /= startup.numSamples;
    tempAvg = avgSample;
    char tempOffset[10];
    sprintf(tempOffset, "%.1f ", tempAvg);
    printWords(0, 2, 102, 106, ST77XX_WHITE, tempOffset);
  }
  while (digitalRead(enSW) == 0) {
    calib.pLowADC = tempLowADC;
    calib.pHiADC = tempHighADC;
    calib.pLowSel = tempLowSel;
    calib.pHiSel = tempHighSel;
  }
}

void offsetTension() {
  float samplesOffset[startup.numSamples];
  float avgSample;
  uint8_t i;
  float tempAvg;
  encoderPos = 0;
  int tempLowADC = calib.tLowADC;
  int tempHighADC = calib.tHiADC;
  while (digitalRead(enSW)) {
    tempLowADC = (calib.tLowADC - (encoderPos * 10));
    tempHighADC = (calib.tHiADC - (encoderPos * 10));
    for (i = 0; i < startup.numSamples; i++) {
      ads.linearCal(tempLowADC, tempHighADC, calib.tLowSel, calib.tHiSel);
      samplesOffset[i] = ads.measure(txdx2);
      delay(10);
    }
    avgSample = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgSample += samplesOffset[i];
    }
    avgSample /= startup.numSamples;
    tempAvg = avgSample;
    char tempOffset[10];
    sprintf(tempOffset, "%.1f ", tempAvg);
    printWords(0, 2, 102, 106, ST77XX_WHITE, tempOffset);
  }
  while (digitalRead(enSW) == 0) {
    calib.tLowADC = tempLowADC;
    calib.tHiADC = tempHighADC;
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

void pressureControl(int accel) {
  float motors;
  int minDelay = 1000;
  currentMicros = millis();                     //I know the variable is named weird. I didnt want to go back and change it again for no reason.
  motors = (encoderPos - avgPressure);
  if (motors <= -0.3) {
    if (currentMicros - previousMicros >= accel) {
      goDown = maxDelay;
      if (goUp > minDelay){
      x = x + 1;
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
        y = y + 1;
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

void PressureADCHigh() {
  int firstADC = 0;
  int avgHigh;
  uint8_t i;
  int samplesHigh[startup.numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < startup.numSamples; i++) {
      samplesHigh[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgHigh = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgHigh += samplesHigh[i];
    }
    avgHigh /= startup.numSamples;
    firstADC = avgHigh;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    calib.pHiADC = firstADC;
    delay(200);
  }
}

void PressureADCLow() {
  calWords();
  int firstADC = 0;
  int avgLow;
  uint8_t i;
  int samplesLow[startup.numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < startup.numSamples; i++) {
      samplesLow[i] = ads.readADC_Differential_0_1();
      delay(1);
    }
    avgLow = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgLow += samplesLow[i];
    }
    avgLow /= startup.numSamples;
    firstADC = avgLow;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    calib.pLowADC = firstADC;
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
    calib.pHiSel = encoderPos;
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
    calib.pLowSel = encoderPos;
    delay(200);
  }
}

void simAdjust() {
  initScreenAdjust();
  delay(100);
  selectMinPressure();
  selectMaxPressure();
  selectRate();
  delay(100);
  sim.valid = true;
  simulation.write(sim);
}

void simSetup() {
  initScreenSim();
  delay(100);
  selectMinPressure();
  selectMaxPressure();
  selectRate();
  delay(100);
  sim.valid = true;
  simulation.write(sim);
}

void TensionADCHigh() {
  int firstADC = 0;
  int avgHigh;
  uint8_t i;
  int samplesHigh[startup.numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < startup.numSamples; i++) {
      samplesHigh[i] = ads.readADC_Differential_2_3();
      delay(1);
    }
    avgHigh = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgHigh += samplesHigh[i];
    }
    avgHigh /= startup.numSamples;
    firstADC = avgHigh;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 66, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    calib.tHiADC = firstADC;
    delay(200);
  }
}

void TensionADCLow() {
  calWords();
  int firstADC = 0;
  int avgLow;
  uint8_t i;
  int samplesLow[startup.numSamples];
  while (digitalRead(enSW)) {
    for (i = 0; i < startup.numSamples; i++) {
      samplesLow[i] = ads.readADC_Differential_2_3();
      delay(1);
    }
    avgLow = 0;
    for (i = 0; i < startup.numSamples; i++) {
      avgLow += samplesLow[i];
    }
    avgLow /= startup.numSamples;
    firstADC = avgLow;
    char buffer[10];
    sprintf(buffer, "%d ", firstADC);
    printWords(0, 2, 90, 26, ST77XX_WHITE, buffer);
    delay(200);
  }
  while (digitalRead(enSW) == 0) {
    calib.tLowADC = firstADC;
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
    calib.tHiSel = encoderPos;
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
    calib.tLowSel = encoderPos;
    delay(200);
  }
}

void printHeader() {
  Serial.println("----------------------------------------");
  sprintf(pressureBufferLCA, "Pressure ADC Min: %i | ", calib.pLowADC); Serial.print(pressureBufferLCA); 
  sprintf(tensionBufferLCA, "Tension ADC Min: %i", calib.tLowADC); Serial.println(tensionBufferLCA);
  sprintf(pressureBufferHCA, "Pressure ADC Max: %i | ", calib.pHiADC); Serial.print(pressureBufferHCA);
  sprintf(tensionBufferHCA, "Tension ADC Max: %i", calib.tHiADC); Serial.println(tensionBufferHCA);
  sprintf(pressureBufferLCS, "Pressure Cal Min: %.0f mmHg | ", calib.pLowSel); Serial.print(pressureBufferLCS);
  sprintf(tensionBufferLCS, "Tension Cal Min: %.0f uN", calib.tLowSel); Serial.println(tensionBufferLCS);
  sprintf(pressureBufferHCS, "Pressure Cal Max: %.0f mmHg | ", calib.pHiSel); Serial.print(pressureBufferHCS);
  sprintf(pressureBufferHCS, "Tension Cal Max: %.0f uN", calib.tHiSel); Serial.println(tensionBufferHCS);
  sprintf(bufferLP, "Min Pressure: %d mmHg", sim.minmmHg); Serial.println(bufferLP);
  sprintf(bufferHP, "Max Pressure: %d mmHg", sim.maxmmHg); Serial.println(bufferHP);
  sprintf(bufferPR, "Pulse Rate: %d BPM", sim.pulseRate); Serial.println(bufferPR);
  Serial.println("----------------------------------------");
  printSettings = false;
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
  encoderPos = startup.acceleration;
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
    startup.acceleration = accelCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectFilterWeight() {
  encoderPos = (startup.filterWeight / 3);
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
    startup.filterWeight = filterCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectMaxPressure() {
  encoderPos = sim.maxmmHg;
  char buffer[3];
  int maxCounter;
  while (digitalRead(enSW)) {
    maxCounter = encoderPos;
    sprintf(buffer, " %3d", maxCounter);
    printWords(0, 2, 110, 67, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    sim.maxmmHg = maxCounter;
    printWords(0, 2, 110, 67, 0xfb2c, buffer);
  }
}

void selectMinPressure() {
  encoderPos = sim.minmmHg;
  char buffer[3];
  int minCounter;
  while (digitalRead(enSW)) {
    minCounter = encoderPos;
    sprintf(buffer, " %3d", minCounter);
    printWords(0, 2, 110, 47, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    sim.minmmHg = minCounter;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
  }
}

void selectMultiplier() {
  float tempMult = startup.multiplier;
  encoderPos = tempMult;
  char buffer[3];
  float multCounter;
  while (digitalRead(enSW)) {
    multCounter = encoderPos;
    sprintf(buffer, "%.0f ", multCounter);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    startup.multiplier = multCounter;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void selectMultiplierAdjust() {
  float tempMult = startup.multiplier;
  encoderPos = tempMult;
  char buffer[3];
  float multCounter;
  while (digitalRead(enSW)) {
    multCounter = encoderPos;
    sprintf(buffer, "%.0f ", multCounter);
    printWords(0, 2, 110, 107, ST77XX_WHITE, buffer);
  }
  while (digitalRead(enSW) == 0) {
    startup.multiplier = multCounter;
    printWords(0, 2, 110, 107, 0xfb2c, buffer);
  }
}

void selectNumSamples() {
  encoderPos = startup.numSamples;
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
    startup.numSamples = sampleCounter;
    printWords(0, 2, 110, 27, 0xfb2c, buffer);
  }
}

void selectRate() {
  encoderPos = (sim.pulseRate / 5);
  char buffer[3];
  int steps;
  int constrainedSteps;
  while (digitalRead(enSW)) {
    steps = (encoderPos * 5);
    if (steps >= 500) {
      steps = 500;
    }
    sprintf(buffer, " %3d", steps);
    printWords(0, 2, 110, 87, ST77XX_WHITE, buffer); 
  }
  while (digitalRead(enSW) == 0) {
    sim.pulseRate = steps;
    printWords(0, 2, 110, 87, 0xfb2c, buffer);
  }
}

void selectTimeDelay() {
  encoderPos = (startup.timeDelay / 5);
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
    startup.timeDelay = delayCounter;
    printWords(0, 2, 110, 47, 0xfb2c, buffer);
  }
}

void SimScreen(uint16_t color, const char *state) {
  tft.drawRect(2, 108, 158, 20, color);
  printWords(7, 1, 6, 122, color, state);
  printWords(7, 1, 2, 16, ST77XX_WHITE, "Pressure");
  printWords(7, 1, 96, 16, ST77XX_WHITE, "Tension");
  printWords(8, 1, 1, 54, 0xfb2c, "Range:");
  printWords(8, 1, 1, 75, 0xfb2c, "Rate:");
  printWords(8, 1, 1, 95, 0xfb2c, "Time:");
  tft.drawFastHLine(0, 20, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 41, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 61, 160, ST77XX_WHITE);
  tft.drawFastHLine(0, 81, 160, ST77XX_WHITE);
}

void testType() {
  encoderPos = 0;
  const char *testMenu[] = { "Line Fill", "Triangle", "Sim Pulse", "Reset?" };
  int testCounter;
  while (digitalRead(enSW)) {
    testCounter = encoderPos;
    if (testCounter < 0) { testCounter = 0; }
    if (testCounter > 3) { testCounter = 3; }
    listBox(69, 105, 90, 23, ST77XX_BLACK);
    printWords(9, 1, 70, 118, ST77XX_WHITE, testMenu[testCounter]);
  }
  while (digitalRead(enSW) == 0) {
    expType = testCounter;
    printWords(9, 1, 70, 118, 0xfb2c, testMenu[testCounter]);
  }
  if (expType == 0) {
    fillLines = true;
    Serial.println("Line Filling");
  }
  if (expType == 1) {
    fillLines = false;
    Serial.println("Triangular Pulse");
    tft.fillScreen(ST7735_BLACK);
    runStateSim = 0;
    runStateMoto = 0;
    UseStartTime = true;
    encoderPos = 0;
    delay(100);
  }
  if (expType == 2) {
    fillLines = false;
    Serial.println("Simulated Arterial Pressure Pulse");
    tft.fillScreen(ST7735_BLACK);
    runStateSim = 0;
    UseStartTime = true;
    encoderPos = 0;
    delay(100);
  }
  if (expType == 3) {
      NVIC_SystemReset();
  }
}

void triangle() {
  currmillis = millis();
  beatTime = 30000 / sim.pulseRate;
  if (currmillis - prevmillis > beatTime) {
    beatDir = !beatDir;
    pulseCounter++;
    tempRate = currmillis - prevmillis;
    prevmillis = currmillis;
  }
  if (beatDir == true) {
    oscillate(sim.minmmHg);
  } else if (beatDir == false) {
    oscillate(sim.maxmmHg);
  }
  actualRate = 30000 / tempRate;
}

/******************************** Core Functions ********************************/

void isRunningMoto() {
  recvWithStartEndMarkers();
  showNewData();
  sel_pressure = encoderPos;
  currentMillis = millis() - startMillis;
  pressureControl(startup.acceleration);
  if (currentMillis - previousMillis >= startup.timeDelay) {
    currentTime = (currentMillis / 1000.00);
    averagingPressure(startup.numSamples);
    averagingTension(startup.numSamples);
    sprintf(pressure, "%.1f  ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    // printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(RunningOutputMoto, "<P1:%.2f;P2:%.2f>", avgPressure, avgPressure); //change 2nd one to 'avgTension' once VasoTracker can handle it
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

void isRunningSim() {
  if (expType == 1) {
    triangle();
  }
  if (UseStartTime == true) {
    startMillis = millis();
    UseStartTime = false;
  }
  currentMillis = millis() - startMillis;
  if (currentMillis - previousMillis >= startup.timeDelay) {
    averagingPressure(startup.numSamples);
    averagingTension(startup.numSamples);
    currentTime = (currentMillis / 1000.00);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    // printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(rate, "%-.2f", actualRate);
    sprintf(time, "%-.1f", currentTime);
    printWords(0, 2, 80, 64, ST77XX_CYAN, rate);
    printWords(0, 2, 80, 84, ST77XX_CYAN, time);
    sprintf(RunningOutputSim, "%-.2f; %.2f; %.2f; %.1f", currentTime, avgPressure, avgTension, actualRate);
    Serial.println(RunningOutputSim);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    runStateSim = 0;
    encoderPos = 0;
    tft.fillScreen(ST77XX_BLACK);
  }
}

void isStoppingMoto() {
  const char *stopMenu[] = { "UNPAUSE", "RESET?" };
  encoderLimit(0, 1);
  if (encoderPos == 0) {
    listBox(79, 109, 79, 18, ST77XX_BLACK);
    printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  }
  if (encoderPos == 1) {
    listBox(79, 109, 79, 18, ST77XX_BLACK);
    printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  }
    sprintf(selected, "%d ", sel_pressure);
    MotoScreen(ST77XX_RED, "STOPPED");
      if (currentMillis - previousMillis >= startup.timeDelay) {
        averagingPressure(startup.numSamples);
        averagingTension(startup.numSamples);
        sprintf(pressure, "%.1f  ", avgPressure);
        printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
        sprintf(tension, "%.1f  ", avgTension);
        printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
        sprintf(StoppingOutputMoto, "<P1:%.2f;P2:%.2f>", avgPressure, avgPressure); //change 2nd one to 'avgTension' once VasoTracker can handle it
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

void isStoppingSim() {
  int switchChoice;
  const char *stopMenu[] = { "UNPAUSE", "ADJUST", "RESET?" };
  encoderLimit(0, 2);
  listBox(79, 109, 79, 18, ST77XX_BLACK);
  printWords(7, 1, 88, 122, ST77XX_YELLOW, stopMenu[encoderPos]);
  SimScreen(ST77XX_RED, "STOPPING");
  int minmmHg = sim.minmmHg;
  int maxmmHg = sim.maxmmHg;
  if (currentMillis - previousMillis >= startup.timeDelay) {
    averagingPressure(startup.numSamples);
    averagingTension(startup.numSamples);
    sprintf(pressure, "%.1f ", avgPressure);
    printWords(0, 2, 16, 24, ST77XX_CYAN, pressure);
    sprintf(tension, "%.1f  ", avgTension);
    printWords(0, 2, 106, 24, ST77XX_CYAN, tension);
    sprintf(range, "%d/%d", minmmHg, maxmmHg);
    dtostrf(actualRate, 4, 1, rate);
    printWords(0, 2, 80, 44, ST77XX_CYAN, range);
    printWords(0, 2, 80, 64, ST77XX_CYAN, rate);
    previousMillis = currentMillis;
  }
  while (digitalRead(enSW) == 0) {
    switchChoice = encoderPos;
    if (switchChoice == 0) {
      numSteps = (sim.maxmmHg - sim.minmmHg) * startup.multiplier;
      pulseInt = (60000000 / sim.pulseRate) / (2 * numSteps);
      runStateSim = 1;
      int minmmHg = sim.minmmHg;
      int maxmmHg = sim.maxmmHg;
      tft.fillScreen(ST7735_BLACK);
      printWords(0, 2, 80, 44, ST77XX_CYAN, range);
      SimScreen(ST77XX_GREEN, "RUNNING");
      prevmillis = currentMillis;
    }
    if (switchChoice == 1) {
      tft.fillScreen(ST7735_BLACK);
      simAdjust();
      selectMultiplierAdjust();
      tft.fillScreen(ST7735_BLACK);
      encoderPos = 0;
    }
    if (switchChoice == 2) {
      NVIC_SystemReset();
    }
  }
}
