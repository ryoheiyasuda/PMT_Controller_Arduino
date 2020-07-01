/*
    COPYRIGHT nozomi yasuda

    TODO:
    ====
    Pin numbers are random, need to be adjusted

    Serial input commands:
    ======================
    v_i%d = set voltage to %d (like v_i50)
    v_m = monitor voltage
    pmtA1 = turn on pmt A (or B)
    pmtA0 = turn off pmt A
    powA1 = turn on Peltier A etc to make PMT A ready.
    powA0 = turn off PMT A.
    autA1 = auto-recovery upon PMT A error. (default off)
    autA0 = turn off above feature.

    defined(ARDUINO_AVR_MEGA) --- mega
    defined(ARDUINO_SAM_DUE) --- due

    defined(TEENSYDUINO) --- Teesy products
*/

#if defined(ARDUINO_SAM_DUE)
#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;
#define use_flash 1
#else
#include <EEPROM.h>
#define use_flash 0
#endif

const short debug = 1;
const short nPMT = 2; //number of PMT's connected to arduino

//Pins
const short pelPowPin[nPMT] = {35, 28}; //External connector pin number: 3
const short pelErrPin[nPMT] = {36, 29}; //External connector pin number: 5
const short pmtPowPin[nPMT] = {37, 30}; //External connector pin number: 7
const short pmtErrPin[nPMT] = {38, 31}; //External connector pin number: 9
const short pmtOnPin[nPMT] = {39, 32}; //External connector pin number: 11
const short vMonitorInputPin[nPMT] = {A0, A1}; //External connector pin number: 15 --- Analog input

#if defined(ARDUINO_SAM_DUE)
const short vltOutputPin[nPMT] = {DAC0, DAC1}; //External connector pin number: 17 --- Analog output
#else
const short vltOutputPin[nPMT] = {2, 4}; //External connector pin number: 17 --- Analog output (use 2- 13)
#endif
const short fanPowPin[nPMT] = {50, 33}; //External connector pin number: 19

//use for monitor panel.
const short vMonitorOutputPin[nPMT] = {3, 5}; //Output for vMonitorInput. --- Analog output (use 2- 13)

const short ledPin[nPMT] = {8, 9}; //Signal for PMT on/off and flickers during error use 2-13.

//Rotary encoder pin
const short rotaryPinA[nPMT] = {41, 40};
const short rotaryPinB[nPMT] = {43, 42};


//Voltage calibration
const int v_input_calib = 1024; //10 bit input. Pretty noisy.
const double vdd[nPMT] = {3.3, 3.3};

#if defined(ARDUINO_SAM_DUE) or defined(TEENSYDUINO)
const short n_bits = 10; //used only for Due.
const int v_output_calib = 1024; //10 bit output
#else
const short n_bits = 8; //
const int v_output_calib = 256; //8 bit output
#endif

//Rotaty encoder state values
long counter[nPMT] = {0, 0};
short rotaryCurrentState[nPMT] = {0, 0};
short rotaryLastState[nPMT] = {0, 0};
long counterMax = v_output_calib / 3.3; //maximum voltage ~1V

//EEPROM / flash saving
short counterAddress[nPMT] = {4, 8}; //stores counter in Flash

//PMT states
bool pmt_err[nPMT] = {false, false};
bool pel_err[nPMT] = {false, false};
bool pmt_on[nPMT] = {false, false};
bool vlt_change[nPMT] = {true, true};
bool auto_pmt_recov[nPMT] = {false, false}; //Auto-recovery on off.

//adjustments
double v_monitor[nPMT] = {0, 0};
double v_out[nPMT] = {0, 0};
int ledBrightness = v_output_calib;

void setup() {
  for (short i = 0; i < nPMT; i++) {
    pinMode (rotaryPinA[i], INPUT);
    pinMode (rotaryPinB[i], INPUT);
    pinMode (pelPowPin[i], OUTPUT);
    pinMode (pelErrPin[i], INPUT);
    pinMode (pmtPowPin[i], OUTPUT);
    pinMode (pmtErrPin[i], INPUT);
    pinMode (pmtOnPin[i], INPUT);
    pinMode (vltOutputPin[i], OUTPUT); //analog
    pinMode (vMonitorOutputPin[i], OUTPUT); //analog
    pinMode (vMonitorInputPin[i], INPUT);
    pinMode (fanPowPin[i], OUTPUT);
    pinMode (ledPin[i], OUTPUT);
    digitalWrite (pmtPowPin[i], LOW);
    digitalWrite (pelPowPin[i], LOW);
    digitalWrite (fanPowPin[i], LOW);
  }

  Serial.begin (115200);

#if n_bits > 8
  analogWriteResolution(n_bits);
  analogReadResolution(n_bits);
#endif

  for (short i = 0; i < nPMT; i++)
  {
    rotaryLastState[i] = digitalRead(rotaryPinA[i]);
    readFlash(i);
    vltChange(i);

    pmtOnOff(true, i);
  }

  Serial.print("Initialized\r");
}

void readFlash(short pmtID)
{
#if use_flash == 1
  byte* b4 = dueFlashStorage.readAddress(counterAddress[pmtID]);
  memcpy(&(counter[pmtID]), b4, 4);
#else
  EEPROM.get(counterAddress[pmtID], counter[pmtID]);
#endif
}

void writeFlash(short pmtID)
{
#if use_flash == 1
  byte b4[4];
  memcpy(b4, &(counter[pmtID]), 4);
  dueFlashStorage.write(counterAddress[pmtID], b4, 4);
#else
  EEPROM.put(counterAddress[pmtID], counter[pmtID]);
#endif
}

void rotaryEncoderProcess()
{
  for (short pmtID = 0; pmtID < nPMT; pmtID++)
  {
    rotaryCurrentState[pmtID] = digitalRead(rotaryPinA[pmtID]);
    if (rotaryCurrentState[pmtID] != rotaryLastState[pmtID]) {
      if (digitalRead(rotaryPinB[pmtID]) != rotaryCurrentState[pmtID]) {
        counter[pmtID] += 1;
      } else {
        counter[pmtID] -= 1;
      }
      vlt_change[pmtID] = true;
    }
    rotaryLastState[pmtID] = rotaryCurrentState[pmtID];
  }
}

void pmtRead()
{
  for (short i = 0; i < nPMT; i++)
  {
    pel_err[i] = digitalRead(pelErrPin[i]);
    pmt_err[i] = digitalRead(pmtErrPin[i]);
    pmt_on[i] = digitalRead(pmtOnPin[i]);
    
    if (pmt_err[i] || pel_err[i]) {
      vlt_change[i] = true;

      if (auto_pmt_recov[i])
      {
        pmtOnOff(false, i);
        delay(50);
        pmtOnOff(true, i);
      }
      else
      {
        analogWrite (ledPin[i], ledBrightness);
        delay(400);
        digitalWrite (ledPin[i], 0);
        delay(400);
      }
    }
    else if (pmt_on[i]) {
      analogWrite (ledPin[i], ledBrightness);
    }
    else {
      analogWrite (ledPin[i], 0);
    }

    v_monitor[i] = (double)analogRead(vMonitorInputPin[i]) * vdd[i] / (double)v_input_calib;
  } //for PMTID
}

void pmtWrite()
{
  if (Serial.available()) {
    String s = Serial.readStringUntil('\r');
    Serial.println(s);
    String s3 = s.substring(0, 3);
    char s4 = s[3];
    short pmtID = 0;
    if (s[3] == 'B')
      pmtID = 1;

    if (s3.equals("v_i")) //input
    {
      if (s.length() == 4)
      {
        Serial.print(v_out[pmtID]);
        Serial.print("\r");
      }
      else
      {
        int val = s.substring(4).toInt();
        counter[pmtID] = val;
        vlt_change[pmtID] = true;
      }
    }
    else if (s3.equals("v_m"))//monitor
    {
      Serial.print(v_monitor[pmtID]);
      Serial.print("\r");
    }
    else if (s3.equals("pmt")) {
      bool val = (bool)(s.substring(4).toInt());
      pmtOnOff(val, pmtID);
    }
    else if (s3.equals("pow")) {
      bool val = (bool)(s.substring(4).toInt());
      powOnOff(val, pmtID);
    }
    else if (s3.equals("aut")) {
      bool val = (bool)(s.substring(4).toInt());
      auto_pmt_recov[pmtID] = val;
    }
  }
}

void pmtOnOff(bool ON, short pmtID)
{
  if (ON) {
    powOnOff(true, pmtID);
  }
  
  digitalWrite (pmtPowPin[pmtID], ON);
  if (debug)
  {
    Serial.print("ID = ");
    Serial.print(pmtID);
    Serial.print(", PMT = ");
    Serial.println(ON);
  } 
}

void powOnOff(bool ON, short pmtID)
{
  if (!ON) {
    pmtOnOff(false, pmtID);
  }
  digitalWrite (pelPowPin[pmtID], ON);

  if (debug)
  {
    Serial.print("ID = ");
    Serial.print(pmtID);
    Serial.print(", PEL = ");
    Serial.println(ON);
  } 

}

void vltChange(short pmtID) {
  if (vlt_change[pmtID])
  {

    if (debug)
    {
      DisplayParameters(pmtID);
    }

    if (counter[pmtID] < 0)
    {
      counter[pmtID] = 0;
    }
    else if (counter[pmtID] > counterMax)
    {
      counter[pmtID] = counterMax;
    }

    //writeFlash(pmtID);

    if (pmt_err[pmtID])
      analogWrite(vltOutputPin[pmtID], 0);
    else
    {
      analogWrite(vltOutputPin[pmtID], counter[pmtID]);
      v_out[pmtID] = (1.0/6.0 + 2.0/3.0 * (double)counter[pmtID] / v_output_calib) * vdd[pmtID];
      if (debug)
      {
        Serial.print("; Voltage written = ");
        Serial.print(v_out[pmtID]);
        Serial.println();
      }
    }

  }
}

void DisplayParameters(int pmtID)
{
      Serial.print ("PMT ID = ");
      Serial.print(pmtID);
      Serial.print ("; Voltage = ");
      Serial.print (v_monitor[pmtID]);
      Serial.print("V; counter = ");
      Serial.print(counter[pmtID]);
      Serial.print("; on = ");
      Serial.print(pmt_on[pmtID]);
      Serial.print(pmt_err[pmtID]);
      Serial.print(pel_err[pmtID]);
      //Serial.print(vlt_change[pmtID]);
      //Serial.print(vltOutputPin[pmtID]);
      //Serial.println();
}

void loop()
{

  for (short i = 0; i < nPMT; i++)
    vlt_change[i] = false;
  rotaryEncoderProcess();  
  pmtRead();  
  pmtWrite();
  for (short i = 0; i < nPMT; i++)
    vltChange(i);
}
