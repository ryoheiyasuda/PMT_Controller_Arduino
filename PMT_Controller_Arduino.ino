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
const short fanPowPin[nPMT] = {40, 33}; //External connector pin number: 19

//use for monitor panel.
const short vMonitorOutputPin[nPMT] = {3, 5}; //Output for vMonitorInput. --- Analog output (use 2- 13)

const short ledPin[nPMT] = {8, 9}; //Signal for PMT on/off and flickers during error use 2-13.

//Rotary encoder pin
const short rotaryPinA[nPMT] = {40, 41};
const short rotaryPinB[nPMT] = {42, 43};


//Voltage calibration
const int v_input_calib = 1024; //10 bit input. Pretty noisy.
const double vdd[nPMT] = {5.0, 5.0};

#if defined(ARDUINO_SAM_DUE) or defined(TEENSYDUINO)
const short n_bits = 10; //used only for Due.
const int v_output_calib = 1024; //10 bit output
#else
const short n_bits = 8; //
const int v_output_calib = 256; //8 bit output
#endif

//Rotaty encoder state values
short counter[nPMT] = {0, 0};
short rotaryCurrentState[nPMT] = {0, 0};
short rotaryLastState[nPMT] = {0, 0};
short counterMax = v_output_calib / 5; //maximum voltage ~1V

//EEPROM / flash saving
short counterAddress[nPMT] = {2, 4}; //stores counter in Flash

//PMT states
bool pmt_err[nPMT] = {false, false};
bool pel_err[nPMT] = {false, false};
bool pmt_on[nPMT] = {false, false};
bool vlt_change[nPMT] = {true, true};
bool auto_pmt_recov[nPMT] = {false, false}; //Auto-recovery on off.

//adjustments
double v_monitor[nPMT] = {0, 0};
int ledBrightness = v_output_calib * 0.2;

//For averaging voltage input.
double sum_vol[nPMT] = {0, 0};
int sum_count[nPMT] = {0, 0};
int iter = 50; //number of iteration for average voltage for arduino

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
  }

  Serial.print("Initialized\r");
}

void readFlash(short pmtID)
{
#if use_flash == 1
  byte* b2 = dueFlashStorage.readAddress(counterAddress[pmtID]);
  memcpy(b2, &(counter[pmtID]), 2);
#else
  EEPROM.get(counterAddress[pmtID], counter[pmtID]);
#endif
}

void writeFlash(short pmtID)
{
#if use_flash == 1
  byte b2[2];
  memcpy(&(counter[pmtID]), b2, 2);
  dueFlashStorage.write(counterAddress[pmtID], b2, 2);
#else
  EEPROM.put(counterAddress[pmtID], counter[pmtID]);
#endif
}

void rotaryEncoderProcess()
{
  for (short i = 0; i < nPMT; i++)
  {
    rotaryCurrentState[i] = digitalRead(rotaryPinA[i]);
    if (rotaryCurrentState[i] != rotaryLastState[i]) {
      if (digitalRead(rotaryPinB[i]) != rotaryCurrentState[i]) {
        counter[i] ++;
      } else {
        counter[i] --;
      }
      vlt_change[i] = true;
    }
    rotaryLastState[i] = rotaryCurrentState[i];
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
      //Serial.print("pmt error\r");

      vlt_change[i] = true;

      if (auto_pmt_recov[i])
      {
        pmtOnOff(false, i);
        delay(50);
        powOnOff(false, i);
        delay(50);
        powOnOff(true, i);
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


    sum_count[i] ++;
    sum_vol[i] += (double)analogRead(vMonitorInputPin[i]);
    if (sum_count[i] == iter)
    {
      v_monitor[i] = sum_vol[i] * vdd[i] / (double)v_input_calib / (double)iter;
      sum_vol[i] = 0;
      sum_count[i] = 0;
      if (pmt_err[i])  {
        analogWrite (vMonitorOutputPin[i], 0);
      }
      else
        analogWrite (vMonitorOutputPin[i], v_monitor[i] * v_output_calib / vdd[i]);
    }
    delay(1);
  }
}

void pmtWrite()
{
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    String s3 = s.substring(0, 3);
    char s4 = s[3];
    short pmtID = 0;
    if (s[3] == 'B')
      pmtID = 1;

    if (s3.equals("v_i")) //input
    {
      if (s.length() == 4)
      {
        Serial.print((double)counter[pmtID] / v_output_calib * vdd[pmtID]);
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
  digitalWrite (pmtPowPin[pmtID], ON);
}

void powOnOff(bool ON, short pmtID)
{
  if (!ON) {
    pmtOnOff(false, pmtID);
  }
  digitalWrite (pelPowPin[pmtID], ON);
}

void vltChange(short pmtID) {
  if (vlt_change[pmtID])
  {
    if (counter[pmtID] < 0)
      counter[pmtID] = 0;
    else if (counter[pmtID] > counterMax)
      counter[pmtID] = counterMax;
      
    writeFlash(pmtID);

    if (pmt_err)
      analogWrite(vltOutputPin[pmtID], 0);
    else
      analogWrite(vltOutputPin[pmtID], counter[pmtID]);
      
    //Serial.print("PMT id = ");
    //Serial.print(pmtID);
    //Serial.print("counter = ,");
    //Serial.print(counter[pmtID]);
    //Serial.print("\r");
  }
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
