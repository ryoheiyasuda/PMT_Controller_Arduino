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

*/

#include <EEPROM.h>

const short nPMT = 2; //number of PMT's connected to arduino 

//Pins 
const short pelPowPin[nPMT] = {3,5}; //External connector pin number: 3 
const short pelErrPin[nPMT] = {6,7}; //External connector pin number: 5 
const short pmtPowPin[nPMT] = {8,9}; //External connector pin number: 7  
const short pmtErrPin[nPMT] = {10,11}; //External connector pin number: 9 
const short pmtOnPin[nPMT] = {12,13}; //External connector pin number: 11
const short vltOutputPin[nPMT] = {14,15}; //External connector pin number: 17 
const short vMonitorInputPin[nPMT] = {A0, A1}; //External connector pin number: 15 
const short fanPowPin[nPMT] = {18,19}; //External connector pin number: 19
const short vMonitorOutputPin[nPMT] = {20,21}; //Output for vMonitorInput.
const short ledPin[nPMT] = {22,23}; //Signal for PMT on/off and flickers during error 
const short ledGNDPin[nPMT] = {24,25};   

//Rotary encoder and counter
short counter[nPMT] = {0, 0};
short rotaryCurrentState[nPMT] = {0, 0};
short rotaryLastState[nPMT] = {0, 0};
const short rotaryPinA[nPMT] = {50, 51};  
const short rotaryPinB[nPMT] = {52, 53};
const short counterAddress[nPMT] = {2, 4}; //stores counter in EEPROM
const short counterMax = 50; //maximum voltage 0.9V is roughly 50 in 8-bit

//PMT states
bool pmt_err[nPMT] = {false, false};
bool pel_err[nPMT] = {false, false};
bool pmt_on[nPMT] = {false, false};
bool vlt_change[nPMT] = {true, true};
bool auto_pmt_recov[nPMT] = {false, false}; //Auto-recovery on off.

//adjustments
double v_monitor[nPMT] = {0, 0};
double v_input_calib[nPMT] = {1024.0, 1024.0}; //10 bit input. Pretty noisy.
double v_output_calib[nPMT] = {256.0, 256.0}; //8 bit output
double vdd[nPMT] = {5.0, 5.0};
int ledBrightness = 20; //of 255 

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
    pinMode (ledGNDPin[i], OUTPUT);
    digitalWrite (pmtPowPin[i], LOW);
    digitalWrite (pelPowPin[i], LOW);
    digitalWrite (fanPowPin[i], LOW);
    digitalWrite (ledGNDPin[i], LOW);
   }
  Serial.begin (9600);
  for (short i = 0; i < nPMT; i++)
  {
    rotaryLastState[i] = digitalRead(rotaryPinA[i]);
    EEPROM.get(counterAddress[i], counter[i]);
    vltChange(i);
  }
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
  
    if (pmt_err[i]) {
      //Serial.println("pmt error");
      if (auto_pmt_recov[i])
      {
        digitalWrite (pmtPowPin[i], LOW);
        delay(100);
        digitalWrite (pmtPowPin[i], HIGH);
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
    v_monitor[i] = sum_vol[i] * vdd[i] / v_input_calib[i] / iter;
    sum_vol[i] = 0;
    sum_count[i] = 0;
    analogWrite (vMonitorOutputPin[i], v_monitor[i] * v_output_calib[i] / vdd[i]);
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
        Serial.println((double)counter[pmtID] / v_output_calib[pmtID] * vdd[pmtID]);
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
      Serial.println(v_monitor[pmtID]);

    }
    else if (s3.equals("pmt")) {
      int val = s.substring(4).toInt();
      if (val == 1) {
        pmtOnOff(true, pmtID);
      }
      else {
        pmtOnOff(false, pmtID);
      }
    }
    else if (s3.equals("pow")) {
      bool val = (bool)(s.substring(4).toInt());
      powerOnOff(val, pmtID);
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

void powerOnOff(bool ON, short pmtID)
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
    EEPROM.put(counterAddress[pmtID], counter[pmtID]);
    analogWrite(vltOutputPin[pmtID], counter[pmtID]);
    //Serial.print("PMT id = ");
    //Serial.print(pmtID);
    //Serial.print("counter = ,");
    //Serial.println(counter[pmtID]);
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
