/*
    COPYRIGHT nozomi yasuda

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

/*
  #define outputA 50 //rotary encoder 1st pin
  #define outputB 52 //rotary encoder 2nd pin
  #define pelPowPin 4 //External connector pin number: 3
  #define pelErrPin 5 //External connector pin number: 5
  #define pmtPowPin 6 //External connector pin number: 7
  #define pmtErrPin 7 //External connector pin number: 9
  #define pmtOnPin 8 //External connector pin number: 11
  #define vltOutputPin 10 //External connector pin number: 17
  #define vMonitorInputPin A0 //External connector pin number: 15
  #define fanPowPin 12 //External connector pin number: 19

  #define vMonitorOutputPin 9 //Output for vMonitorInput.
  #define ledPin 3 //Signal for PMT on/off and flickers during error
  #define ledGNDPin 2
*/

#define pelPowPin 4 //External connector pin number: 3
#define pelErrPin 5 //External connector pin number: 5
#define pmtPowPin 6 //External connector pin number: 7
#define pmtErrPin 7 //External connector pin number: 9
#define pmtOnPin 8 //External connector pin number: 11
#define vltOutputPin 10 //External connector pin number: 17
#define vMonitorInputPin A0 //External connector pin number: 15
#define fanPowPin 12 //External connector pin number: 19

#define vMonitorOutputPin 9 //Output for vMonitorInput.
#define ledPin 3 //Signal for PMT on/off and flickers during error 
#define ledGNDPin 2

const short rotaryPinA[2] = {50, 51};  //rotary encoder 1st pin
const short rotaryPinB[2] = {52, 53};
const short nPMT = 2;
const short counterAddress[2] = {2, 4}; //stores counter in EEPROM
const short counterMax = 50; //maximum voltage 0.9V is roughly 50 in 8-bit


//Rotary encoder
short counter[2] = {0, 0};
short aState[2] = {0, 0};
short aLastState[2] = {0, 0};

//PMT states
bool pmt_err = false;
bool pel_err = false;
bool pmt_on = false;
bool vlt_change[2] = {true, true};
bool auto_pmt_recov = false; //Auto-recovery on off.

double v_monitor = 0;
double v_input_calib = 1024.0; //10 bit input. Pretty noisy.
double v_output_calib = 256.0; //8 bit output
double vdd = 5.0;
int ledBrightness = 20; //of 255

//For averaging voltage input.
double sum_vol = 0;
int sum_count = 0;
int iter = 50; //number of iteration for average voltage for arduino


void setup() {
  for (short i = 0; i < nPMT; i++) {
    pinMode (rotaryPinA[i], INPUT);
    pinMode (rotaryPinB[i], INPUT);
  }
  pinMode (pelPowPin, OUTPUT);
  pinMode (pelErrPin, INPUT);
  pinMode (pmtPowPin, OUTPUT);
  pinMode (pmtErrPin, INPUT);
  pinMode (pmtOnPin, INPUT);
  pinMode (vltOutputPin, OUTPUT); //analog
  pinMode (vMonitorOutputPin, OUTPUT); //analog
  pinMode (vMonitorInputPin, INPUT);
  pinMode (fanPowPin, OUTPUT);
  pinMode (ledPin, OUTPUT);
  pinMode (ledGNDPin, OUTPUT);


  digitalWrite (pmtPowPin, LOW);
  digitalWrite (pelPowPin, LOW);
  digitalWrite (fanPowPin, LOW);
  digitalWrite (ledGNDPin, LOW);

  Serial.begin (9600);

  for (short i = 0; i < nPMT; i++)
  {
    aLastState[i] = digitalRead(rotaryPinA[i]);
    EEPROM.get(counterAddress[i], counter[i]);
    vltChange(i);
  }
}

void rotaryEncoderProcess()
{
  for (short i = 0; i < nPMT; i++)
  {
    aState[i] = digitalRead(rotaryPinA[i]);
    if (aState[i] != aLastState[i]) {
      if (digitalRead(rotaryPinB[i]) != aState[i]) {
        counter[i] ++;
      } else {
        counter[i] --;
      }
      vlt_change[i] = true;
    }
    aLastState[i] = aState[i];
  }
}

void pmtRead()
{
  pel_err = digitalRead(pelErrPin);
  pmt_err = digitalRead(pmtErrPin);
  pmt_on = digitalRead(pmtOnPin);

  if (pmt_err) {
    //Serial.println("pmt error");
    if (auto_pmt_recov)
    {
      digitalWrite (pmtPowPin, LOW);
      delay(100);
      digitalWrite (pmtPowPin, HIGH);
    }
    else
    {
      analogWrite (ledPin, ledBrightness);
      delay(400);
      digitalWrite (ledPin, 0);
      delay(400);
    }
  }
  else if (pmt_on) {
    analogWrite (ledPin, ledBrightness);
  }
  else {
    analogWrite (ledPin, 0);
  }

  sum_count ++;
  sum_vol += (double)analogRead(vMonitorInputPin);
  if (sum_count == iter)
  {
    v_monitor = sum_vol * vdd / v_input_calib / iter;
    sum_vol = 0;
    sum_count = 0;
    analogWrite (vMonitorOutputPin, v_monitor * v_output_calib / vdd);
  }
  delay(1);
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
      if (s.length() == 3)
      {
        Serial.println((double)counter[pmtID] / v_output_calib * vdd);
      }
      else
      {
        int val = s.substring(3).toInt();
        counter[pmtID] = val;
        vlt_change[pmtID] = true;
      }
    }
    else if (s3.equals("v_m"))//monitor
    {
      Serial.println(v_monitor);

    }
    else if (s3.equals("pmt")) {
      int val = s.substring(3).toInt();
      if (val == 1) {
        pmtOnOff(true);
      }
      else {
        pmtOnOff(false);
      }
    }
    else if (s3.equals("pow")) {
      bool val = (bool)(s.substring(3).toInt());
      powerOnOff(val);
    }
    else if (s3.equals("aut")) {
      bool val = (bool)(s.substring(3).toInt());
      auto_pmt_recov = val;
    }
  }
}

void pmtOnOff(bool ON)
{
  digitalWrite (pmtPowPin, ON);
}

void powerOnOff(bool ON)
{
  if (!ON) {
    pmtOnOff(false);
  }
  digitalWrite (pelPowPin, ON);
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
