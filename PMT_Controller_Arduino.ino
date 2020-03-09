/*
    COPYRIGHT nozomi yasuda

    Serial input commands:
    ======================
    v_i%d = set voltage to %d (like v_i50)
    v_m = monitor voltage
    pmt1 = turn on pmt
    pmt0 = turn off pmt
    pow1 = turn on everything (including Peltier)
    pow0 = turn off everything (including Peltier)
    aut1 = auto-recovery upon PMT error. (default off)
    aut0 = turn off above feature.
*/

#include <EEPROM.h>

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

const int counterAddress = 2; //stores counter in EEPROM
const int counterMax = 50; //maximum voltage 0.9V is roughly 50 in 8-bit

//Rotary encoder
int counter = 0;
int aState;
int aLastState;

//PMT states
bool pmt_err = false;
bool pel_err = false;
bool pmt_on = false;
bool vlt_change = true;
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
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

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

  aLastState = digitalRead(outputA);

  EEPROM.get(counterAddress, counter);
  vltChange();
}

void rotaryEncoderProcess()
{
  aState = digitalRead(outputA);
  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {
      counter ++;
    } else {
      counter --;
    }
    vlt_change = true;
  }
  aLastState = aState;
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
    if (s3.equals("v_i")) //input
    {
      if (s.length() == 3)
      {
        Serial.println((double)counter / v_output_calib * vdd);
      }
      else
      {
        int val = s.substring(3).toInt();
        counter = val;
        vlt_change = true;
      }
    }
    else if (s3.equals("v_m"))//monitor
    {
      Serial.println(v_monitor);

    }
    else if (s3.equals("pmt")) {
      int val = s.substring(3).toInt();
      if (val == 1) {
        powerOnOff(true);
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
  digitalWrite (pelPowPin, ON);
  digitalWrite (pmtPowPin, ON);
}

void vltChange() {
  if (vlt_change)
  {
    if (counter < 0)
      counter = 0;
    else if (counter > counterMax)
      counter = counterMax;
    EEPROM.put(counterAddress, counter);
    analogWrite(vltOutputPin, counter);
    //Serial.println(counter);
  }
}

void loop()
{

  vlt_change = false;
  rotaryEncoderProcess();
  pmtRead();
  pmtWrite();
  vltChange();

}
