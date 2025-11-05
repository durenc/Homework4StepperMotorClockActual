#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

const int EN    = 14;
const int STEP1 = 9;
const int DIR1  = 10;
const int STEP2 = 11;
const int DIR2  = 12;

const int Home1 = 5;
const int Home2 = 6;

unsigned long currentHour=0;
unsigned long currentMin=0;

// rack and pinion geometry conversions
const int   STEPS_PER_REV = 200;           // full steps
const float PD_MM         = 32.0f;         // pitch diameter for 16T, m=2
const float CIRC_MM       = 3.14159265f * PD_MM;
const float SPMM          = STEPS_PER_REV / CIRC_MM; // 1.99 steps/mm

const float LEN_H_MM      = 120.0f;        // hour rack travel
const float LEN_M_MM      = 120.0f;        // minute rack travel

inline long mmToSteps(float mm) { return lround(mm * SPMM); }
//steps once
void stepOnce(int pin, int delayMicros) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(delayMicros);
  digitalWrite(pin, LOW);
  delayMicroseconds(delayMicros);
}


//sets up move to for motor
void moveToMotor1(int stepPin, int dirPin, long &pos, long target, int usHigh = 10000) {
  while (pos != target) {
    if (pos < target) {
      digitalWrite(dirPin, LOW);  
      stepOnce(stepPin, usHigh);
      pos++;
    } else {
      digitalWrite(dirPin, HIGH);
      stepOnce(stepPin, usHigh);
      pos--;
    }
  }
}

//needed second one since motors need to go opposite directions
void moveToMotor2(int stepPin, int dirPin, long &pos, long target, int usHigh = 10000) {
  while (pos != target) {
    if (pos < target) {
      digitalWrite(dirPin, HIGH); 
      stepOnce(stepPin, usHigh);
      pos++;
    } else {
      digitalWrite(dirPin, LOW);
      stepOnce(stepPin, usHigh);
      pos--;
    }
  }
}

long pos1=0;
long pos2=0;

void moveTo1(long target, int usHigh = 10000) { moveToMotor1(STEP1, DIR1, pos1, target, usHigh); }
void moveTo2(long target, int usHigh = 10000) { moveToMotor2(STEP2, DIR2, pos2, target, usHigh); }

// converts times to rack positions
long targetHourStepsFrom(float t) {
  float h_frac = (t) / 24.0f;  // 0..1
  return mmToSteps(h_frac * LEN_H_MM);
}
long targetMinuteStepsFrom(float t) {
  float m_frac = (t) / 60.0f;   // 0..1
  return mmToSteps(m_frac * LEN_M_MM);
}


#include <esp_sntp.h>

void setupTime() {
  // sets TZ + NTP servers
  configTzTime("CST6CDT,M3.2.0/2,M11.1.0/2",
               "pool.ntp.org", "time.nist.gov", "time.google.com");

  const time_t EPOCH_OK = 1700000000; 
  time_t now;
  uint32_t tries = 0;
  while ((now = time(nullptr)) < EPOCH_OK && tries < 120) { 
    delay(250);
    tries++;
  }
  if (now < EPOCH_OK) {
    Serial.println("NTP failed. Continue in manual/test mode.");
    return;
  }

  struct tm t; localtime_r(&now, &t);
  currentHour = t.tm_hour; currentMin = t.tm_min;
  moveTo1(targetHourStepsFrom(currentHour));
  moveTo2(targetMinuteStepsFrom(currentMin));
}

// Update times
void printTimeOnly() {
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);

  static int lastHour = -1, lastMinute = -1;

  if (lastHour   != -1 && t.tm_hour < lastHour)   moveTo1(0); // goes from 23 to 0
  if (lastMinute != -1 && t.tm_min  < lastMinute) moveTo2(0); // goes from 59 to 0

  currentHour = t.tm_hour;
  currentMin  = t.tm_min;
  Serial.println(currentHour);
  Serial.println(currentMin);
  moveTo1(targetHourStepsFrom(currentHour));
  moveTo2(targetMinuteStepsFrom(currentMin));

  lastHour = currentHour;
  lastMinute = currentMin;
  Serial.println(currentHour);
  Serial.println(currentMin);

}

// Parse "HH:MM" or "HH MM" from Serial
bool readTimeFromSerial(int &h, int &m) {
  if (!Serial.available()) return false;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return false;

  // replace any non-digit with space, then split
  for (int i = 0; i < s.length(); ++i) {
    char c = s[i];
    if ((c < '0' || c > '9')) s.setCharAt(i, ' ');
  }
  s.trim();
  int sp = s.indexOf(' ');
  if (sp < 0) return false;

  int hh = s.substring(0, sp).toInt();
  int mm = s.substring(sp + 1).toInt();

  if (hh < 0 || hh > 23) return false;
  if (mm < 0 || mm > 59) return false;

  h = hh; m = mm;
  return true;
}



// Testing code to move to positions entered via Serial
void printTimeTest() {
  static int lastHour = -1, lastMinute = -1;

  int h, m;
  if (!readTimeFromSerial(h, m)) return;  // nothing typed yet

  // Optional wrap-to-zero if time goes "backwards"
  if (lastHour != -1 && h < lastHour)   moveTo1(0);
  if (lastMinute != -1 && m < lastMinute) moveTo2(0);

  currentHour = h;
  currentMin  = m;

  Serial.print("Set to ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.println(currentMin);

  moveTo1(targetHourStepsFrom(currentHour));
  moveTo2(targetMinuteStepsFrom(currentMin));

  lastHour = currentHour;
  lastMinute = currentMin;
}





void homeRack(int stepPin, int dirPin, int homePin, long &pos, bool towardHomeIsLow = true) {
  // Move toward the switch until pressed
  digitalWrite(dirPin, towardHomeIsLow ? LOW : HIGH);

  Serial.print("Homing motor on pin ");
  Serial.println(stepPin);

  // Move until the home button is pressed
  while (digitalRead(homePin) == HIGH) {  
    stepOnce(stepPin, 10000); 
    digitalRead(homePin);               
  }

  // Set position to zero once switch is hit
  pos = 0;

  Serial.print("Homing complete on pin ");
  Serial.println(stepPin);
}


void setup() {
  pinMode(EN, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(Home1,INPUT_PULLUP);
  pinMode(Home2,INPUT_PULLUP);
  digitalWrite(EN, LOW);
  digitalWrite(DIR1, HIGH);   // adjust if direction is inverted
  digitalWrite(DIR2, LOW);

  delay(2000);
  Serial.begin(9600);
  delay(2000);
  Serial.println("Stepper clock rack controller starting...");
  //Uncomment out code when not testing

  Serial.print("Connecting to WiFi");
  WiFi.begin("NSA Security Van HQ", "windowstothehallway", 6);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  homeRack(STEP1, DIR1, Home1, pos1, false);
  homeRack(STEP2, DIR2, Home2, pos2, true);
  setupTime();
}

void loop() {
  //Testing code
  //Serial.println("Enter time as HH:MM or HH MM:");
  //printTimeTest();
  //delay(100);
  
  //Actual code to commment back in

  printTimeOnly();
  delay(1000);  // update once per second
  
}
