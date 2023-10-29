///////////////////////  W A R N I N G  //////////////////////////
// 
//  does not work with ESP8266 Community version 3.1.2

#include <ESP8266WiFi.h>
#include <TinyGPS.h>
#include <time.h>
#include <sys/time.h>
#include <SoftwareSerial.h>
#include "NTC.h"
#include "GoN_Relay.h"
#include "ClockLight.h"

#ifndef GoN_LED_H
  #define LED_RED   5 // PIN_D1
  #define LED_GREEN 4 // PIN_D2
#endif

#define RELAY_1   12 // PIN_D6
#define RELAY_2   13 // PIN_D7

/////////// Needs work at ModBusRelay ////////////////////////
// settings doubled up in there

#define RXPin     14 // PIN_D5  // Serial Receive pin (GPS) 
#define TXPin      2 // PIN_D4  // Serial Transmit pin (GPS)

//////////////////////////////////////////////////////////////

#define sOK                0x00
#define sLowTemperature    0x01
#define sMissingThermistor 0x01 << 1
#define MinimumTime 5*60      // prevents rapid switching

Relay relay1(RELAY_1);
Relay relay2(RELAY_2);

//struct ScheduledTimes {
time_t lastTimeFetched = 0;
bool lastTimeFailed = true;
time_t lastGPSCheck = -(10 * 60);
//};

uint8_t volatile g_Status = sOK;

float volatile sensorTemperature;

const char * Version = "ver = 0.0.3<br>";

TinyGPS gps;
SoftwareSerial ss(RXPin, TXPin);  //communicate with GPS

uint32_t RTCmillis() {
  // system_get_rtc_time() is in use (but very inaccurate anyway)
  return (system_get_rtc_time() * (system_rtc_clock_cali_proc() >> 12)) / 1000;
}

void fpm_wakeup_cb_func(void) {     // called after wakeup from light sleep
  wifi_fpm_close();
  Serial.println("awoken from light sleep");
  Serial.flush();
}

bool SyncWithGPS()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
       // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
  }  
  gps.stats(&chars, &sentences, &failed);
  if (chars == 0) {
    Serial.println("** No characters received from GPS: check wiring **");
    return (false);
  }  
  else { 
    int y;
    byte h, m, s, mon, d, hundredths;
    unsigned long age;
  
    gps.crack_datetime(&y, &mon, &d, &h, &m, &s, &hundredths, &age); // get time from GPS
    if (age < 1000 or age > 3000)                             // dont use data older than 1 second
    {
      struct tm timeinfo;
      timeinfo.tm_hour = h;
      timeinfo.tm_min = m;
      timeinfo.tm_sec = s;  
      timeinfo.tm_mday = d;
      timeinfo.tm_mon = mon - 1;
      timeinfo.tm_year = y - 1900;  
  
      time_t t = mktime(&timeinfo);
      
      struct timeval now = { .tv_sec = t };
      settimeofday(&now, NULL);
  
      Serial.print("\nTime from GPS: ");
      Serial.print(h);
      Serial.print(":");
      Serial.print(m);
      Serial.print(":");
      if (s < 10) {Serial.print("0");}; 
      Serial.print(s);
      Serial.print(" -- ");
      Serial.print(d);
      Serial.print("/");
      Serial.print(mon);
      Serial.print("/");
      Serial.println(y);
      Serial.print("Time from RTC: ");
      Serial.println(asctime(&timeinfo));
      
      if (y > 2020) return (true);
    }
    else
    {
      Serial.print("Age: ");
      Serial.println(age);
    }
  }
  return (false);
}

void blink(uint8_t led, int onTime, int offTime, int repetitions) {
  for (int i = 0; i < repetitions; i++) {
    digitalWrite(led, LOW);
    delay(onTime);
    digitalWrite(led, HIGH);
    delay(offTime);  
  }
}

 
void setup() {
//  pinMode(RXPin, INPUT ); // force it? Enable if continuous freezes occur
 
  Serial.begin(74880);
  ss.begin(9600);
  while (!Serial){};
  Serial.println("Hello!"); 

  WiFi.mode(WIFI_OFF); 

  pinMode(LED_RED, OUTPUT);      // LED pin as output.
  pinMode(LED_GREEN, OUTPUT);    // LED pin as output.
  
  lastTimeFetched = time(nullptr);
  if (lastTimeFetched > 10000) {
    blink(LED_GREEN, 500, 500, 5);
  }
  else
  {
    blink(LED_RED, 500, 500, 5);
  }
  String myString; 
   myString = "ClockTroller has just restarted \r\n - ";

}

void loop() {

  static const int sleepTime = 60; //seconds
  static int loopCounter = 0;
  time_t now;

  loopCounter++;
//  if (loopCounter > 1) {
    if (SyncWithGPS()) lastGPSCheck = now;
//    loopCounter = 0;
//  }

// get current time

  struct tm timeinfo;
  now = time(nullptr);
  if ((lastTimeFailed && (now > (lastTimeFetched + (1 * 60 * 60))) ) || 
    (now > (lastTimeFetched + (24 * 60 * 60)))) {
      lastTimeFailed = false;
  }
  else {
    lastTimeFailed = true;
  }

  sensorTemperature = temperature();
  if (sensorTemperature == -400) {
    g_Status |= sMissingThermistor;  // set missing thermistor state
  }  
 else Serial.println(sensorTemperature);
  
  gmtime_r(&now, &timeinfo);

  static int tc;
  tc = (timeinfo.tm_hour * 60) + timeinfo.tm_min;


  // if ((now < lastGPSCheck) || (now > (lastGPSCheck + (10 * 60) ))) {
  //   if (SyncWithGPS()) lastGPSCheck = now;
  // }

// Adjust for BST/GMT ///////////////////////////////////////////////

  Clock c;
  if ((c.isFallBack(timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_wday) &&
    isBetween(tc, 200, 301))) {
      relay1.setState(HIGH);
  }
  else
    if ((c.isSpringForward(timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_wday) &&
      isBetween(tc, 0, 1101))) {
        relay1.setState(HIGH);
    }
  else
    relay1.setState(LOW);  

// Clock lights ////////////////////////////////////////////////////
  Sun sun;
//  Serial.printf("Sunrise today is %i, sun set is %i\n", sun.Rise(timeinfo.tm_mday, timeinfo.tm_mon),
//    sun.Set(timeinfo.tm_mday, timeinfo.tm_mon));
  if ((relay1.getState() == LOW) && isBetween(tc, sun.Rise(timeinfo.tm_mday, timeinfo.tm_mon),
    sun.Set(timeinfo.tm_mday, timeinfo.tm_mon))) {
      relay2.setState(LOW);  // switch on light
    }
    else {
      relay2.setState(HIGH); // off
    }

// Status LED signing

  int flashTime = 0;
  if ((g_Status & sMissingThermistor) != 0) {
    blink(LED_RED, 100, 100, 1);
    blink(LED_GREEN, 100, 100, 2);
    flashTime = 200 + 400;
  } 
  if (g_Status == sOK) {
    blink(LED_GREEN, 100, 1, 1);
    flashTime = 101;
  }
  Serial.flush();
  extern os_timer_t *timer_list;
  timer_list = nullptr;

// enable light sleep
  wifi_station_disconnect();
  wifi_set_opmode_current(NULL_MODE);
  
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_set_wakeup_cb(fpm_wakeup_cb_func);

// sleep for x seconds
  long sleepTimeMilliSeconds = sleepTime * 1000 - flashTime + 650;  //adjust last figure for higher precision
// light sleep function requires microseconds
  int err = wifi_fpm_do_sleep(sleepTimeMilliSeconds * 1000);

// timed light sleep is only entered when the sleep command is
// followed by a delay() that is at least 1ms longer than the sleep
  delay(sleepTimeMilliSeconds + 1);

  if (err != 0) {
    Serial.print("ERROR: ");
    Serial.println(err);
  }  
}
