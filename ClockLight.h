#ifndef CLOCKLIGHT_H
#define CLOCKLIGHT_H

#include <time.h>
#include <ESP8266WiFi.h>

struct sunLight {
  unsigned int rise;
  unsigned int set;
  unsigned int mlen;
};

struct Sun {
  int Rise(const int mday, const int mon);
  int Set(const int mday, const int mon);
};

struct Clock {  
  bool isFallBack(const int mday, const int mon, const int wday);
  bool isSpringForward(const int mday, const int mon, const int wday);
};


bool isBetween(const int currTime, const uint16_t from, const uint16_t till, bool overRide = false, bool randomStart = false);


#endif
