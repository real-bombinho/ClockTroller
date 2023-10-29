#include "ClockLight.h"
//#include <time.h>

sunLight AnnualSunLight[12] = { // at 21st of each month, starting Jan
{8*60 + 26, 16*60 + 26, 31},
{7*60 + 23, 17*60 + 33, 31},
{6*60 + 11, 18*60 + 32, 28},
{4*60 + 50, 19*60 + 36, 31},
{3*60 + 47, 20*60 + 35, 30},
{3*60 + 24, 21*60 +  9, 31},
{3*60 + 56, 20*60 + 46, 30},
{4*60 + 56, 19*60 + 40, 31},
{5*60 + 57, 18*60 + 18, 31},
{6*60 + 57, 17*60 + 01, 30},
{8*60 + 02, 15*60 + 59, 31},
{8*60 + 43, 15*60 + 43, 30}};

static uint8_t randomStartDelay = 0;

int Sun::Rise(const int mday, const int mon) {
  if (mday == 21) {
    return AnnualSunLight[mon].rise;
  }
  else {
    int r2 = AnnualSunLight[mon].rise;
    int d;
    if (mday < 21) {
      int i = (mon + 11) % 12; // prior month roll over
      int r1 = AnnualSunLight[i].rise;
      d = (r2 - r1) * -(21 - mday) / AnnualSunLight[i].mlen;
    }
    else {
      int i = (mon + 13) % 12; // next month roll over
      int r1 = AnnualSunLight[mon].rise;
      d = (r2 - r1)* (mday - 21) / AnnualSunLight[mon].mlen;
    }
    uint8_t b = (r2 + d) / 60;
    return r2 + d + (b * 40);
  }
}

int Sun::Set(const int mday, const int mon) {
  if (mday == 21) {
    return AnnualSunLight[mon].set;
  }
  else {
    int r2 = AnnualSunLight[mon].set;
    if (mday < 21) {
      int i = (mon + 11) % 12;
      int r1 = AnnualSunLight[i].set;
      int d = (r2 - r1)* (21 - mday) / AnnualSunLight[i].mlen;
      uint8_t b = (r2 - d) / 60;
      return r2 - d + (b * 40);
    }
    else {
      int i = (mon + 13) % 12;
      int r1 = AnnualSunLight[mon].set;
      int d = (r2 - r1)* (mday - 21) / AnnualSunLight[mon].mlen;
      uint8_t b = (r2 + d) / 60;
      return r2 + d + (b * 40);
    }
  }
}

// end of Sun //////////////////////////////////////////////////////////////////////////////////////////////////

bool Clock::isFallBack(const int mday, const int mon, const int wday) {
  return ((mon == 9) && (mday > 24) && (wday == 0)); // mon == 9 -> October
}

bool Clock::isSpringForward(const int mday, const int mon, const int wday) {
  //Serial.printf("%i/%i weekday: %i\n", mday, mon + 1, wday);
  return ((mon == 2) && (mday > 24) && (wday == 0)); // mon == 2 -> March
}

// end of Clock ////////////////////////////////////////////////////////////////////////////////////////////////

bool isBetween(const int currTime, const uint16_t from, const uint16_t till, bool overRide, bool randomStart) {
  if (overRide) return true;
  uint8_t b = currTime / 60;
  uint16_t c = currTime + (b*40);
//  Serial.printf("Hours current = %i",b);
//  Serial.println("");
  uint16_t f = from;
  if (randomStart) {
    f = f + randomStartDelay;}
  if (from < till) {
    return ((c > f) && (c < till));
  }
  else {
    return !((c > till) && (c < f));
  }
}
