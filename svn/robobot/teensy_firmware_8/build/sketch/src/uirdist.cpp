#line 1 "/home/local/svn/robobot/teensy_firmware_8/src/uirdist.cpp"
/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
 *   jcan@dtu.dk
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include "main.h"
#include "uirdist.h"
#include "ueeconfig.h"
#include "uad.h"

/// Sharp IR distance sensor interface object.
UIrDist irdist;

void UIrDist::setup()
{
  pinMode ( PIN_POWER_IR, OUTPUT ); // line sensor LED half power (HW2) - or power to IR (with new power board) (output anyhow)
  digitalWriteFast ( PIN_POWER_IR, false );
  //
  addPublistItem("ir", "Get IR and calibration data 'ir dist1(m) dist2(m) raw1 raw2 cal1-13 cal1-50 cal2-13 cal2-50 use");
  addPublistItem("ird", "Get IR distance 'ird dist1(m) dist2(m) sensorOn");
  usb.addSubscriptionService(this);
}


void UIrDist::tick()
{
  estimateIrDistance();
//   subscribeTick();
}


bool UIrDist::decode(const char* buf)
{
  bool used = true;
//   if (subs[0]->decode(buf))
//     sendStatusDistIR();
  if (strncmp(buf, "iron ", 5) == 0)
  {
    const char * p1 = &buf[5];
    uint8_t v = strtol(p1, (char**)&p1, 10);
    setIRpower(v != 0);
    usb.send("# ir set\n");
  }
  else if (strncmp(buf, "irc ", 4) == 0)
  { // 
    char * p1 = (char *)&buf[4];
    irCal13cm[0] = strtol(p1, &p1, 10);
    irCal50cm[0] = strtol(p1, &p1, 10);
    irCal13cm[1] = strtol(p1, &p1, 10);
    irCal50cm[1] = strtol(p1, &p1, 10);
    setIRpower(strtol(p1, &p1, 10));
    usb.send("# ir calibrate\n");      
    //
    calibrateIr();
  }
  else
    used = false;
  return used;
}

void UIrDist::sendData(int item)
{
  if (item == 0)
    sendStatusDistIR();
  else if (item == 1)
    sendStatusDistIR2();
}


void UIrDist::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# IR distance -------\r\n");
  snprintf(reply, MRL, "# -- \tirc A13 A50 B13 B50 V\tSet calibration values: Axx=sensor 1 (side), xx=distance in cm; Bxx=Sensor 2 (front); V=1 in on\r\n");
  usb.send(reply);
  usb.send("# -- \tiron V \tTurn IR sensor on or off V=1 for on (0=off)\r\n");
}


void UIrDist::sendStatusDistIR()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL, "ir %.3f %.3f %lu %lu %lu %lu %lu %lu %d\r\n" ,
           irDistance[0], irDistance[1],
           irRaw[0], irRaw[1],
           irCal13cm[0], irCal50cm[0],
           irCal13cm[1], irCal50cm[1],
           useDistSensor
  );
  usb.send(reply);
}

void UIrDist::sendStatusDistIR2()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL, "ir %.3f %.3f %lu %lu %d\r\n" ,
           irDistance[0], irDistance[1],
           irRaw[0], irRaw[1],
           useDistSensor
  );
  usb.send(reply);
}

///////////////////////////////////////////////////////

void UIrDist::calibrateIr()
{
  // 13cm - 50cm calibration
  // inverse of distance is linear, if measured from GP2Y0A21 base
  // 2 measurements ad d1=13cm and d2=50cm - (scaled by 10000 in c-code)
  // corresponding values v1=irCal20cm, v2 = irCal50cm
  // as inclination:
  // dri = (1/d1 - 1/d2)/(v1 - v2)
  // any value r(x) for an AD measurement x is then
  // 1/r(x) = dri * x - dri * v2 + 1/d2, reduced to
  // 1/r(x) = irA * x - irB, or 
  // r(x ) = 1/(irA * x - irB)
  //
  // calibration fixed distances - inverted and scaled
  const int32_t d1 = 10000/13; // cm
  const int32_t d2 = 10000/50; // cm
  for (int i = 0; i < 2; i++)
  { // calculate constants for both sensors
    const int32_t v1 = irCal13cm[i];
    const int32_t v2 = irCal50cm[i];
    float dri = float(d1-d2)/float(v1-v2);
    // and scaled back to meters^-1
    irA[i] = dri / 100.0; // inclination
    irB[i] = (dri * v2 - d2) / 100.0; // offset
    // debug
    //         if (hbTimerCnt %200 == 0)
    if (false)
    {
      const int MSL = 90;
      char s[MSL];
      snprintf(s, MSL, "# ir%d, irA=%g, irB=%g, dri=%g\n", i, irA[i], irB[i], dri);
      usb.send(s);
    }
    // debug 2
  }
}

////////////////////////////////////////////////


/////////////////////////////////////////////////////////

void UIrDist::estimateIrDistance()
{
  if (useDistSensor)
  { // dist sensor has power, so estimate
    // sensor uses 28ms, AD every 1ms, so filter over 10 samples
    // removes some noise from wire
    if (initIrFilter)
    { // when IR is first turned on
      irRaw[0] = ad.irRawAD[0] * 10;
      irRaw[1] = ad.irRawAD[1] * 10;
      initIrFilter = false;
    }
    else
    {
      irRaw[0] = (irRaw[0]*9)/10 + ad.irRawAD[0];
      irRaw[1] = (irRaw[1]*9)/10 + ad.irRawAD[1];
    }
    irDistance[0] = 1.0/(float(irRaw[0]) * irA[0] - irB[0]);
    irDistance[1] = 1.0/(float(irRaw[1]) * irA[1] - irB[1]);
    if (irDistance[0] > 1.5 or irDistance[0] < 0.05)
      irDistance[0] = 1.5;
    if (irDistance[1] > 1.5 or irDistance[1] < 0.05)
      irDistance[1] = 1.5;
  }
  else
  { // not installed or not on (set to far away 10m)
    irDistance[0] = 10.0;
    irDistance[1] = 10.0;
  }
}



void UIrDist::setIRpower(bool power)
{
  if (power and not useDistSensor)
  { // initialize average filter
    initIrFilter = true;
  }
  useDistSensor = power;
  //
  digitalWriteFast(PIN_POWER_IR, useDistSensor);
}


/////////////////////////////////////

void UIrDist::eePromSave()
{
  uint8_t f = 0;
  if (useDistSensor) f = 1 << 0;
  eeConfig.pushByte(f);
  eeConfig.pushWord(irCal13cm[0]);
  eeConfig.pushWord(irCal13cm[1]);
  eeConfig.pushWord(irCal50cm[0]);
  eeConfig.pushWord(irCal50cm[1]);
}

/////////////////////////////////////

void UIrDist::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    int f = eeConfig.readByte();
    bool use = f & (1 << 0);
    setIRpower(use);
    // old calibration values - set default
    irCal13cm[0] = eeConfig.readWord();
    irCal13cm[1] = eeConfig.readWord();
    irCal50cm[0] = eeConfig.readWord();
    irCal50cm[1] = eeConfig.readWord();
    calibrateIr();
  }
  else
  { // skip the same number of bytes
    int skip = 1 + 4 * 2;
    eeConfig.skipAddr(skip);
  }
}

