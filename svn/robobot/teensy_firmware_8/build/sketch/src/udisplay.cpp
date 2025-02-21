#line 1 "/home/local/svn/robobot/teensy_firmware_8/src/udisplay.cpp"
/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
 *   jca@elektro.dtu.dk            
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
#include "udisplay.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uirdist.h"
#include "uimu2.h"
#include "umotor.h"
#include "uservice.h"
#include "uusbhost.h"
#include "uasenc.h"

/// Sharp IR distance sensor interface object.
UDisplay display;

void UDisplay::setup()
{
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  if (dss == nullptr)
  {
    // if (robot.robotHWversion == 8)
    //   // version 5.0 (green board) has display on I2C 0
    //   dss = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 1000000);
    // else
      // default to 5.1
      // version 5.1 (purple board) has display on I2C 1
      dss = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, I2C_CLOCK); //, 1000000, 1000000);
  }  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!dss->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
  {
    usb.send("# UDisplay::setup: SSD1306 allocation failed\n");
  }
  Wire.setClock(1000000);
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //
  // display all 16/32 elements (4 lines each 32/16 pixels wide (32/16 bytes send in each block)
  for (int i=0; i < 4; i++)
  { // show splash
    // for (int c = 0; c < 4; c++) // send in bigger chunks (32 bytes)
    for (int c = 0; c < 8; c++) // send in smaller chunks (16 bytes)
      dss->display(i, c);
  }
  delay(400); // Pause for 400 ms seconds
  //
  #endif
  addPublistItem("display", "Get current display text");
  usb.addSubscriptionService(this);
}


void UDisplay::tick()
{
  tickCnt++;
// #if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  if (useDisplay)
  {
    bool fast = tickCnt % 26 == 0;
    bool slow = tickCnt % 155 == 0;
    if (fast or slow)
    {
      dss->clearDisplay();
      dss->setCursor(42,16);             // Start at row 8 (line 2), 16 = row 3
      dss->setCursor(0,8);             // Start at row 8 (line 2), 16 = row 3

  //     dss->setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //     dss->println(3.141592);
      
      dss->setTextSize(2);             // Draw 1x or 2X-scale text
      dss->setTextColor(SSD1306_WHITE);
      snprintf(lineName, MAX_LINE_LENGTH, "%d %s\n", robot.deviceID, robot.getRobotName());
      lineName[10] = '\0';
      dss->print(F(lineName));
      dss->setTextSize(1);             // Normal 1:1 pixel scale
      dss->setTextColor(SSD1306_WHITE);        // Draw white text
      dss->setCursor(0,24);             // Start at row 24
      char useIr = 'D';
      if (irdist.useDistSensor)
        useIr = 'd';
      char useImu = 'I';
      if (imu2.imuAvailable > 0)
        useImu = 'i';
      char m1ok = 'm', m2ok = 'm';
      if (not motor.m1ok)
        m1ok = 'M';
      if (not motor.m2ok)
        m2ok = 'M';
      char usbok;
      if (usb.usbIsUp)
        usbok = 'u';
      else
        usbok = 'U';
      char joy;
      if (usbhost.manOverride)
        joy = 'g';
      else
        joy = 'G';
      char useASenc = 'A';
      if (asenc.asencValid[0])
        useASenc = 'a';
      snprintf(lineState, MAX_LINE_LENGTH, "%4.1f %4.1fV %d %c%c%c%c%c%c%c",
               float(service.time_us % 100000000) * 1e-6,
               robot.batteryVoltage,
               robot.robotHWversion /*control.missionState*/,
               m1ok, m2ok,
               usbok,
               useImu,
               useIr, joy, useASenc);
      lineState[MAX_LINE_LENGTH-1] = '\0';
      dss->println(F(lineState));    
      dss->setCursor(0,0);             // Start at top-left corner
      dss->println(F(lineFree));
      if (fast)
      {
        dss->display(3, updFastCol);
        // old or new requires corresponding change in Adafruit_SSD1306_mod.cpp line 1008 + 1022..1026
        // updFastCol = (updFastCol + 1) % 4; // old - send 32 bytes in one burst
        updFastCol = (updFastCol + 1) % 8; // send 16 bytes in one burst
      }
      else if (slow)
      {
        dss->display(updSlowLine, updSlowCol++);
        // if (updSlowCol >= 4) // old - send 32 bytes in one burst
        if (updSlowCol >= 8)
        { // send 16 bytes in one burst
          updSlowCol = 0;
          updSlowLine = (updSlowLine + 1) % 3; // slow is first 3 lines only
        }
      }
    }
  }
// #endif
//   subscribeTick();
}


bool UDisplay::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "disp ", 5) == 0)
  {
    const char * p1 = &buf[5];
    setLine(p1);
  }
  else if (strncmp(buf, "dispon ", 7) == 0)
  {
    const char * p1 = &buf[7];
    int v = strtol(p1, nullptr, 10);
    useDisplay = v == 1;
  }
  else
    used = false;
  return used;
}

void UDisplay::sendData(int item)
{
  if (item == 0)
    sendDisplayLines();
}


void UDisplay::sendHelp()
{
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  usb.send("# Display -------\r\n");
  usb.send("# -- \tdisp text \tSet a display line text\r\n");
  usb.send("# -- \tdispon V \tenable (1) or disable (0) display\r\n");
  #endif
}

void UDisplay::sendDisplayLines()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "# '%s';'%s';'%s'\r\n", lineName, lineState, lineFree);
  usb.send(s);
}

void UDisplay::setLine(const char* line)
{
  strncpy(lineFree, line, MAX_LINE_LENGTH);
  lineFree[MAX_LINE_LENGTH-1] = '\0';
}


/////////////////////////////////////

void UDisplay::eePromSave()
{
  uint8_t v = useDisplay == 1;
  eeConfig.pushByte(v);
}

/////////////////////////////////////

void UDisplay::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    int f = eeConfig.readByte();
    if (f & 0x01)
      useDisplay = true;
    else
      useDisplay = false;
  }
  else
    // display is robot specific
    eeConfig.skipAddr(1);
}

