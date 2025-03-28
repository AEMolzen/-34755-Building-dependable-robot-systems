/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
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

#include <string.h>
#include "ulog.h"
// #include "motor_controller.h"
//#include "serial_com.h"
// #include "mpu9150.h"
// #include "ucontrol.h"
// #include "robot.h"
#include "main.h"
#include "ulinesensor.h"
// #include "dist_sensor.h"
#include "ueeconfig.h"
// #include "wifi8266.h"
// #include "ucontrol.h"
// #include "umission.h"

#include "uusb.h"
#include "ustate.h"
#include "uad.h"
#include "ucurrent.h"
#include "umotor.h"
#include "uencoder.h"
#include "uirdist.h"
#include "uimu2.h"
#include "umotortest.h"

ULog logger;



void ULog::setup()
{ // allocate buffer space on heap (better on Teensy 4.1)
  if (logBuffer == nullptr)
    logBuffer = (int8_t *) malloc(LOG_BUFFER_MAX);
  //
//   addPublistItem("lfc", "Get log flags for control 'lfc vel turn pos edge wall dist bal balvel balpos'");
  addPublistItem("lfl", "Get log flags 'lfl mis acc gyro mag motref motv mota enc vel turnr pose line dist batt ex chirp'");
  addPublistItem("lst", "Get log status 'lst interval rows rowsMax logSize'");
  usb.addSubscriptionService((USubss*)this);
}

bool ULog::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "lfls ", 5) == 0)
    setLogFlagsOther(buf);
  else if (strncmp(buf, "lognow ", 6) == 0)
  {
    const char * p1 = &buf[6];
    logInterval = strtol(p1, nullptr, 10);
    startLogging(logInterval, true);
  }
  else if (strncmp(buf, "lsts ", 5) == 0)
  { // temporarly set log inderval (just for timing info)
    const char * p1 = &buf[5];
    logInterval = strtol(p1, nullptr, 10);
    // convert ms to samples
    logInterval = (logInterval*100)/state.CONTROL_PERIOD_10us;
    current.logIntervalChanged();
  }
  else if (strncmp(buf, "logmsg ", 6) == 0)
  { // subscribe to the messages to log
    // then call 'logmsg 1\n'
    const char * p1 = &buf[6];
    bool logOn = true;
    // se if there is a number, and turn off if the number is '0'
    while (not isDigit(*p1) and *p1 != '\0')
      p1++;
    if (*p1 == '0')
        logOn = false;
    if (logOn)
      initLogStreamedMsg();
    else
      // mark the buffer as full
      logStreamedMsgFull = true;
  }
  else if (strncmp(buf, "log ", 3) == 0)
  {
    if (logRowCnt > 0)
    {
      logToUSB = true;
      logStreamedMsgOutPos = 0;
    }
    else if (motortest.mLogIndex > 0)
    {
      motortest.getMotorTestLog();
    }
    else
    {
      usb.send("%% log is empty\n");
      usb.send("# log is empty\n");
    }
  }
  else
    used = false;
  return used;
}

void ULog::initLogStreamedMsg()
{
  if ((char*)logBuffer != nullptr)
  { // may be nullptr in the Teensy 4.1 configuration
    // else always true
    logStreamedMsg = true;
    logStreamedMsgPos = 0;
    logStreamedMsgFull = false;
    logRowCnt = 0; // reset normal logging too
  }
}

void ULog::sendHelp()
{
  usb.send("# logger ------\n");
  usb.send("# -- \tlognow \tstart logging now\n");
  usb.send("# -- \tlog \tGet current log, if any\n");
  usb.send("# -- \tlfls \tSet log flags - same order as lfl\n");
//   usb.send("# -- \tlfcs \tSet log control log flags - same order as lfc\n");
  usb.send("# -- \tlsts \tSet log interval (for timing info only)\n");
  usb.send("# -- \tlogmsg \tStart log of (all) streamed messages (e.g. subscriptions)\n");
}


void ULog::tick()
{
  ltc = hbTimerCnt; // counts tick
  // log data at requested interval
  if ((ltc - lastTimerCnt ) >= logInterval /*or control.chirpRun*/)
  {
    bool doLog = true; // not control.chirpRun;
//     if (not doLog)
//     { // we log anyhow, if we are doing chirp modulation
//       if (control.chirpLog)
//       { // time to do a log action
//         control.chirpLog = false;
//         doLog = true;
//       }
//     }
    if (doLog)
    {
      lastTimerCnt = ltc;
      m++;
      if (loggerLogging())
      {
        stateToLog();
      }
    }
  }
  // test if log is requested
  if (logToUSB) // send log to USB or wifi
  { // signal log read using on-board LED
    state.setStatusLed ( HIGH );
    if (logger.logRowCnt > 0)
    {
      row = logger.logWriteBufferTo (row);
      row++;
      if ( row >= logger.logRowCnt ) // finished
      {
        logToUSB = false;
        row = -1;
      }
    }
    else if (logStreamedMsg)
    { // write one text line to USB
      writeBufferMsg();
      // stop when no more messages
      if (logStreamedMsgOutPos >= logStreamedMsgPos)
        logToUSB = false;
    }
    else
    {
      usb.send("%% Log buffer is empty\r\n");
      logToUSB = false;
    }
    state.setStatusLed ( LOW );
    if (not logToUSB)
      usb.send("logend\n");
  }
  
}


void ULog::setLogFlagsOther(const char * buf)
{
  const char * p1 = &buf[5];
  logRowFlags[LOG_TIME] = true; //strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_MISSION] = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_ACC] = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_GYRO] = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_MAG] = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_MOTV_REF] = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_MOTV]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_MOTA]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_ENC]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_WHEELVEL]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_TURNRATE]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_POSE]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_LINE]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_DIST]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_BATT]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_TIMING]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_EXTRA]  = strtol(p1, (char **) &p1, 10);
  logRowFlags[LOG_CHIRP]  = strtol(p1, (char **) &p1, 10);
  initLogStructure(100000 / state.CONTROL_PERIOD_10us);
}

// void ULog::setLogFlagsControl(const char * buf)
// {
//   const char * p1 = &buf[5];
//   logRowFlags[LOG_CTRL_VELL] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_TURN] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_POS] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_EDGE] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_WALL] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_FWD_DIST] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_BAL] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_BAL_VEL] = strtol(p1, (char **) &p1, 10);
//   logRowFlags[LOG_CTRL_BAL_POS] = strtol(p1, (char **) &p1, 10);
//   initLogStructure(100000 / state.CONTROL_PERIOD_10us);
// }

void ULog::sendData(int item)
{
  if (item == 0)
//     sendLogFlagsControl();
//   else if (item == 1)
    sendLogFlagsOther();
  else if (item==1)
    sendLogInfo();
}

void ULog::sendLogInfo()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "lst %d %d %d %d\r\n",
           logInterval, logRowCnt, logRowsCntMax, LOG_BUFFER_MAX);
  usb.send(s);
}

// void ULog::sendLogFlagsControl()
// {
//   const int MSL = 150;
//   char s[MSL];
//   snprintf(s, MSL, "lfc %d %d %d %d %d %d %d %d %d\r\n",
//            logRowFlags[LOG_CTRL_VELL],
//            logRowFlags[LOG_CTRL_TURN],
//            logRowFlags[LOG_CTRL_POS],
//            logRowFlags[LOG_CTRL_EDGE],
//            logRowFlags[LOG_CTRL_WALL],
//            logRowFlags[LOG_CTRL_FWD_DIST],
//            logRowFlags[LOG_CTRL_BAL],
//            logRowFlags[LOG_CTRL_BAL_VEL],
//            logRowFlags[LOG_CTRL_BAL_POS]
//   );
//   usb.send(s);
// }

void ULog::sendLogFlagsOther()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "lfl %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
//            logRowFlags[LOG_TIME],
           logRowFlags[LOG_MISSION],
           logRowFlags[LOG_ACC],
           logRowFlags[LOG_GYRO],
           logRowFlags[LOG_MAG],
           logRowFlags[LOG_MOTV_REF],
           logRowFlags[LOG_MOTV],
           logRowFlags[LOG_MOTA],
           logRowFlags[LOG_ENC],
           logRowFlags[LOG_WHEELVEL],
           logRowFlags[LOG_TURNRATE],
           logRowFlags[LOG_POSE],
           logRowFlags[LOG_LINE],
           logRowFlags[LOG_DIST],
           logRowFlags[LOG_BATT],
           logRowFlags[LOG_TIMING],
           logRowFlags[LOG_EXTRA],
           logRowFlags[LOG_CHIRP]
  );
  usb.send(s);
}

/**
 * Start logging with current log flags.
 * \param logInterval 0: unchanged, else set to this number of milliseconds
 * \param restart if true, then log is cleared
 * */
void ULog::startLogging(int loginterval, bool restart)
{
  if (loginterval > 0)
  {
    logInterval = loginterval;
    current.logIntervalChanged();
  }
  // initialize log - if not already
  if (restart)
  { // logger not active, so start
    initLogStructure(100000 / state.CONTROL_PERIOD_10us);
  }
  // start logging
  toLog = true;
}

void ULog::stopLogging(void)
{
  toLog = false;
}

//////////////////////////////////////////

void ULog::stateToLog()
{ 
  //   const int MSL = 100;
  //   char s[MSL];
  // //  snprintf(s, MSL, "# state to log, flags, time=%d, mission=%d,...\n", logRowFlags[LOG_TIME], logRowFlags[LOG_MISSION]);
  //   snprintf(s, MSL, "#time %.3f mission %d, state %d.%d, logger line %d/%d\r\n", 
  //            time, mission, missionState, misLineNum, logRowCnt, logRowsCntMax);
  //   usb.send(s);
  if (not logFull)
  {
    logFull = not addNewRowToLog();
    if (not logFull)
    {
      addToLog(LOG_TIME, &missionTime, sizeof(missionTime));
//       if (logRowFlags[LOG_MISSION])
//       {
//         int16_t mv[4];
//         int8_t * mb = (int8_t *) &mv[1];
//         uint32_t * mf = (uint32_t *) &mv[2];
//         mv[0] = control.missionState;
//         // log line number not state
//         mb[0] = control.misThread;
//         mb[1] = control.missionLineNum;
//         mf[0] = userMission.eventFlagsSavedLog;
//         userMission.eventFlagsSavedLog = 0;
//         addToLog(LOG_MISSION, mv, sizeof(mv));
//       }
      if (logRowFlags[LOG_ACC])
        addToLog(LOG_ACC, imu2.acc, sizeof(imu2.acc));
      if (logRowFlags[LOG_GYRO])
        addToLog(LOG_GYRO, imu2.gyro, sizeof(imu2.gyro));
      //       if (logRowFlags[LOG_MAG])
      //         addToLog(LOG_MAG, imuMag, sizeof(imuMag));
//       if (logRowFlags[LOG_MOTV_REF])
//         addToLog(LOG_MOTV_REF, control.vel_ref, sizeof(control.vel_ref));
      if (logRowFlags[LOG_MOTV])
      {
        int16_t mv[2];
        if (motor.motorEnable[0])
        { // motor enabled, save value in mV
          mv[0] = int(motor.motorVoltage[0] * 1000);
          mv[1] = int(motor.motorVoltage[1] * 1000);
        }
        else
        { // motor not enabled, so log a zero instead - regardless of control value
          mv[0] = 0;
          mv[1] = 0;
        }
        addToLog(LOG_MOTV, mv, sizeof(mv));
      }
      if (logRowFlags[LOG_ENC])
      { // encoder values
        addToLog(LOG_ENC, encoder.encoder, sizeof(encoder.encoder));
      }
      if (logRowFlags[LOG_MOTA])
      { // save filtered motor current
        addToLog(LOG_MOTA, current.motorCurrentA, sizeof(current.motorCurrentA));
      }
      if (logRowFlags[LOG_WHEELVEL])
        addToLog(LOG_WHEELVEL, encoder.wheelVelocityEst, sizeof(encoder.wheelVelocityEst));
      if (logRowFlags[LOG_TURNRATE])
      {
        float v = encoder.robotTurnrate;
        addToLog(LOG_TURNRATE, &v, sizeof(v)); 
      }
      if (logRowFlags[LOG_POSE])
        addToLog(LOG_POSE, encoder.pose, sizeof(encoder.pose));
      if (logRowFlags[LOG_LINE])
      { // pack to array
        int16_t ldv[22];
        float * fdv = (float*) &ldv[8];
        int8_t * lsc = (int8_t*) &ldv[19];
        uint8_t * flags = (uint8_t*) &ldv[18];
        ldv[0] = ad.adcLSH[0] - ad.adcLSL[0];
        ldv[1] = ad.adcLSH[1] - ad.adcLSL[1];
        ldv[2] = ad.adcLSH[2] - ad.adcLSL[2];
        ldv[3] = ad.adcLSH[3] - ad.adcLSL[3];
        ldv[4] = ad.adcLSH[4] - ad.adcLSL[4];
        ldv[5] = ad.adcLSH[5] - ad.adcLSL[5];
        ldv[6] = ad.adcLSH[6] - ad.adcLSL[6];
        ldv[7] = ad.adcLSH[7] - ad.adcLSL[7];
        fdv[0] = ls.lsLeftSide;  // word  8 and  9 (32 bit float) [cm]
        fdv[1] = ls.lsRightSide; // word 10 and 11 (32 bit float)
        fdv[2] = ls.findCrossingLineVal; // word 12 og 13
        fdv[3] = 0; //whiteVal; // is actually also black value, if looking for black, word 14 and 15
        fdv[4] = ls.edgeAngle; // word 16 and 17
        *flags = 0;
        if (ls.lineSensorOn)
          *flags = 1;
        if (ls.lsIsWhite)
          *flags |= 2;
        if (ls.lsEdgeValid)
          *flags |= 4;
        if (ls.lsEdgeValid)
          *flags |= 8;
        if (ls.lsPowerHigh)
          *flags |= 0x10;
        if (ls.lsTiltCompensate)
          *flags |= 0x20;
        lsc[0] = ls.crossingLineCnt; // word 17
        lsc[1] = ls.lsEdgeValidCnt;
//         lsc[2] = ls.lsIdxLow;
//         lsc[3] = ls.lsIdxHi;
        addToLog(LOG_LINE, &ldv, sizeof(ldv));
      }
      if (logRowFlags[LOG_DIST])
      {
        addToLog(LOG_DIST, irdist.irRaw, sizeof(irdist.irRaw));
      }
      if (logRowFlags[LOG_BATT])
        addToLog(LOG_BATT, &ad.batVoltRawAD, sizeof(ad.batVoltRawAD));
      if (logRowFlags[LOG_TIMING])
      {
        addToLog(LOG_TIMING, &state.cycleTime2, sizeof(state.cycleTime2));
      }
//       if (logRowFlags[LOG_CTRLTIME])
//         addToLog(LOG_CTRLTIME, &state.cycleTime, sizeof(state.cycleTime));
//       // internal control values
//       if (logRowFlags[LOG_CTRL_VELL])
//         control.ctrlVelLeft->toLog(LOG_CTRL_VELL);
//       if (logRowFlags[LOG_CTRL_VELR])
//         control.ctrlVelRight->toLog(LOG_CTRL_VELR);
//       if (logRowFlags[LOG_CTRL_TURN])
//         control.ctrlTurn->toLog(LOG_CTRL_TURN);
//       if (logRowFlags[LOG_CTRL_POS])
//         control.ctrlPos->toLog(LOG_CTRL_POS);
//       if (logRowFlags[LOG_CTRL_EDGE])
//         control.ctrlEdge->toLog(LOG_CTRL_EDGE);
//       if (logRowFlags[LOG_CTRL_WALL])
//         control.ctrlWallTurn->toLog(LOG_CTRL_WALL);
//       if (logRowFlags[LOG_CTRL_FWD_DIST])
//         control.ctrlWallVel->toLog(LOG_CTRL_FWD_DIST);
//       if (logRowFlags[LOG_CTRL_BAL])
//         control.ctrlBal->toLog(LOG_CTRL_BAL);
//       if (logRowFlags[LOG_CTRL_BAL_VEL])
//         control.ctrlBalVel->toLog(LOG_CTRL_BAL_VEL);
//       if (logRowFlags[LOG_CTRL_BAL_POS])
//         control.ctrlBalPos->toLog(LOG_CTRL_BAL_POS);
      //       if (logRowFlags[LOG_BAL_CTRL])
//       {
//         float b[5] = {regBalE[0], regul_bal_uvel, regBalUI[0], regBalUD[0], balTiltRef};
//         float v[5] = {regBalVelE[0], regBalVelU[0], regBalVelUI[0], regBalVelUD[0], regul_balvel_reduc};
//         
//         if (false)
//           // log balance controller
//           addToLog(LOG_BAL_CTRL, b, sizeof(b)); 
//         else
//           // log balance velocity
//           addToLog(LOG_BAL_CTRL, v, sizeof(v)); 
//       }
      //     if (logRowFlags[LOG_EXTRA])
      //     {
      //       float val[4] = {regTurnE[0], regTurnU[0], regul_turn_vel_reduc[0], regul_turn_vel_reduc[1]};
      //       addToLog(LOG_EXTRA, val, sizeof(val));
      //     }
      if (logRowFlags[LOG_EXTRA])
      {
        //float val[4] = {regBalE[0], regBalU[0], regBalVelE[0], regBalVelU[0]};
        //float val[4] = {tiltu1, 0, accAng, gyroTilt};
        // extra as values in velocity controller - left motor
        // float val[4] = {regVelELeft[0], regVelUDLeft[0], regul_vel_tot_ref[0], regVelULeft[0]};
        //float val[1] = {mission_turn_ref}; //, regTurnM[2], regTurnUD[0], regTurnE[2], regTurnE[0], regTurnE[0], regTurnUI[0]};
        addToLog(LOG_EXTRA, dataloggerExtra, sizeof(dataloggerExtra));
      }
//       if (logRowFlags[LOG_CHIRP])
//       {
//         float val[4] = {control.chirpAmplitude, float(control.chirpFrq), float(control.chirpAngle), control.chirpValue};
//         addToLog(LOG_CHIRP, val, sizeof(val));
//       }
    }
    else
      stopLogging();
  }
}


void ULog::writeTime(int8_t * data, int row, char * p1, int maxLength)
{ // write time in seconds to string
  float v = *(float*)data;
  if (row < 0)
    snprintf(p1, maxLength, "%% %2d    time %.4f sec, from %s (%d)\r\n", col++, v, state.getRobotName(), state.deviceID);
  else
    snprintf(p1, maxLength, "%.4f ", v);
}

// void ULog::writeMission(int8_t * data, int row, char * p1, int maxLength)
// {
//   int16_t * v = (int16_t *)data;
//   int8_t * vb = &data[2];
//   uint32_t * vf = (uint32_t *)&data[4];
//   if (row < 0)
//   {
//     snprintf(p1, maxLength, "%% %2d %2d %2d %2d   (mission %d), state %d, entered (thread %d, line %d), events 0x%lx (bit-flags)\r\n",
//               col, col+1, col+2, col+3, control.mission, v[0], vb[0], vb[1], vf[0]);
//     col += 4;
//   }
//   else
//   {
//     snprintf(p1, maxLength, "%d %d %d 0x%lx ", v[0], vb[0], vb[1], vf[0]);
//   }
// }

void ULog::writeAcc(int8_t * data, int row, char * p1, int maxLength)
{
  //int16_t * v = (int16_t*)data;
  float * v = (float*)data;
  if (row < 0)
  {
//     snprintf(p1, maxLength, "%% %2d %2d %2d Acc x,y,z [m/s2]: %g %g %g\r\n", col, col+1, col+2, v[0]* imu.accScale[0], v[1]* imu.accScale[1], v[2]* imu.accScale[2]);
    snprintf(p1, maxLength, "%% %2d %2d %2d Acc x,y,z [m/s2]: %g %g %g\r\n", col, col+1, col+2, v[0], v[1], v[2]);
    col += 3;
  }
  else
    // snprintf(p1, maxLength, "%f %f %f ", v[0]* imu.accScale[0], v[1]* imu.accScale[1], v[2]* imu.accScale[2]); // old IMU
    snprintf(p1, maxLength, "%f %f %f ", v[0], v[1], v[2]);
}

void ULog::writeGyro(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d %2d Gyro x,y,z [deg/s]: %g %g %g\r\n", 
             col, col+1, col+2, v[0], v[1], v[2]);
    col += 3;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f %.4f ", v[0], v[1], v[2]);
}

// void ULog::writeMag(int8_t * data, int row, char * p1, int maxLength)
// {
//   int16_t * v = (int16_t*)data;
//   if (row < 0)
//   {
//     snprintf(p1, maxLength, "%% %2d %2d %2d Mag x,y,z [int]: %d %d %d\r\n", col, col+1, col+2, v[0], v[1], v[2]);
//     col += 3;
//   }
//   else
//     snprintf(p1, maxLength, "%d %d %d ", v[0], v[1], v[2]);
// }

void ULog::writeCurrent(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Motor current left, right, supply [A]: %.3f %.3f %.3f\r\n", col, col+2, v[0], v[1], v[2]);
    col += 3;
  }
  else
  {
    snprintf(p1, maxLength, "%.3f %.3f %.3f ", v[0], v[1], v[2]);
  }
}

void ULog::writeVel(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Wheel velocity [m/s] left, right: %.4f %.4f\r\n", col, col +1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f ", v[0], v[1]);
}

void ULog::writeTurnrate(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %d   Turnrate [r/s]: %.4f\r\n", col, v[0]);
    col += 1;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f ", v[0], v[1]);
}


void ULog::writeEnc(int8_t * data, int row, char * p1, int maxLength)
{
  int32_t * v = (int32_t *)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Encoder left, right: %ld %ld\r\n", col, col+1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(p1, maxLength, "%ld %ld ", v[0], v[1]);
}

void ULog::writeMotPWM(int8_t * data, int row, char * p1, int maxLength)
{
  int16_t * v = (int16_t*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Motor voltage [PWM] left, right: %d %d\r\n", col, col+1, v[0], v[1]);
    col +=2;
  }
  else
    snprintf(p1, maxLength, "%d %d ", v[0], v[1]);
}


void ULog::writeMotVRef(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Motor velocity ref left, right: %.4f %.4f\r\n", col, col+1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f ", v[0], v[1]);
}

/////////////////////////////////////////////////

void ULog::writeMotVolt(int8_t * data, int row, char * p1, int maxLength)
{
  int16_t * v = (int16_t*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d Motor voltage [V] left, right: %.2f %.2f\r\n", col, col+1, float(v[0])/1000.0, float(v[1])/1000.0);
    col += 2;
  }
  else
    snprintf(p1, maxLength, "%.2f %.2f ", float(v[0])/1000.0, float(v[1])/1000.0);
}

//////////////////////////////////////////////////

void ULog::writeBaro(int8_t * data, int row, char * p1, int maxLength)
{
  int16_t * v = (int16_t*)data;
  uint32_t * u = (uint32_t*)&data[2];
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d %2d Barometer temp, pressure and height T, P, H: %.1f %ld %.2f\r\n", col, col+1, col+2, *v / 10.0, *u, v[3]/100.0);
    col += 3;
  }
  else
    snprintf(p1, maxLength, "%.1f %ld %.2f ", float(*v) / 10.0, *u, v[3]/100.0);
}

/////////////////////////////////////////////////

void ULog::writePose(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d %2d %2d Pose x,y,h,tilt [m,m,rad,rad]: %g %g %g %g\r\n", col, col+1, col+2, col+3, v[0], v[1], v[2], v[3]);
    col += 4;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f %.6f %.6f ", v[0], v[1], v[2], v[3]);
}

////////////////////////////////////////////////

void ULog::writeBatt(int8_t * data, int row, char * p1, int maxLength)
{
  uint16_t * v = (uint16_t*)data;
  if (row < 0)
    snprintf(p1, maxLength, "%% %2d    Battery voltage [V]\r\n", col++); //, state.getBatteryVoltage(*v));
  else
    snprintf(p1, maxLength, "%.2f ", state.getBatteryVoltage(*v));
}

void ULog::writeTiming(int8_t * data, int row, char * p1, int maxLength)
{
  int32_t * v = (int32_t*)data;
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d %2d timing [CPU_clock(us), ad, sensor, control, end, T-sample, load o/oo]\r\n",
             col, col+6);
    // no data, as it gives an error ??? but why?
//     snprintf(p1, maxLength, "%% %2d %2d timing [time CPU_clock, ad, sensor, control, end, T-sample, load o/oo]: %lu %ld %ld %ld %ld %ld %ld\r\n",
//              col, col+6, uint32_t(v[0]), v[1], v[2], v[3], v[4], v[5], v[6]);
    col += 7;
  }
  else
  {
    int32_t v2[7];
    v2[1] = v[1];
    v2[2] = v[2];
    v2[3] = v[3];
    v2[4] = v[4];
    // sample time
    v2[6] = v[6];
    if (v2[1] > 3000)
      v2[1] = -1;
    if (v2[2] > 3000)
      v2[2] = -1;
    if (v2[3] > 3000)
      v2[3] = -1;
    if (v2[4] > 3000)
      v2[4] = -1;
    if (v2[6] > 3000)
      v2[6] = -1;
    snprintf(p1, maxLength, "%lu %ld %ld %ld %ld %ld %ld ", uint32_t(v[0]), v2[1], v2[2], v2[3], v2[4], v[5], v2[6]);
  }
}

////////////////////////////////////////////////

// void ULog::writeCtrlTime(int8_t * data, int row, char * p1, int maxLength)
// {
//   uint32_t * v = (uint32_t*)data;
//   if (row < 0)
//   {
//     snprintf(p1, maxLength, "%% %2d %2d read sensor time [ms]: %.3f and ctrl time %.3f, cycleEnd %.3f\r\n", col, col+2,
//              float(v[0]) * 1000.0 / F_CPU, float(v[1]) * 1000.0 / F_CPU, float(v[2]) * 1000.0 / F_CPU);
//     col += 3;
//   }
//   else
//     snprintf(p1, maxLength, "%.3f %.3f %.3f ",
//              float(v[0]) * 1000.0 / F_CPU, float(v[1]) * 1000.0 / F_CPU, float(v[2]) * 1000.0 / F_CPU);
// }

///////////////////////////////////////////////

/**
 * Control values logged
 * */
// void ULog::writeCtrlVal(int8_t * data, int row, char * p1, int maxLength, int item)
// {
//   float * v = (float*)data;
//   if (row < 0)
//   {
//     const char * name;
//     switch (item)
//     {
//       case LOG_CTRL_VELL: name = "ctrl left "; break;
//       case LOG_CTRL_VELR: name = "ctrl right"; break;
//       case LOG_CTRL_TURN: name = "ctrl head"; break;
//       case LOG_CTRL_POS: name = "ctrl pos"; break;
//       case LOG_CTRL_EDGE: name = "ctrl edge"; break;
//       case LOG_CTRL_WALL: name = "ctrl wall"; break;
//       case LOG_CTRL_FWD_DIST: name = "ctrl fwd dist"; break;
//       case LOG_CTRL_BAL: name = "ctrl balance"; break;
//       case LOG_CTRL_BAL_VEL: name = "ctrl bal vel"; break;
//       case LOG_CTRL_BAL_POS: name = "ctrl bal pos"; break;
//       default: name = "error"; break;
//     }
//     snprintf(p1, maxLength, "%% %2d %2d %s, ref=%g, m=%g, m2=%g, uf=%g, r2=%g, ep=%g,up=%g, ui=%g, u1=%g, u=%g\r\n",
//              col, col+CTRL_LOG_SIZE-1, name, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
//     col += CTRL_LOG_SIZE;
//   }
//   else
//     snprintf(p1, maxLength, "%g %g %g %g %g %g %g %g %g %g ", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
// }

/////////////////////////////////////////////

void ULog::writeLinesensorExtra(int8_t * data, int row, char * p1, int maxLength)
{
  float * v = (float*)data;
  const int n = 19;
  // float positions
  // 0..6 is lensor gradient
  // 7,8 max,min gradient
  // 9,10,11 left,cent,right - left side
  // 12 edge position  - left side
  // 13,14,15 left,cent,right - rignt side
  // 16 edge position  - right side
  // 17, 18 max index, min idx (of int values)
  if (row < 0)
  {
    snprintf(p1, maxLength, 
             "%% %2d %2d linesensorExtra %.3f %.3f %.3f %.3f %.3f %.3f %.3f,\r\n"
             "%%    %2d %2d max=%.3f, min=%.3f, imax=%g, imin=%g, \r\n"
             "%%    %2d %2d lleft=%.4f, lcent=%.4f, lright=%.4f, ledgepos=%f,\r\n"
             "%%    %2d %2d rleft=%.4f, rcent=%.4f, rright=%.4f, redgepos=%f.\r\n",
             col, col+n-1, v[0], v[1], v[2], v[3], v[4], v[5], v[6],
             col+7, col + 10,
             v[7], v[8], v[17], v[18],
             col + 11, col + 14,
             v[9], v[10], v[11], v[12],
             col + 15, col+18,
             v[13], v[14], v[15], v[16]
        );
    col += n;
  }
  else
  {
    snprintf(p1, maxLength, "%.3f %.3f %.4f %.4f %4f %4f %4f "
        "%.3f %.3f %g %g "
        "%.4f %.4f %.4f %f "
        "%.4f %.4f %.4f %f ",
        v[0], v[1], v[2], v[3], v[4], v[5], v[6],
        v[7], v[8], v[17], v[18],
        v[9], v[10], v[11], v[12],
        v[13], v[14], v[15], v[16]
        );
  }
}

/**
 * convert binary log data item to (part of) text line loadable by MATLAB 
 * \param data is pointer to start of binary data
 * \param row is recording row number (-1 indicates make an explanation line preceded with a "%")
 * \param p1 is buffer to write data to
 * \param maxLength is remaining space in (p1) buffer */
void ULog::writeExtra(int8_t * data, int row, char * p1, int maxLength)
{
  if (false)
    writeLinesensorExtra(data, row, p1, maxLength);
  else
  {
    int m,n;
    float * v = (float *) data;
    if (row < 0)
    {
      snprintf(p1, maxLength, "%% %2d %2d linesensorExtra ", col, col+dataloggerExtraSize-1);
      col += dataloggerExtraSize;
      n = strlen(p1);
      m = n;
      for (int i = 0; i < dataloggerExtraSize; i++)
      {
        p1 += n;
        snprintf(p1, maxLength - m, "%g ", v[i]);
        n = strlen(p1);
        m += n;
      }
      p1 += n;
      snprintf(p1, maxLength - m, "\r\n");
    }
    else
    {
      m = 0;
      n = 0;
      for (int i = 0; i < dataloggerExtraSize; i++)
      {
        snprintf(p1, maxLength - m, "%g ", v[i]);
        n = strlen(p1);
        m += n;
        p1 += n;
      }
    }
  }
  return;
}

/**
 * convert binary log data item to (part of) text line loadable by MATLAB 
 * \param data is pointer to start of binary data
 * \param row is recording row number (-1 indicates make an explanation line preceded with a "%")
 * \param p1 is buffer to write data to
 * \param maxLength is remaining space in (p1) buffer */
// void ULog::writeChirp(int8_t * data, int row, char * p1, int maxLength)
// {
//   float * v = (float *) data;
//   if (row < 0)
//   {
//     snprintf(p1, maxLength, "%% %2d %2d Chirp amplitude=%g, frequency =%g rad/s, phase=%g rad, value=%g\r\n", col, col+3, v[0], v[1], v[2], v[3]);
//     col += 4;
//   }
//   else
//   {
//     snprintf(p1, maxLength, "%g %g %g %g ", v[0], v[1], v[2], v[3]);
//   }
//   return;
// }

/////////////////////////////////////

void ULog::writeLineSensor(int8_t * data, int row, char * p1, int maxLength)
{
  /*
   *        int16_t ldv[20];
   *        float * fdv = (float*) &ldv[8];
   *        int8_t * lsc = (int8_t*) &ldv[19];
   *        uint8_t * flags = (uint8_t*) &ldv[18];
   ldv[0] = adcLSH[0] - adcLSL[0];
   ldv[1] = adcLSH[1] - adcLSL[1];
   ldv[2] = adcLSH[2] - adcLSL[2];
   ldv[3] = adcLSH[3] - adcLSL[3];
   ldv[4] = adcLSH[4] - adcLSL[4];
   ldv[5] = adcLSH[5] - adcLSL[5];
   ldv[6] = adcLSH[6] - adcLSL[6];
   ldv[7] = adcLSH[7] - adcLSL[7];
   fdv[0] = lsLeftSide;  // word  8 and  9 (32 bit float)
   fdv[1] = lsRightSide; // word 10 and 11 (32 bit float)
   fdv[2] = findCrossingLineVal; // word 12 og 13
   fdv[3] = whiteVal; // is actually also black value, if looking for black, word 14 and 15
   fdv[4] = edgeAngle
   *flags = 0;
   if (lineSensorOn)
     *flags = 1;
   if (lsIsWhite)
     *flags |= 2;
   if (lsEdgeValid)
     *flags |= 4;
   if (lsEdgeValid)
     *flags |= 8;
   if (lsPowerHigh)
     *flags |= 0x10;
   if (lsPowerAuto)
     *flags |= 0x20;
   lsc[0] = crossingLineCnt; // word 19
   lsc[1] = lsEdgeValidCnt;
   lsc[2] = lsIdxLow;
   lsc[3] = lsIdxHi;
   
   addToLog(LOG_LINE, &ldv, sizeof(ldv));
   */
  int16_t * v = (int16_t*)data;
  float * vf = (float*)&v[8];
  int8_t * lsc = (int8_t*) &v[19];
  uint8_t flags = v[18];
  if (row < 0)
  {
    snprintf(p1, maxLength, "%% %2d .. %2d Edge sensor: left pos, right pos, lv, values d d d d d d d d, "
                     " white, used, LEDhigh, xingValue, xlcnt, lvcnt, edgeAngle\r\n",
             col, col+18
//     snprintf(p1, maxLength, "%% %2d .. %2d Edge sensor: left %f %d, right %f %d, values %d %d %d %d %d %d %d %d, "
//                      " white %d, used %d, LEDhigh=%d, xingVal=%.2f xlcnt=%d lvcnt=%d,LineVal=%.2f, lineLow=%d, lineHi=%d, edgeAngle=%g,"
//                      " (info whitelevel=[%d %d %d %d %d %d %d %d],"
//                      " blacklevel=[%d %d %d %d %d %d %d %d]),\r\n",
//              col, col+21,
//              vf[0], (flags & 0x04) == 0x04, vf[1], (flags & 0x08) == 0x08,
//              v[0],v[1], v[2], v[3], v[4], v[5], v[6], v[7],
//              (flags & 0x02) == 0x02, (flags & 0x01) == 0x01, (flags & 0x10) == 0x10,
//              vf[2],
//              lsc[0], lsc[1], vf[3], lsc[2],lsc[3], vf[4],
//              ls.whiteLevel[0], ls.whiteLevel[1], ls.whiteLevel[2], ls.whiteLevel[3],
//              ls.whiteLevel[4], ls.whiteLevel[5], ls.whiteLevel[6], ls.whiteLevel[7],
//              ls.blackLevel[0], ls.blackLevel[1], ls.blackLevel[2], ls.blackLevel[3],
//              ls.blackLevel[4], ls.blackLevel[5], ls.blackLevel[6], ls.blackLevel[7]
    );
    col += 19;
  }
  else
    snprintf(p1, maxLength, "%.4f %.4f %d  %d %d %d %d %d %d %d %d  %d %d %d %.2f %d %d %g ",
             vf[0], vf[1], (flags & 0x08) == 0x08,
             v[0],v[1], v[2], v[3], v[4], v[5], v[6], v[7],
             (flags & 0x02) == 0x02, (flags & 0x01) == 0x01, (flags & 0x10) == 0x10,
             vf[2],
             lsc[0], lsc[1], vf[4]
    );
}

/////////////////////////////////////////////////

void ULog::writeDistSensor(int8_t * data, int row, char * p1, int maxLength)
{
  uint32_t * v = (uint32_t*)data;
  float d1 = 1.0/(v[0] * irdist.irA[0] - irdist.irB[0]);
  float d2 = 1.0/(v[1] * irdist.irA[1] - irdist.irB[1]);

  if (d1 > 1.5 or d1 < 0.05)
    d1 = 1.5;
  if (d2 > 1.5 or d2 < 0.05)
    d2 = 1.5;

  //   if (IR13_50CM)
//   {
//   }
//   else
//   { // old calculation
//     if (v[0] > 0)
//       d1 = irA[0] + irB[0]/v[0];
//     if (v[1] > 1)
//       d2 = irA[1] + irB[1]/v[1];
//   }  
  if (row < 0)
  {
//     if (IR13_50CM)
//       snprintf(p1, maxLength, "%% %2d %2d Distance sensor [AD]: %lu %lu\r\n", col, col+1, v[0], v[1]);
//     else
      snprintf(p1, maxLength, "%% %2d %2d Distance sensor [m]: %.3f %.3f\r\n", col, col+1, d1, d2);
    col += 2;
  }
  else
  {
//     if (IR13_50CM)
//       snprintf(p1, maxLength, "%lu %lu ", v[0], v[1]);
//     else
      snprintf(p1, maxLength, "%.3f %.3f ", d1, d2);
  }
}


void ULog::writeBufferMsg()
{
  if (logStreamedMsgOutPos == 0)
  {
    const char * p1 = "%%%% Logged communication messages intended for USB\n";
    int n = strlen(p1);
    usb.send_block(p1, n);
    p1 = "%%%% 1   Teensy time stamp in seconds since boot\n";
    n = strlen(p1);
    usb.send_block(p1, n);
    p1 = "%%%% 2   The message text line\n";
    n = strlen(p1);
    usb.send_block(p1, n);
  }
  if (logStreamedMsgOutPos < logStreamedMsgPos)
  {
    const char * p1 = (char*)logBuffer + logStreamedMsgOutPos;
    int n = strlen(p1);
    usb.send_block(p1, n);
    // advance to first character after the message and its terminating null
    logStreamedMsgOutPos += n + 1;
  }
}


///////////////////////////////////////////////

void ULog::initLogStructure(int timeFactor)
{ // stop logging of messages
  logStreamedMsg = false;
  // start at first position
  logRowSize = 0;
//   logTimeFactor = timeFactor;
  logRowFlags[LOG_TIME] = true;
  // set log entry positions
  for (int i = 0; i <  LOG_MAX_CNT; i++)
  {
    if (logRowFlags[i])
    {
      int bz;
      logRowPos[i] = logRowSize;
      switch (logRowItemSize[i * 2 + 1])
      {
        case LOG_FLOAT  : bz = sizeof(float); break;
        case LOG_DOUBLE : bz = sizeof(double); break;
        case LOG_INT8   : bz = sizeof(int8_t); break;
        case LOG_UINT8  : bz = sizeof(uint8_t); break;
        case LOG_INT16  : bz = sizeof(int16_t); break;
        case LOG_UINT16 : bz = sizeof(uint16_t); break;
        case LOG_INT32  : bz = sizeof(int32_t); break;
        case LOG_UINT32 : bz = sizeof(uint32_t); break;
        default: bz = 1; break;
      }
      logRowSize += bz * logRowItemSize[i * 2];
      if (false)
      { // debug
        const int MSL = 50;
        char s[MSL];
        snprintf(s, MSL, "#initLogStructure log %d size=%d\r\n", i, logRowSize);
        usb.send(s);
      }
    }
  }
//   const int MSL = 50;
//   char s[MSL];
//   snprintf(s, MSL,"initLogStructure size=%d bytes/line\n", logRowSize);
//   usb.send(s);
  if (logBuffer == nullptr)
    logRowsCntMax = 0;
  else
    logRowsCntMax = LOG_BUFFER_MAX / logRowSize;
  // clear buffer
  logRowCnt = 0;
  logFull = false;
}

////////////////////////////////////////////

int posDbg = 0;

void ULog::addToLog(logItem item, void * data, int dataCnt)
{
  int8_t * pd = logBuffer + (logRowCnt - 1) * logRowSize + logRowPos[item];
//   if (false)
//   {
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "#add_to_log 0 <= (%d + %d) < %d, line %d/%d, from %x cnt %d\n", 
//              (logRowCnt - 1) * logRowSize, 
//              logRowPos[item],
//              LOG_BUFFER_MAX, 
//              logRowCnt, logRowsCntMax,
//              (int)data, dataCnt
//             );
//     usb.send(s);
//   }
  memcpy(pd, data, dataCnt);
}

void ULog::addMsgLog(const char * data)
{
  if (logStreamedMsgFull or not logStreamedMsg)
    return;
  else
  {
    int n = strlen(data);
    const int MSL = 32;
    // debug
//     const int MSSL = 200;
//     char ss[MSSL];
//     snprintf(ss, MSL, "# logging %d data from pos %lu/%d\n", n, logStreamedMsgPos, LOG_BUFFER_MAX);
//     int m = strlen(ss);
//     usb.send_block(ss, m);
    // debug end
    if (n + MSL + logStreamedMsgPos < LOG_BUFFER_MAX)
    { // get position in the buffer
      char * pd = (char*)logBuffer + logStreamedMsgPos;
      // add timestamp
      // a tiny chance that tsec and tusec are out of sync - ignored
      snprintf(pd, MSL, "%lu.%03lu ", tsec, tusec/1000);
      pd[32] = '\0'; // just in case
      int m = strlen(pd);
      pd += m;
      // add the message - and also copy the end null byte
      strncpy(pd, data, n+1);
      // advance buffer pointer to the first char after the null byte
      logStreamedMsgPos += n + m + 1;
    }
    else
    { // no more space
      logStreamedMsgFull = true;
      usb.send("# log message buffer full\n");
    }
  }
}



//////////////////////////////////////////////////////

bool ULog::addNewRowToLog()
{ // logRowCnt is next log entry - uses (logRowCnt-1)
  if (logRowCnt >= logRowsCntMax)
    return false;
  int8_t * pd = logBuffer + logRowCnt * logRowSize;
  if (uint32_t(0x20000000) > (uint32_t)pd and (uint32_t(0x20000000) < ((uint32_t)pd + (uint32_t)logRowSize))) 
  { // skip the row that spans address 0x20000000
//     const int MSL = 70;
//     char s[MSL];
//     snprintf(s, MSL,"#mem-hole skipped row %d from %x to %x\n", logRowCnt, (unsigned int)pd, (unsigned int)pd + logRowSize);
//     usb.send(s);
    logRowCnt++;
    if (logRowCnt >= logRowsCntMax)
      return false;    
  }
  logRowCnt++;
//   if (logRowCnt % 10 == 0)
//   {
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# ULog::addNewRow Row %d/%d of %d bytes\n", logRowCnt, logRowsCntMax, logRowCnt * logRowSize);
//     usb.send(s);
//   }
  return true;
}

//////////////////////////////////////////////////

void ULog::setLogSize(logItem item, int count, char type)
{
  logRowItemSize[item * 2] = count;
  logRowItemSize[item * 2 + 1] = type;
}

//////////////////////////////////////////////////

int tried = 0;

int ULog::logWriteBufferTo(int row)
{
//  int row;
  logItem item;
  int8_t * bp;
  const int MLL = 2000;
  char logline[MLL + 3];
  char * p1 = logline;
  int n = 0; // used number of characters
  // write all recorded rows
  // row -1 is flag to get matlab text
  if (row <= 0)
  { // first text row or row 0 repeated
    bp = logBuffer;
    // used by text printout in all write functions
    col = 1;
  }
  else
    // find position for this row
    bp = logBuffer + row * logRowSize;
  // test for skip at teensy 3 memory block change
  if (uint32_t(0x20000000) > (uint32_t)bp and (uint32_t(0x20000000) < ((uint32_t)bp + (uint32_t)logRowSize))) 
  { // there is problems at address 0x20000000 on Teensy 3.2 and must be skipped
    // but we do it on all versions anyhow
    row++;
    if (row >= logRowCnt)
      // it was last usable row
      return row;
    bp += logRowSize;
  }
  // send all recorded data types
  for (item = LOG_TIME; item < LOG_MAX_CNT; item = logItem(int(item) + 1))
  { // go thorough all possible log items
    if (logRowFlags[item])
    { // this log item is recorded
      switch (item)
      { // reformat binary data as text to string from p1
        case LOG_TIME:  writeTime(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
//         case LOG_MISSION: writeMission(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_ACC:   writeAcc(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_GYRO:  writeGyro(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_MOTV_REF:  writeMotVRef(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_MOTV:  writeMotVolt(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_MOTA:  writeCurrent(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_ENC:   writeEnc(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_WHEELVEL:   writeVel(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_TURNRATE:   writeTurnrate(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_POSE:  writePose(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        case LOG_BATT:
          writeBatt(&bp[logRowPos[item]], row, p1, MLL - n);
          n += strlen(p1);
          p1 = &logline[n];
          break;
        case LOG_TIMING:
          if (true)
          {
            writeTiming(&bp[logRowPos[item]], row, p1, MLL - n);
            n += strlen(p1);
            p1 = &logline[n];
          }
          break;
        case LOG_LINE:
          if (true)
          {
            writeLineSensor(&bp[logRowPos[item]], row, p1, MLL - n);
            n += strlen(p1);
            p1 = &logline[n];
          }
        break;
        case LOG_DIST:  writeDistSensor(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
//         case LOG_CTRLTIME: writeCtrlTime(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
//         // all control data types are equal
//         case LOG_CTRL_VELL:
//         case LOG_CTRL_VELR:
//         case LOG_CTRL_TURN:
//         case LOG_CTRL_POS:
//         case LOG_CTRL_EDGE:
//         case LOG_CTRL_WALL:
//         case LOG_CTRL_FWD_DIST:
//         case LOG_CTRL_BAL:
//         case LOG_CTRL_BAL_VEL:
//         case LOG_CTRL_BAL_POS:
//           writeCtrlVal(&bp[logRowPos[item]], row, p1, MLL - n, item);
//           n += strlen(p1);
//           p1 = &logline[n];
//           break;
         // this extra is hard coded and need change if used
        case LOG_EXTRA: writeExtra(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
//         case LOG_CHIRP: writeChirp(&bp[logRowPos[item]], row, p1, MLL - n); n += strlen(p1); p1 = &logline[n]; break;
        default: break;
      }
      if (row == -1)
      { // first log line is send as MATLAB individual comment lines
        if (logToUSB)
        { // send one headline at a time
          logline[n] = '\0';
          rowSendOK = usb.send(logline);
        }
        // reset result string for new headline.
        n = 0;
        p1 = logline;
      }
    }
  }
  //add a new line after a full data-line
  if (row >= 0)
  { // all other lines are send with all measurements in one line.
    // add carriage return and new line.
    *p1++ = '\r';
    *p1++ = '\n';
    *p1++ = '\0';
    rowSendOK = usb.send(logline); //, rowSendOK, true);
    if (not rowSendOK)
    { // resend last row to wifi only
      row--;
      tried++;
      usb.send("# UUsb::re-try\r\n");
    }
    else
    {
      tried = 0;
    }
  }
  return row;
}

/**
 * Mission init is called before any control is attempted,
 * this can be used to initialize ant variables dependent on measured values, i.e.
 * battery voltage or gyro.
 * and to set data logger options */
void ULog::setLogFlagDefault()
{
  // log data size
  // data log size and count must be initialized here
  // there must further be a write function and other parts in data_logger.cpp 
  // to add new logged data items
  setLogSize(LOG_TIME, 1, LOG_FLOAT);
  setLogSize(LOG_MISSION, 4, LOG_INT16);
  setLogSize(LOG_ACC,  3, LOG_FLOAT);
  setLogSize(LOG_GYRO, 3, LOG_FLOAT);
  setLogSize(LOG_MAG,  3, LOG_FLOAT);
  setLogSize(LOG_MOTV_REF, 2, LOG_FLOAT);
  setLogSize(LOG_MOTV, 2, LOG_INT16);
  setLogSize(LOG_MOTA, 3, LOG_FLOAT);
  setLogSize(LOG_ENC,  2, LOG_UINT32);
  setLogSize(LOG_POSE, 4, LOG_FLOAT);
  setLogSize(LOG_WHEELVEL,  2, LOG_FLOAT);
  setLogSize(LOG_TURNRATE,  2, LOG_FLOAT);
  setLogSize(LOG_LINE, 22, LOG_UINT16);
  setLogSize(LOG_DIST, 2, LOG_UINT32);
  setLogSize(LOG_BATT, 1, LOG_UINT16);
  setLogSize(LOG_TIMING, 7, LOG_UINT32);
  setLogSize(LOG_CTRLTIME, 3, LOG_INT32);
  //setLogSize(LOG_BARO, 4, LOG_INT16);
//   setLogSize(LOG_BAL_CTRL, 5, LOG_FLOAT);
  setLogSize(LOG_EXTRA, dataloggerExtraSize, LOG_FLOAT);
  setLogSize(LOG_CHIRP, 4, LOG_FLOAT);
  //   const int CTRL_LOG_SIZE = 10;
  setLogSize(LOG_CTRL_VELL, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_VELR, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_TURN, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_POS, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_EDGE, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_WALL, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_FWD_DIST, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_BAL, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_BAL_VEL, CTRL_LOG_SIZE, LOG_FLOAT);
  setLogSize(LOG_CTRL_BAL_POS, CTRL_LOG_SIZE, LOG_FLOAT);
  //
  logRowFlags[LOG_TIME] = 1; // not tested - time always on
  //
  // log flags (default)
//   control.missionState = 0;
  logRowFlags[LOG_TIME] = true; // state number in mission
  logRowFlags[LOG_MISSION] = true; // state number in mission
  logRowFlags[LOG_ACC] = false;    // in ?
  logRowFlags[LOG_GYRO] = false;    // in ?
  //logRowFlags[LOG_MAG] = false;    // not used
  logRowFlags[LOG_MOTV] = true;    // orderd anchor voltage (before PWM)
  logRowFlags[LOG_MOTV_REF] = true;    // Orderd motor velocity
  logRowFlags[LOG_MOTA] = false;   // measured anchor current in Amps
  logRowFlags[LOG_WHEELVEL] = true;  // wheel velocity in rad/s
  logRowFlags[LOG_ENC] = false;    // raw encoder counter
  logRowFlags[LOG_POSE] = true;    // calculated pose x,y,th
  logRowFlags[LOG_LINE] = false;    // line sensor
  logRowFlags[LOG_DIST] = false;    // distance sensor
  logRowFlags[LOG_BATT] = true;    // battery oltage in Volts
  logRowFlags[LOG_TIMING] = false;    // battery oltage in Volts
  logRowFlags[LOG_CTRL_VELL] = false;  // All relevant values in controller
  logRowFlags[LOG_CTRL_VELR] = false;   
  logRowFlags[LOG_CTRL_TURN] = false;   
  logRowFlags[LOG_CTRL_POS] = false;    
  logRowFlags[LOG_CTRL_EDGE] = false;    
  logRowFlags[LOG_CTRL_WALL] = false;    
  logRowFlags[LOG_CTRL_FWD_DIST] = false;    
  logRowFlags[LOG_CTRL_BAL] = false;    // 
  logRowFlags[LOG_CTRL_BAL_VEL] = false;  // 
  logRowFlags[LOG_CTRL_BAL_POS] = false;  // control of in balance position
  logRowFlags[LOG_CHIRP] = false;    // chirp log not default

}


void ULog::eePromSaveStatusLog()
{
  uint32_t flags = 0;
  for (int i = 0; i < LOG_MAX_CNT; i++)
  { // LOG_MAX_CNT is about 16 and less than 32
    if (logRowFlags[i])
      flags += 1 << i;
  }
  eeConfig.push32(flags);
  eeConfig.push32(logInterval);
}

void ULog::eePromLoadStatusLog()
{
  uint32_t flags;
  int skipCount = 4 + 4;
  if (state.robotIDvalid() and not eeConfig.isStringConfig())
  {
    flags = eeConfig.read32();
    for (int i = 0; i < LOG_MAX_CNT; i++)
    {
      logRowFlags[i] = (flags & (1 << i)) != 0;
    }
    logInterval = eeConfig.read32();
    current.logIntervalChanged();
  }
  else
  { // just skip, leaving default settings
    eeConfig.skipAddr(skipCount);
  }
}




