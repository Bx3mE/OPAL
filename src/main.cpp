/*
  OPAL.ino - Main projectfile to run OPAL FW on PJRC Teensy 4.x board

  Part of OpenGalvo - OPAL Firmware

  Copyright (c) 2020-2021 Daniel Olsson

  OPAL Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  OPAL Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with OPAL Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <Arduino.h>
#include <HardwareSerial.h>
#include "Pins.h"
#include "configuration.h"
#include "Helpers.h"
#include "Mcodes.h"
#include "main.h"

#ifdef LASER_IS_SYNRAD

#include <Synrad48Ctrl.h>
#include <XY2_100.h>

Synrad48Ctrl laser;

#endif



XY2_100 galvo;

#include <CircularBuffer.h>
CircularBuffer<GCode, CMDBUFFERSIZE> commandBuffer;
CircularBuffer<GCode, MBUFFERSIZE> mBuffer;

#include "SerialCMDReader.h"
SerialCMDReader serialReciever;

uint64_t next_command_time = 0;

bool beginNext = true;
static uint64_t startNanos;
static uint64_t endNanos;

static uint64_t _now;

static coordinate previousTarget;
coordinate currentTarget;

static double distx;
static double disty;
static double distz;

static double feedrate = DEFAULT_FEEDRATE;

GCode * oldPreviousMove;
GCode * previousMove;
GCode * currentMove;
GCode * nextMove;

coordinate target;
int lastLaserPWR = 0;
bool laserChanged = false;
int itcnt = 0;
static int interpolCnt = 0;
static bool movesaAreIncremental = false;

void setup() {
  previousTarget.x = 0;
  previousTarget.y = 0;
  previousTarget.z = 0;

  currentTarget.x = 0;
  currentTarget.y = 0;
  currentTarget.z = 0;
  
  Serial.begin(115200);
  Serial.println("Init");
  galvo.begin();
  Serial.println("Galvo init done");
  serialReciever.begin(&commandBuffer);
  Serial.println("Serial Reciever ready");
    pinMode(13, OUTPUT);
    
    digitalWrite(13,0); //Set LED OFF
}

void calculateMoveLengthNanos(double xdist, double ydist, double moveVelocity, double* result)  {  //Velocity is presumed to be in coordinates/s

  double lengthOfMove = sqrt( (0.0 + xdist)*(0.0 + xdist)  + (0.0 + ydist)*(0.0 + ydist) ); // calc hypo a^2+b^2=c^2
  //TODO: Verify unit conversion
  *result = ((lengthOfMove*1000*1000*1000/moveVelocity)); //s=v*t -> s/v =t   (movelength/moveVolocity) -> 2pos / (4pos/s) = 0.5s *1000 = 500ms
  return;
}

void processMcode(GCode* code)
{
  Serial.println("M-Code");
  switch (code->code)
  {
  case 3: //M3
    /*    Code is Spindle CV Normally used for spindle on direction CV and 'Sxxxxx' to set RPM
        When using Laser laser has no rotaton so handles same as M4  */
        //Fallthrough intended
  case 4: //M4
  {
    Serial.println("M3/M4");
    int lpower = code->s;
    if( lpower != MAX_VAL)
    {
      lastLaserPWR = lpower;
    }
    #ifdef LASER_IS_SYNRAD
    laser.setLaserPWM(lastLaserPWR); //Unknown code might cause unexpected behaviour Better turn off laser - SAFETY
    Serial.print("Laser SET!");Serial.println(lastLaserPWR);
    #else
    digitalWrite(LASER_PWM_OUT_PIN, lastLaserPWR);
    #endif    
    break;
  }
  case 5: //M5
    Serial.println("M5");
    lastLaserPWR = 0;        
    #ifdef LASER_IS_SYNRAD
      laser.setLaserPWM(0); //Unknown code might cause unexpected behaviour Better turn off laser - SAFETY
    #else
      digitalWrite(LASER_PWM_OUT_PIN, 0);
    #endif
    break;

  case 17: //M17 - Turn all steppers ON -> Galvo & Stepper PSU Control (SSRs)
    Serial.println("M17");
    digitalWrite(GALVO_SSR_OUT_PIN, 1);
    digitalWrite(STEPPER_SSR_OUT_PIN, 1); 
    break;
  case 18: //M18 - Turn all steppers OFF -> Galvo & Stepper PSU Control (SSRs)
    Serial.println("M18");
    digitalWrite(GALVO_SSR_OUT_PIN, 0);
    digitalWrite(STEPPER_SSR_OUT_PIN, 0); 
    break;

case 80: //M80 - Laser PSU Control (SSR)
    Serial.println("M80");
    // Implicit delay for SynradCtrl::laserInitTime milliseconds (5000ms)
    #ifdef LASER_IS_SYNRAD
    laser.begin(LASER_PWM_OUT_PIN, LASER_SSR_OUT_PIN);
    #else
    digitalWrite(LASER_SSR_OUT_PIN,1);
    #endif

    break;
  case 81: //M81 - Laser PSU Control (SSR)
    Serial.println("M81");
    #ifdef LASER_IS_SYNRAD
    laser.stop();
    #else
    digitalWrite(LASER_SSR_OUT_PIN,0);
    #endif
    break;
  
  default:
    break;
  }
  //Serial.print("\nExecuting MCode: ");Serial.print(code->codeprefix);Serial.println(code->code);
  //Serial.print("-- Executing MCode: Not Implemented\n");
}

void process()  {

  //TODO: add serial forwarder to marlin slave board for driving pistons on Z axis and powder distribution on Secondary serial 

  _now = nanos();
  
  if(beginNext)  {
    //Serial.println("Begin next");
    while(!mBuffer.isEmpty())    {
      //processMCodes
      Serial.println("processing MCode");
      GCode* cmd = new GCode(mBuffer.pop());
      processMcode(cmd);
      delete cmd;
      Serial.println("processing MCode Completed");
    }
    if(currentMove->code == 90)
    {
      delete currentMove;
      currentMove = nextMove;
    }
    else
    {
      delete oldPreviousMove; 
      oldPreviousMove = previousMove;
      previousMove = currentMove;
      currentMove = nextMove;
    }

    bool gcodeFound = false;
    while(!gcodeFound && !commandBuffer.isEmpty())
    {
      nextMove = new GCode((commandBuffer.pop()));
      if((*nextMove).codeprefix != 'G'){
        mBuffer.unshift(*nextMove);
      }
      else{
        gcodeFound = true;  
      }       
    }      
    if(!gcodeFound){
      nextMove = NULL;
    }

    // Buffer MGMT is done 

    //Allways update history...
    if(previousMove)  //SET UP PREVIOUS POSITION
    {
      if((*previousMove).absolute)
      {
        if((*previousMove).x != MAX_VAL)
          previousTarget.x = (*previousMove).x;
        if((*previousMove).y != MAX_VAL)
          previousTarget.y = (*previousMove).y;
        if((*previousMove).z != MAX_VAL)
          previousTarget.z = (*previousMove).z;
      }
      else
      { //incremental
        if((*previousMove).x != MAX_VAL)
          previousTarget.x += (*previousMove).x;
        if((*previousMove).y != MAX_VAL)
          previousTarget.y += (*previousMove).y;
        if((*previousMove).z != MAX_VAL)
          previousTarget.z += (*previousMove).z;
      }
    }
    
    if(currentMove)  { 

      if((*currentMove).code == 90)
      {   //G90 means no calculation and no interpolation as feedrate is ignored
        Serial.println("G90");
        movesaAreIncremental = false;
        beginNext = true;
        laser.handleLaser();
        return;
      }
      else if((*currentMove).code == 91)
      {   //G9 means no movement
        Serial.println("G91");
        movesaAreIncremental = true;
        beginNext = true;
        laser.handleLaser();
        return;
      }

      if(movesaAreIncremental)
      {
        (*currentMove).absolute = false;
        if((*currentMove).x != MAX_VAL)
          currentTarget.x = previousTarget.x + (*currentMove).x;
        if((*currentMove).y != MAX_VAL)
          currentTarget.y = previousTarget.y + (*currentMove).y;
        if((*currentMove).z != MAX_VAL)
          currentTarget.z = previousTarget.z + (*currentMove).z;
      }
      else
      {
        //(*currentMove).absolute = true; //default initialized value
        if((*currentMove).x != MAX_VAL)
          currentTarget.x = (*currentMove).x;
        if((*currentMove).y != MAX_VAL)
          currentTarget.y = (*currentMove).y;
        if((*currentMove).z != MAX_VAL)
          currentTarget.z = (*currentMove).z;
      }
      if((*currentMove).f != MAX_VAL)
        feedrate = (*currentMove).f;
      // Set Laser Power
      if((*currentMove).s != MAX_VAL)
      {
        lastLaserPWR = (*currentMove).s;
        laserChanged = true;
      }

      if((*currentMove).code == 1) 
      {   // G1
        #ifdef LASER_G0_OFF_G1_ON
          #ifdef LASER_IS_SYNRAD
          laser.setLaserPWM(lastLaserPWR); //Unknown code might cause unexpected behaviour Better turn off laser - SAFETY
          #endif
        #endif
        startNanos = _now;
        if((*currentMove).absolute)
        {
          distx = currentTarget.x-previousTarget.x;
          disty = currentTarget.y-previousTarget.y;
          distz = currentTarget.z-previousTarget.z; 
        }  
        else
        {
          distx = (*currentMove).x;
          disty = (*currentMove).y;
          distz = (*currentMove).z;
        } 
        calculateMoveLengthNanos(distx, disty, feedrate, &((*currentMove).moveLengthNanos));
        if(previousMove->code == 90)
        {
          Serial.print("\nMoveLengthnanos = ");
          Serial.print((*currentMove).moveLengthNanos);
          Serial.print(" distX = ");
          Serial.print(distx);
          Serial.print(" disty = ");
          Serial.print(disty);
          Serial.print(" feedrate = ");
          Serial.print(feedrate);
        }
        endNanos = startNanos + (*currentMove).moveLengthNanos;
      }
      else if((*currentMove).code == 0)
      {   //G0 means no calculation and no interpolation as feedrate is ignored
        #ifdef LASER_G0_OFF_G1_ON
            #ifdef LASER_IS_SYNRAD
             laser.setLaserPWM(0); //Unknown code might cause unexpected behaviour Better turn off laser - SAFETY
            #endif
        #endif
      }
      else
      {   //
        Serial.println("Unknown GCode! code: ");
        Serial.println((*currentMove).codeprefix);
        Serial.println((*currentMove).code);
        #ifdef LASER_IS_SYNRAD
          laser.setLaserPWM(0); //Unknown code might cause unexpected behaviour Better turn off laser - SAFETY
        #endif
      }
      beginNext = false;
    }//end if(currentmove)
  }//end if(beginnext)
  
  //interpolate  
  if(currentMove && (_now > endNanos || (*currentMove).code == 0))  //We are out of time or G0
  {
    Serial.println("Out of time - go to final pos x: ");Serial.println(currentTarget.x);Serial.println(" y:");Serial.println(currentTarget.y);
    #ifdef LASER_IS_SYNRAD
      laser.handleLaser();    
    #endif

    #ifdef INVERSE_X
    double mapx = map(currentTarget.x, 0.0,X_MAX, 65535,0)+0.5;
    #else
    double mapy = map(currentTarget.x, 0.0,X_MAX, 0,65535)+0.5;
    #endif
    
    #ifdef INVERSE_Y
    double mapx = map(currentTarget.y, 0.0,Y_MAX, 65535,0)+0.5;
    #else
    double mapy = map(currentTarget.y, 0.0,Y_MAX, 0,65535)+0.5;
    #endif

    galvo.goTo( mapx, mapy ); //Make sure to hit the commanded position
    beginNext = true;
    interpolCnt=0;
    return;
  }
  else if (currentMove)
  {
    uint64_t t = (*currentMove).moveLengthNanos; 
    double fraction_of_move = (double)(_now-startNanos)/t;
    double x = (previousTarget.x + (distx*fraction_of_move));
    double y = (previousTarget.y + (disty*fraction_of_move));

    interpolCnt++;
    #ifdef LASER_IS_SYNRAD
      laser.handleLaser();    
    #endif

    #ifdef INVERSE_X
    double mapx = map(x, 0.0,X_MAX, 65535,0)+0.5;
    #else
    double mapy = map(x, 0.0,X_MAX, 0,65535)+0.5;
    #endif
    
    #ifdef INVERSE_Y
    double mapx = map(y, 0.0,Y_MAX, 65535,0)+0.5;
    #else
    double mapy = map(y, 0.0,Y_MAX, 0,65535)+0.5;
    #endif

    if(interpolCnt%10==0)
    {
      Serial.print("\ninterpol - go to pos x: ");Serial.println(currentTarget.x);Serial.println(" y:");Serial.println(currentTarget.y);
      Serial.print("\ninterpol - time left:");Serial.println((int)(endNanos - _now));
    }
    galvo.goTo( mapx, mapy );
    return ;
  }
  else 
  {
    //Serial.println("Idle");
    #ifdef LASER_IS_SYNRAD
      laser.handleLaser();    
    #endif
  }
  return;
}

void loop() {  
  
  serialReciever.handleSerial();
  
  process();
    


}


