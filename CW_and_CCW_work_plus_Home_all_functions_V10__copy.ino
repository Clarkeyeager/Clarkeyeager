String CodeVersion = "V10";
String CodeDate = "2/25/2024";
//  Author: Clarke Yeager

String msg1 = "Current dome rotation in degrees ";
String msg2 = "What angle of dome rotation would you like to start at between 0 (north) and 359 degrees";
String msg3 = "the dome rotational angle you requested in degrees is ";

#include "Wire.h"
#include "I2C_eeprom.h"

I2C_eeprom ee(0x57, I2C_DEVICESIZE_24LC256);  //establishes the I2C address for the 24LC256 EEPROM

// ==============This is the code for the dome controller board V3=====================

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-- This code works manual and program control operation Don't change it --!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//                                      CWpgm & CCWpgm work as well as both bush buttons
//                                      This also takes care of the overshoot not counting tach pulses
//                                      This also takes converts tach pulses to rotational angle
//                                      This also causes the home to move the shortest distance to home position
//                                      This version takes care of going past zero degrees with push buttons

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// IF RUNING WITH USB MAKE SURE THAT THE SERIAL MONITOR HAS "NO LINE ENDING" SELECTED
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//                      &&&& EEPROM address allocation &&&&
//  ===============================================================================================
int positionCountAddress = 12;  // This keeps the current encoder count. Each address uses two bytes  !!!!!!!!!!!!!!!!!!!!!!

//  ---------------------------------------------------------------------------------------------------

//  -------------------------  pin assignments  --------------------------------
int homeSwitch = 8;  // this switch drives the dome to home position
int CWswitch = 6;    // This switch manually drives the dome clockwise
int CCWswitch = 7;   // This switch manually drives the dome counter-clockwise

int CWswitchState = 0;
int CWswStatus = 0;
int CCWswitchState = 0;

int SlowDownHall = 10;  // This will tell the dome that it is close to home and slow down
int StopHall = 9;       // This will stop the dome from rotating during a homing
//  ---  The following is the "H" configuration drive output to drive the motor in both directions ---
int TopLeftDrive = 3;      // "H" drive top left output PWM
int TopRightDrive = 5;     // "H" drive top right output PWM
int BottomLeftDrive = 4;   // "H" drive bottom left output
int BottomRightDrive = 2;  // "H" drive bottom right output

int TP = A1;

//  ----------------------------------------------------------------------------

void write2BytesIntIntoEEPROM(int positionCountAddress, int number) {
  ee.writeByte(positionCountAddress, (number >> 8) & 0xFF);  //This writes two bytes to EEPROM, byte1 and byte2
  ee.writeByte(positionCountAddress + 1, number & 0xFF);     // byte1 is shifted and then byte2 is added to the result of the shift
}

int read2BytesFromEEPROM(int positionCountAddress)  //This reads two bytes to EEPROM, byte1 and byte2
{
  return (ee.readByte(positionCountAddress) << 8) + ee.readByte(positionCountAddress + 1);
}

//  ---------------------------  enter Settings  ------------------------------------
float DomeRingHoles = 175;  //Number of holes in the dome ring
float GearCogs = 14;        //Number of gocs on the gear
float GearTachs = 24;       //Number of Tachs per gear rotation
//                      -------  calculations  ------
float TachsPerDomeRev = ((DomeRingHoles / GearCogs) * GearTachs);  //Tachs per 360 degree of dome rotation
float Ratio = (360 / TachsPerDomeRev);                             //Ratio of tachs for 360 degrees dome rotation

//  ---------------------------  variables  ------------------------------------
int TopLftDrive = 0;       //            Drive FETs for the motor "H" drive
int TopRtDrive = 0;        //            Drive FETs for the motor "H" drive
int BotLftDrive = 0;       //            Drive FETs for the motor "H" drive
int BotRtDrive = 0;        //            Drive FETs for the motor "H" drive
int minPower = 75;         //            This establishes the minimum drive current to ensure the dome will rotate 12*(80/255)=4.47V
int CWPower;               //            This is the PWM controled CW motor drive current
int CCWPower;              //            This is the PWM controled CCW motor drive current
int RampSpeed = 3;         //            This controls how fast the PWM dirve current increases and decreases
int encoderCnt = 0;        //            This keeps track of the encoder count ( this will also stored in EEPROM)
int encoderPin = 11;       //            This is the interupt pin where the encoder signal is attached
int slowDownHall;          //            This will iniciate moving the dome to home position
int slowDownHallPin = 10;  //            This is the interrupt pin where the Hall signal is attached
int homeStopHallPin = 9;

int encoderInterruptEnable = 1;
int SwitchHome;
int goHome = 1;
int goHomeState;
int stopAtHome = 1;
int positionCount;
int angle;
int begin = 0;
int moveCount;
int CWpgm = 0;   //CWpgm = 0 that says no control by the pgm, if it is = 1 that says it will be controlled by the pgm
int CCWpgm = 0;  //CCWpgm = 0 that says no control by the pgm, if it is = 1 that says it will be controlled by the pgm
int eeOutput;
int SwitchCW;
int SwitchCCW;
int CWcountEnable;
int CCWcountEnable;
int CWcoast = 0;
int CCWcoast = 0;
int HomeOffset = 20;  // This is the offset from true notrh to the home sensor location in degrees
                      // this is used to determine the shortest direction to home, rotate CW or CCW
int HomeAngle = 140;  // This sets the desired angle in degrees for the home location
unsigned long currentTime;

// previous time for the tasks depending upon time.
unsigned long prevTime_T1 = millis();

// time intervals for the tasks
unsigned long interval_T1 = 150;  // sample frequency

//  ----------------------------------------------------------------------------

void encoderInterrupt()  // This interrupt counts the encoder pulses
{

  //  delayMicroseconds(1000);

  if (encoderInterruptEnable == 1) {

    if ((digitalRead(CWswitch)) == 0 || CWcountEnable == 1) {
      encoderCnt = (encoderCnt + 1);

      if (encoderCnt > (360 / Ratio))
        encoderCnt = 1;
      write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);
      //       Serial.println(encoderCnt);

      digitalWrite(TP, 1);
      delay(30);
      digitalWrite(TP, 0);

    } else if ((digitalRead(CCWswitch)) == 0 || CCWcountEnable == 1) {
      encoderCnt = (encoderCnt - 1);

      if (encoderCnt < 1) {
        encoderCnt = (360 / Ratio);
        //        write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);
        //        Serial.print("encoderCnt -1 = ");
      }
      //      Serial.println(encoderCnt);
    }
  }
}

void slowDownInterrupt() {  //  Slow down just before reaching home
  delayMicroseconds(15000);
  goHomeState = 0;
}
void homeStopHallInterrupt() {  //  Stop when home location is reached
  delayMicroseconds(15000);
  stopAtHome = 0;
}

void setup() {
  {
    Serial.begin(9600);
    delay(2000);

    ee.begin();

    delay(10);
    pinMode(TP, OUTPUT);
    pinMode(TopLeftDrive, OUTPUT);
    pinMode(TopRightDrive, OUTPUT);
    pinMode(BottomLeftDrive, OUTPUT);
    pinMode(BottomRightDrive, OUTPUT);
    pinMode(CWswitch, INPUT_PULLUP);
    pinMode(CCWswitch, INPUT_PULLUP);
    pinMode(homeSwitch, INPUT_PULLUP);
    pinMode(slowDownHallPin, INPUT);  // Home location Hall
    pinMode(homeStopHallPin, INPUT);  // Home location Hall
    attachInterrupt(digitalPinToInterrupt(encoderPin), encoderInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(slowDownHallPin), slowDownInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(homeStopHallPin), homeStopHallInterrupt, RISING);
  }
  CCWPower = minPower;  // Set minimum power required to ensure the dome will rotate CW
  CWPower = minPower;   // Set minimum power required to ensure the dome will rotate CCW
}

void loop() {

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  //This asks what angle you want the rotator to move to on power up
  //once you enter an angle it goes into "angle"
  //after you enter the angle you want the manual switches work
  //There could be an or condition between the mechanical switches and a variable to operate the rotator


  if (begin == 0) {

    delay(5);

    eeOutput = read2BytesFromEEPROM(positionCountAddress);
    delay(20);
    encoderCnt = eeOutput;
    Serial.println(" ");
    Serial.print(msg1);  //This converts tach count back into angle -- Current dome rotation in degreescurrent

    Serial.println(int(eeOutput * Ratio));  // This converts tach pulses to display in degrees

    Serial.println("*************************************** ");
    Serial.println(msg2);  //What angle of dome rotation would you like to move to between 0 (north) and 359 degrees
    Serial.println(" ");

    while ((Serial.available() == 0) && ((((digitalRead(CWswitch) == 1) && (digitalRead(CCWswitch)) == 1))) && (digitalRead(homeSwitch)) == 1) {
      /*do nothing*/
    }

    angle = (Serial.parseInt() / Ratio);  // This converts angle to number of pulses

    if (angle > read2BytesFromEEPROM(positionCountAddress)) {
      angle = (angle - 3);
      Serial.print(msg3);                      //the dome rotational angle you requested in degrees is
                                               //    Serial.print(" here too ");
      Serial.print(int((angle + 3) * Ratio));  // This converts tach pulses to degrees for display
                                               //    Serial.print((angle + 3));  // This converts tach pulses to degrees for display
      Serial.println(" ");
      Serial.println(" ");
      Serial.println(" ");
    } else {
      angle = (angle + 3);
      Serial.print(msg3);                        //the dome rotational angle you requested in degrees is
                                                 //    Serial.print(" here  ");
      Serial.println(int((angle - 3) * Ratio));  // This converts tach pulses to degrees for display
      Serial.println(" ");
      Serial.println(" ");
    }

    if ((angle < 0) || (angle > ((360 - 4) / Ratio)))  // check for valid input value between 0 and 360 degrees (360 * Ratio)  ???????????????????????????????????????????????
    {
      Serial.println("value entered is not between 0 and 360 degrees");
      Serial.println("Please enter a valid angle");
    } else {
      moveCount = (angle - eeOutput);

      SwitchCW = (digitalRead(CWswitch));
      SwitchCCW = (digitalRead(CCWswitch));
      SwitchHome = (digitalRead(homeSwitch));

      if ((SwitchCW == 1) && (SwitchCCW) == 1 && (SwitchHome) == 1) {

        Serial.println("got here start of loop ");


        if (moveCount > 0) {
          CWpgm = 1;
        }
        CCWpgm = 1;
      } else {
        CWpgm = 0;
        CCWpgm = 0;
      }

      // --------------------------- CW control routine ---------------------------
      encoderInterruptEnable = 1;

      currentTime = millis();

      eeOutput = read2BytesFromEEPROM(positionCountAddress);
      delay(10);
      encoderCnt = eeOutput;

      digitalWrite(BottomRightDrive, (1));  // Turn on the lower Right drive FET
      delay(5);

      CWcountEnable = 1;  //  Allow tach pulses to be counted until motor drive is turned off

      CWswitchState = digitalRead(CWswitch);  // Read the CWswitch and record the CW switch state CWswitchState will = 0 if switch is pressed

      while (CWswitchState == 0 || CWpgm == 1)  // Loop as long as the switch is pressed
      {

        CCWpgm = 0;

        if (CWPower < 255)  // Ramp PWM up from minimum power to rotate the dome to 100% full power
        {
          CWPower = CWPower + 1;
          delay(RampSpeed);
          analogWrite(TopLeftDrive, (CWPower));
          delay(1);
        }

        currentTime = millis();

        // Task 1 : test millis
        if (currentTime - prevTime_T1 > interval_T1) {

          //          if (((angle - CWcoast) - encoderCnt) <= 0) {  //Match encoder count with the desired angle//??????????????????????
          if ((angle - encoderCnt) <= 0) {  //Match encoder count with the desired angle//??????????????????????

            CWpgm = 0;
          }

          prevTime_T1 = currentTime;
        }

        CWswitchState = digitalRead(CWswitch);  // Up date switch status
      }
      //               ----------------------
      while (CWPower > minPower)  // When the CW switch is released decrement the PWM drive until it gets to
      //the minimum current to ensure dome rotation and then turn of the drive current
      {
        CWPower = CWPower - 1;
        delay(RampSpeed);
        analogWrite(TopLeftDrive, (CWPower));
        delay(1);
      }

      analogWrite(TopLeftDrive, (0));
      digitalWrite(BottomRightDrive, (0));
      delay(5);  // Delay to make sure the turn off time before upper Right FET could be turned on.

      //      delay(1000);

      CWcountEnable = 0;  //  Stop counting tach pulses when motor drive is turned off

      write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);
      delay(10);

      // --------------------------- CCW control routine ---------------------------
      encoderInterruptEnable = 1;

      currentTime = millis();

      eeOutput = read2BytesFromEEPROM(positionCountAddress);
      delay(10);
      encoderCnt = eeOutput;

      digitalWrite(BottomLeftDrive, (1));  // Turn on the lower Left drive FET
      delay(5);

      Serial.println("got here start of CCW ");  //??????????????????????????????????????????????????????????????????????????????????????????????????

      CCWcountEnable = 1;  //  Allow tach pulses to be counted until motor drive is turned off

      CCWswitchState = digitalRead(CCWswitch);  // Read the CCWswitch and record the CCW switch state CWswitchState will = 0 if switch is pressed

      while (CCWswitchState == 0 || CCWpgm == 1)  // Loop as long as the switch is pressed
      {
        if (CCWPower < 255)  // Ramp PWM up from minimum power to rotate the dome to 100% full power
        {
          CCWPower = CCWPower + 1;
          delay(RampSpeed);
          analogWrite(TopRightDrive, (CCWPower));
          delay(1);
        }

        currentTime = millis();

        // Task 1 : test millis
        if (currentTime - prevTime_T1 > interval_T1) {

          if (((angle + CCWcoast) - encoderCnt) > -1) {  //Match encoder count with the desired angle
            CCWpgm = 0;
          }

          prevTime_T1 = currentTime;
        }

        CCWswitchState = digitalRead(CCWswitch);  // Update switch status
      }

      while (CCWPower > minPower)  // When the CCW switch is released decrement the PWM drive until it gets to
      //the minimum current to ensure dome rotation and then turn of the drive current
      {
        CCWPower = CCWPower - 1;
        delay(RampSpeed);
        analogWrite(TopRightDrive, (CCWPower));
        delay(1);
      }
      analogWrite(TopRightDrive, (0));
      digitalWrite(BottomLeftDrive, (0));
      delay(5);  // Delay to make sure the turn off time before upper Right FET could be turned on.

      //      delay(1000);

      CCWcountEnable = 0;  //  Allow tach pulses to be counted until motor drive is turned off

      if ((encoderCnt * CCWcoast) < 0) {
        encoderCnt = 0;
      }

      write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);
      delay(10);

      CCWpgm = 0;


      // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

      // --------------------------- Homing control routine ---------------------------

      /* When the Home switch is pressed for more than 2 seconds the dome will rotate to the home position
        There are 175 holes in the ring per 360 deg revolution of the dome
        There are 14 pins on the motor per rev
        174/14 = 12.5 revolutions of the motor per dome rotation
        24 encoder pulses per motor rev * 12.5 = 300 encoder pulses per dome rotation
        360 degrees / 300 pulses = Ratio degrees / encoder pulse
        example for 8 foot dome scale = 360 / 300 = 1.2
        It takes about 60 sec for 1 rev of the dome
        21.2 inch / 24 = .8833 inches per encoder pulse
        .8833 / 200 ms pulses per second = 4.4165 inches per second
        During the Home routine the encoder interrupt is disabled so the count will not be recorded in EEPROM
        When at home the encoder count will be set to an angle such that the dome is pointed in the desired direction and stored in EEPROM
      */

      if (SwitchHome == 0)  // No debounce is used because not important
      {
        encoderInterruptEnable = 0;  //                     should be  0

        Serial.println("got here start of home ");


        if (SwitchHome == 0) {

          Serial.print("Home angle ");
          Serial.print(int(HomeAngle));
          Serial.println(" has been requested");
          Serial.println(" ");

          if (read2BytesFromEEPROM(positionCountAddress) > ((180 / Ratio) - (HomeOffset / Ratio)))  // accounts for the home sensor being offset from north
          //                                                                                       in tachs  HomeOffset is in tachs
          {
            Serial.print("                    Home sensor offset from north =  ");
            Serial.print(HomeOffset);
            Serial.println(" degrees from north");

            Serial.print("                    Position was greater than ");
            Serial.print(180 - (HomeOffset));
            Serial.println(" degrees from north");

            digitalWrite(BottomRightDrive, 1);  // Turn on the lower right drive FET
            delay(5);
            goHomeState = goHome;  // Read the CWswitch and record the CW switch state

            if ((digitalRead(goHomeState)) == 1)  // No debounce is used because not important
            {
              digitalWrite(BottomRightDrive, 1);  // Turn on the lower right drive FET
              delay(5);
            }

            while (goHomeState == 1)  // Loop as long as you are still moving toward home location
            {
              if (CWPower < 255)  // Ramp PWM up from minimum power to rotate the dome to 100% full power
              {
                CWPower = CWPower + 1;
                delay(RampSpeed);
                analogWrite(TopLeftDrive, (CWPower));
              }

              delay(1);
            }

            //               ---------- Start to slow down ------------
            while (CWPower > minPower)  // When the CW motion is ready to stop decrement the PWM drive until it  gets to
            //the minimum drive current to ensure slow dome rotation until home Hall is detected then stop
            {
              CWPower = CWPower - 1;
              delay(RampSpeed);
              analogWrite(TopLeftDrive, (CWPower));
            }

            while (stopAtHome == 1) {
              analogWrite(TopLeftDrive, (minPower));
            }

            analogWrite(TopLeftDrive, 0);  // Turn off both upper left drive and lower right drive
            digitalWrite(BottomRightDrive, 0);
            delay(5);  // Delay to make sure the turn off time before upper right FET could be turned on.
            //        }
            stopAtHome = 1;

            //              ----------  leave power at minimum until home magnet interrupt Then Stop ----------

            stopAtHome = 1;
            goHomeState = 1;

            write2BytesIntIntoEEPROM(positionCountAddress, (HomeAngle / Ratio));
          } else {

            Serial.print("                    Home sensor offset from north =  ");
            Serial.print(HomeOffset);
            Serial.println(" degrees from north");

            Serial.print("                    Position was less than ");
            Serial.print(180 - (HomeOffset));
            Serial.println(" degrees from north");

            digitalWrite(BottomLeftDrive, 1);  // Turn on the lower left drive FET
            delay(5);
            goHomeState = goHome;  // Read the CWswitch and record the CW switch state

            if ((digitalRead(goHomeState)) == 1)  // No debounce is used because not important
            {
              digitalWrite(BottomLeftDrive, 1);  // Turn on the lower left drive FET
              delay(5);
            }

            while (goHomeState == 1)  // Loop as long as you are still moving toward home location
            {
              if (CCWPower < 255)  // Ramp PWM up from minimum power to rotate the dome to 100% full power
              {
                CCWPower = CCWPower + 1;
                delay(RampSpeed);
                analogWrite(TopRightDrive, (CCWPower));
              }

              delay(1);
            }

            //               ---------- Start to slow down ------------
            while (CCWPower > minPower)  // When the CCW motion is ready to stop decrement the PWM drive until it  gets to
            //the minimum drive current to ensure slow dome rotation Until home Hall is detected then stop
            {
              CCWPower = CCWPower - 1;
              delay(RampSpeed);
              analogWrite(TopRightDrive, (CCWPower));
            }

            while (stopAtHome == 1) {
              analogWrite(TopRightDrive, (minPower));
            }

            analogWrite(TopRightDrive, 0);  // Turn off both upper right drive and lower left drive
            digitalWrite(BottomLeftDrive, 0);
            delay(5);  // Delay to make sure the turn off time before upper right FET could be turned on.
            //        }
            stopAtHome = 1;

            //              ----------  leave power at minimum until home magnet interrupt Then Stop ----------

            stopAtHome = 1;   //done in the homeStopHallInterrupt
            goHomeState = 1;  //done in the slowDownInterrupt

            write2BytesIntIntoEEPROM(positionCountAddress, (HomeAngle / Ratio));

            Serial.print("HomeAngle = ");
            Serial.println(HomeAngle);
          }
        }
      }
    }
  }
}
