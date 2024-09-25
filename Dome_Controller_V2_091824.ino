  #include "Wire.h"
  #include "I2C_eeprom.h"

  String CodeVersion = "Dome_Controller";
  String CodeDate = "9/24/2024";
  //  Author: Clarke Yeager

  String msg1 = "Current dome rotation in degrees ";
  String msg2 = "Enter a legal value angle of dome rotation from north (between 1 and 360 degrees) ";  

  I2C_eeprom ee(0x57, I2C_DEVICESIZE_24LC256);        //establishes the I2C address for the 24LC256 EEPROM

  // ==============This is the code for the dome controller board V3=====================

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-- This dome controller has the following feetures --!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //                                It is based on the compass angle of 1 to 360 degrees
 //                                           (360 degrees is due North)  
  // 
  //                                    Buttons for manual jog control:
  //                                        clockwise rotation
  //                                        counter clockwise rotation
  //                                        Home/Park  (used for parking to a given rotation and establishing calibration)

  //                                    Programmed for the following:
  //                                        Home/Park  (used for parking to a rotation and establishing calibration)
  //                                        Move from any current rotation to any desired rotational angle from 1 to 360 degrees (360 degrees = North)
  //                                        Programmed to always take the shortest distance from the current rotational angle to the desired rotation
  //                                        Programmed to always take the shortest distance to Home/Park

  //                                    Programmed for the following error conditions: 
  //                                        Checks the tach sensor function  --  if tach pulses are missing for a period of time while rotating to the desired angle the rotator stops
  //                                                (an error message is printed out)                                           
  //                                        Checks the Hall sensor function  --  if the Hall sensor doesn't recognize the home magnets for over a full rotation it stops the rotator
  //                                                (an error message is printed out)
  //                                                (intervention is required to correct the error)   
 
  //                                    Features: 
  //                                        This program can be used on any sized dome by specifying the number of cog holes on the paremeter of the dome
  //                                                (Located in the parameter section of code at about line #90)                                           
  //                                        The Home/Park angle can be set to any desired direction for best wind protection

  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  // IF RUNING WITH USB MAKE SURE THAT THE SERIAL MONITOR HAS "NO LINE ENDING" SELECTED
  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

  //                      &&&& EEPROM address allocation &&&&
  //  ===============================================================================================
  int positionCountAddress = 12;  // This keeps the current encoder count. Each address uses two bytes  !!!!!!!!!!!!!!!!!!!!!!

  //  ---------------------------------------------------------------------------------------------------

  //  -------------------------  pin assignments  --------------------------------
  int homeSwitch                  = 8;                               //This switch drives the dome to home position
  int CWswitch                    = 6;                               //This switch manually drives the dome clockwise
  int CCWswitch                   = 7;                               //This switch manually drives the dome counter-clockwise

  int CWswitchState               = 0;
  int CWswStatus                  = 0;
  int CCWswitchState              = 0;

  int SlowDownHall                = 10;                              //This will tell the dome that it is close to home and slow down
  int StopHall                    = 9;                               //This will stop the dome from rotating during a homing

  //  ---  The following is the "H" configuration drive output to drive the motor in both directions ---

  int TopLeftDrive                = 3;                                //"H" drive top left output PWM
  int TopRightDrive               = 5;                                //"H" drive top right output PWM
  int BottomLeftDrive             = 4;                                //"H" drive bottom left output
  int BottomRightDrive            = 2;                                //"H" drive bottom right output

  int TP                          = A1;                               //For interrupt test
  int TP2                         = A2;                               //For interrupt test

  //  ----------------------------------------------------------------------------

  void write2BytesIntIntoEEPROM(int positionCountAddress, int number) {
    ee.writeByte(positionCountAddress, (number >> 8) & 0xFF);          //This writes two bytes to EEPROM, byte1 and byte2
    ee.writeByte(positionCountAddress + 1, number & 0xFF);             //byte1 is shifted and then byte2 is added to the result of the shift
  }

  int read2BytesFromEEPROM(int positionCountAddress)                    //This reads two bytes to EEPROM, byte1 and byte2
  {
    return (ee.readByte(positionCountAddress) << 8) + ee.readByte(positionCountAddress + 1);
  }

  //  ---------------------------  Parameter Settings  ------------------------------------
  float DomeRingHoles             = 175;                                //Number of holes in the dome ring for 8 foot dome
  //float DomeRingHoles           = 262;                                //Number of holes in the dome ring for 12 foot dome  
  float GearCogs                  = 14;                                 //Number of gocs on the gear
  float GearTachs                 = 24;                                 //Number of Tachs per gear rotation
  //                      -------  calculations  ------
  float TachsPerDomeRev = ((DomeRingHoles / GearCogs) * GearTachs);     //Tachs per 360 degree of dome rotation
    //                  ((DomeRingHoles / GearCogs) * GearTachs) = 300  for 8 foot dome
    //                  ((DomeRingHoles / GearCogs) * GearTachs) = 449  for 12 foot dome

  float Ratio = (360 / TachsPerDomeRev);                                //Ratio of tachs for 360 degrees dome rotation

  int ParkAngle                     = 120;                              //This sets the desired angle in degrees from north to the home/park location
                                                                        //This can be adjusted to set 0 degrees to exactly north
  int magOffset                     = 30;                               //This is the angle from North to the home magnet
                                                                        //This is the offset from true north to the home sensor location in degrees

  //  ---------------------------  variables  ------------------------------------
  int TopLftDrive                   = 0;                                 //Drive FETs for the motor "H" drive
  int TopRtDrive                    = 0;                                 //Drive FETs for the motor "H" drive
  int BotLftDrive                   = 0;                                 //Drive FETs for the motor "H" drive
  int BotRtDrive                    = 0;                                 //Drive FETs for the motor "H" drive
  int minPower                      = 100;                               //This establishes the minimum drive current to ensure the dome will rotate 12*(80/255)=4.47V
  int CWPower                       = 0;                                 //This is the PWM controlled CW motor drive current
  int CCWPower                      = 0;                                 //This is the PWM controlled CCW motor drive current
  int RampSpeed                     = 3;                                 //This controls how fast the PWM drive current increases and decreases
  int encoderCnt                    = 0;                                 //This keeps track of the encoder count ( this will also stored in EEPROM)
  int encoderPin                    = 11;                                //This is the interrupt pin where the encoder signal is attached
  int slowDownHall;                                                      //This will initiate moving the dome to home position
  int slowDownHallPin               = 10;                                //This is the interrupt pin where the Hall signal is attached
  int homeStopHallPin               = 9;

  char test;
  char h;

  int encoderInterruptEnable        = 1;
  int SwitchHome;
  int goHome                        = 1;
  int goHomeState;
  int stopAtHome                    = 1;
  int positionCount;
  int angle;
  float scale                       = (360.0/TachsPerDomeRev);                    //
  int CurLoc;
  int CWswDetectedFlag              = 1;
  int CCWswDetectedFlag             = 1;
  int HomeSwDetectedFlag            = 1;
  int Fault = 0;
  int moveCount;
  int moveDecrement;
  int moveDecrementOld;  
  int decrement2                    = 0;
  int largeMove                     = 0;
  int CWpgm                         = 0;                                  //CWpgm = 0 that says no control by the pgm, if it is = 1 that says it will be controlled by the pgm
  int CCWpgm                        = 0;                                  //CCWpgm = 0 that says no control by the pgm, if it is = 1 that says it will be controlled by the pgm
  int eeOutput;
  int SwitchCW                      = 1;
  int SwitchCCW;
  int CWcountEnable;
  int CCWcountEnable;
  int coast                         = 0;
  int CWcoast                       = 0; 
  int CCWcoast                      = -0;
  int HomeOffset                    = 30;                                  //This is the offset from true north to the home sensor location in degrees
                                                                           //this is used to determine the shortest direction to home, rotate CW or CCW
  int HallErrorFlag                 = 0;

  unsigned long currentTime;
  unsigned long currentTime2;
  unsigned long currentTime3;  

  unsigned long prevTime_T1 = millis();                                    //previous time for the tasks depending upon time.
  unsigned long prevTime_T2 = millis();                                    //checks for a system failure if interrupts are missing for a while
  unsigned long prevTime_T3 = millis();                                    //This is a timmer to check Home Hall system for an error

  // time intervals for the tasks
  unsigned long interval_T1 = 150;                                         //sample frequency to check the tach count
  unsigned long interval_T2 = 600;    //300                                     //sample frequency to check if tach pulses are missing

  unsigned long interval_T3 = (70000 * TachsPerDomeRev / 300);             //sample frequency to check to see if 1.2 revolutions without sensing the magnet    

  //  ----------------------------------------------------------------------------
   
  void encoderInterrupt()                                                  //This interrupt counts the encoder pulses
  {
   
      delayMicroseconds(1000);                                             //The count gets messed up without this delay, some timing issues with interrupt

      if (encoderInterruptEnable == 1) 
      {

          encoderInterruptEnable = 0;                                      //This turns off interrupts as soon as it gets an interrupt to prevent false triggers on the negative transition
                                                                           // Interrupts are head off until a negative level is detected in the main routing when the interrupt enable is turned on.

          if ((digitalRead(CWswitch)) == 0 || CWcountEnable == 1) 
          {
            encoderCnt = (encoderCnt + 1);
            CurLoc = (CurLoc + 1);
            decrement2 = decrement2 - 1;

            if (encoderCnt > (360 / Ratio))
              encoderCnt = 1;

              write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);

              moveCount = moveCount - 1;
              moveDecrement = moveDecrement - 1;

            digitalWrite(TP, 1);
            digitalWrite(TP, 1);
            digitalWrite(TP, 1);
            digitalWrite(TP, 1);
            digitalWrite(TP, 1);

          } 
          else if ((digitalRead(CCWswitch)) == 0 || CCWcountEnable == 1) 
          {
              encoderCnt = (encoderCnt - 1);
              CurLoc = (CurLoc - 1);
              decrement2 = decrement2 - 1;
              moveCount = moveCount - 1;
              moveDecrement = moveDecrement - 1;         

              digitalWrite(TP, 1);
              digitalWrite(TP, 1);
              digitalWrite(TP, 1);
              digitalWrite(TP, 1);
              digitalWrite(TP, 1);

              if (encoderCnt < 1) 
              {
              encoderCnt = (360 / Ratio);
              CurLoc = (360 / Ratio);
              }
        }
    }

    digitalWrite(TP, 0);

  }// encoderInterrupt

  /******************************************************************************
  * slowDownInterrupt
  *
  ******************************************************************************/
  void slowDownInterrupt() 
  {                                                                         //Slow down just before reaching home
    delayMicroseconds(15000);
    goHomeState = 0;
  }
  void homeStopHallInterrupt() 
  {  //Stop when home location is reached
    delayMicroseconds(15000);
    stopAtHome = 0;
  }  // homeStopHallInterrup

  /******************************************************************************
  * setup
  *
  ******************************************************************************/

  void setup() 
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
      pinMode(slowDownHallPin, INPUT);                                               //Home location Hall
      pinMode(homeStopHallPin, INPUT);                                               //Home location Hall
      attachInterrupt(digitalPinToInterrupt(encoderPin), encoderInterrupt, RISING);  //RISING  HIGH   FALLING
      attachInterrupt(digitalPinToInterrupt(slowDownHallPin), slowDownInterrupt, RISING);
      attachInterrupt(digitalPinToInterrupt(homeStopHallPin), homeStopHallInterrupt, RISING);
    
      CCWPower = minPower;                                                  //Set minimum power required to ensure the dome will rotate CW
      CWPower = minPower;                                                   //Set minimum power required to ensure the dome will rotate CCW
  }

  /******************************************************************************
  * loop
  *
  ******************************************************************************/
 void loop() 
  {
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      //This asks what angle you want the rotator to move to on power up
      //once you enter an angle it goes into "angle"
      //after you enter the angle you want the manual switches work
      //There could be an /or condition between the mechanical switches and a variable to operate the rotator

      eeOutput = read2BytesFromEEPROM(positionCountAddress);
      delay(20);
      encoderCnt = eeOutput;
    
      Serial.println(" ");            
      Serial.print(msg1);                                                   //Current dome rotation in degrees

      Serial.println(int(eeOutput * Ratio));                                //This converts tach pulses to display in degrees

      Serial.println("*************************************** ");
      Serial.println(" ");      
      Serial.println(msg2);                                                 //Enter a legal value angle of dome rotation between 1 and 360 degrees (north)
      Serial.println(" ");

      while ((Serial.available() == 0) 
            && ((CWswDetectedFlag == 1) && (CCWswDetectedFlag == 1)) 
            && (HomeSwDetectedFlag) == 1) 
      {
          CWswDetectedFlag = digitalRead(CWswitch);
          CCWswDetectedFlag = digitalRead(CCWswitch);
          HomeSwDetectedFlag = digitalRead(homeSwitch);
      }

      angle = (Serial.parseInt(SKIP_NONE) / Ratio);                         //This converts angle to number of pulses

      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     moveCount = (angle - eeOutput);

    CurLoc = read2BytesFromEEPROM(positionCountAddress);                   //This just records the starting location stored in the EEprom before modifying it with steps
                                                                            //This will replace the moveCount so it won't get reset at the 360 degree wrap around like the moveCount does
                                                                            //This will replace the moveCount when the move is >180 degrees to go to the new position in the shortest distance.

     //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     char ch = Serial.read();

     largeMove = 0;

     switch (ch) 
      {
        // ===================================================
        case 'H':
        case 'h':
        case 'P':
        case 'p':

            HomeSwDetectedFlag = 0;

          break;

          Serial.println("If an illegal entry Has Been Entered, Please Try Again");
      } // switch ch

    if (abs(angle - CurLoc) > (TachsPerDomeRev / 2))                         //determines if the mew - old angle is more than 180 degrees from the last angle
    {                                                                        //this will be used to move the dome in the shortest direction ot the desired angle 
    largeMove = 1;
    } 
    else 
    {
    largeMove = 0;
    }

      if ((CWswDetectedFlag == 0) || (CCWswDetectedFlag == 0) || (HomeSwDetectedFlag == 0) || angle != 0) 
      {

          if (angle > read2BytesFromEEPROM(positionCountAddress)) 
          {
              angle = (angle - coast);
              Serial.print(" ___________________________ the dome rotational angle you requested in degrees is *** ");
              Serial.print (int(angle * scale) + 1);
              Serial.print(" *** ");              
              Serial.println(" ___________________________ ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" "); 
              Serial.println(" ");                                                           
          } 
          else 
          {
           angle = (angle + coast);
           if(angle + 1 == 1)
            {
              Serial.println(" ___________________________ You requested a Home/Park command ___________________________");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" "); 
              Serial.println(" ");
              Serial.println(" "); 
              Serial.println(" ");              
            }
            else
            {
              Serial.print(" ___________________________ the dome rotational angle you requested in degrees is *** ");                          
              
                     //             Serial.print(msg3);                      //the dome rotational angle you requested in degrees is  (msg3)
              Serial.print (int(angle * scale) + 1);
              Serial.print(" *** ");
              Serial.println(" ___________________________ ");               
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" ");
              Serial.println(" "); 
              Serial.println(" ");
              Serial.println(" "); 
              Serial.println(" ");              
            }                                          
          }

        if ((angle < 0) || (angle > ((360 - coast) / Ratio)))                //check for valid input value between 0 and 360 degrees (360 * Ratio)
        {
        Serial.println("         value entered is not between 0 and 360 degrees");
        Serial.println("               Please enter a valid angle");
        Serial.println(" ");
        Serial.println(" "); 
        Serial.println(" ");

        } 
        else 
        {

          decrement2 = abs((angle - eeOutput));

            if (largeMove == 1)                                             //this routine calculates how many tach steps are required to move to desired rotation  
            {
                 if ((angle - eeOutput) < 1) 
                {
                  moveDecrement = (TachsPerDomeRev - eeOutput + angle);
                } 
                  else 
                {
                      moveDecrement = (TachsPerDomeRev - angle + eeOutput);
                }

            } 
              else 
            {
               
               moveDecrement = abs(angle - eeOutput);
            }

               SwitchCW = CWswDetectedFlag;
               SwitchCCW = CCWswDetectedFlag;
               SwitchHome = HomeSwDetectedFlag;

              if ((SwitchCW == 1) && (SwitchCCW) == 1 && (SwitchHome) == 1) 
            {

              //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                                                                           //this routing determines if count exceeds 180 degrees  
                                                                           //this rotates the dome in the shortest direction                                                                         

               if (abs(angle - CurLoc) > (TachsPerDomeRev / 2))          
               {
               largeMove = 1;



                   if ((TachsPerDomeRev / 2) - (CurLoc - angle) < 1) 
                   {
                   CWpgm = 1;
                   } 
                   else 
                   {
                  CCWpgm = 1;
                   }
               }  
               else 
               {

                   if ((angle - CurLoc) > 1) 
                    {
                    CWpgm = 1;
                    } 
                    else 
                    {
                    CCWpgm = 1;
                    
                    }
                }

             } 
              else 
             {
             CWpgm = 0;                                                     //stop CW and CCW drive program
             CCWpgm = 0;
             }
        
         // --------------------------- CW control routine ---------------------------
          encoderInterruptEnable = 1;

             // Task 2 : test millis 
          currentTime = millis();                      
          currentTime2 = millis();

          eeOutput = read2BytesFromEEPROM(positionCountAddress);
          delay(10);
          encoderCnt = eeOutput;

          digitalWrite(BottomRightDrive, (1));                              //Turn on the lower Right drive FET
          delay(5);

          CWcountEnable = 1;                                                //Allow tach pulses to be counted until motor drive is turned off

          CWswitchState = digitalRead(CWswitch);                            //Read the CWswitch and record the CW switch state CWswitchState will = 0 if switch is pressed
          CWswDetectedFlag = digitalRead(CWswitch);                         //Read the CWswitch and record the CW switch state CWswitchState will = 0 if switch is pressed

          currentTime2 = millis();
          prevTime_T2 = currentTime2;                                      //checks for a system failure if interrupts are missing for a while

           while (CWswitchState == 0 || CWpgm == 1)                         //Loop as long as the switch is pressed
           { 

                 if (CWPower < 255)                                         //Ramp PWM up from minimum power to rotate the dome to 100% full power
                 {
                 CWPower = CWPower + 1;
                 delay(RampSpeed);
                 analogWrite(TopLeftDrive, (CWPower));
                 delay(1);
                 }

                 currentTime = millis();

                 if (currentTime - prevTime_T1 > interval_T1)               //This timmer is used to periodically check the tach count
                 { 
                   currentTime2 = millis();

                    if (currentTime2 - prevTime_T2 > interval_T2)           //This timmer is used to check if tach intterrups are missing
                     {
                         if(moveDecrementOld == moveDecrement)                  
                         {
                        
                         CWpgm = 0; 
                         Serial.println(" ");
                         Serial.println(" ");
                         Serial.println(" ");                                                               
                         Serial.println(" ");
                         Serial.println("     ___________________________________ There has been a system failure ___________________________________");
                         Serial.println("     _________________________ Execute a home command then move to desired rotation _________________________"); 
                         Serial.println(" ");
                         Serial.println(" ");
                         Serial.println(" ");                                                               
                         Serial.println(" ");
                         }
                     moveDecrementOld = moveDecrement;
                             
                     prevTime_T2 = currentTime2;
                     }

                   //##########################################################################
                    if (largeMove == 1)
                    {

                       if (decrement2 <= 0) 
                        {                                                    // Match encoder count with the desired angle
                       CWpgm = 0;
                        }

                    } 
                   else 
                   {
                       if (((angle + CCWcoast) - encoderCnt) < -1) 
                       {                                                     //Match encoder count with the desired angle
                       CWpgm = 0;
                       }
                   }

                    if (largeMove == 1) 
                    {
                       if ((moveDecrement - CWcoast) <= 0)                   //Match encoder count with the desired angle
                           {
                          CWpgm = 0;
                           }
                    } 
                    else 
                    {
                        if ((angle - encoderCnt) - CWcoast <= 0)             //Match encoder count with the desired angle
                        {
                        CWpgm = 0;
                        }
                    }

                   prevTime_T1 = currentTime;                               //This timmer is used to periodically check the tach count
                 }
                  CWswitchState = digitalRead(CWswitch);                    //Up date switch status
                  CWswDetectedFlag = CWswitchState;                         //Up date switch status

              if (digitalRead(encoderPin) == 0) 
              {
              encoderInterruptEnable = 1;                                   //This enables interrupts after the sensor signal has gone negative. Interrupts were disable
                                                                            //once the interrupt was recognized on the positive transition of the tach sensor, preventing
                                                                            //false interrupts on the falling edge of the tach sensor.
              }
           }

           while (CWPower > minPower)                                        //When the CW switch is released decrement the PWM drive until it gets to
                                                                             //the minimum current to ensure dome rotation and then turn of the drive current
            {
             CWPower = 0;     

            delay(RampSpeed);
            analogWrite(TopLeftDrive, (CWPower));
            delay(1);

               if (digitalRead(encoderPin) == 0) 
               {
                encoderInterruptEnable = 1;                                  //This enables interrupts after the sensor signal has gone negative. Interrupts were disable
                                                                             //once the interrupt was recognized on the positive transition of the tach sensor, preventing
                                                                             //false interrupts on the falling edge of the tach sensor.
               }
            }

            analogWrite(TopLeftDrive, (0));                                  //turn off motor
            digitalWrite(BottomRightDrive, (0));
            delay(250);                                                      //Delay to make sure the turn off time before upper Right FET could be turned on.

           CWcountEnable = 0;                                                //Stop counting tach pulses when motor drive is turned off

            write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);
            delay(10);

            encoderInterruptEnable = 1;


           // --------------------------- CCW control routine ---------------------------
           encoderInterruptEnable = 1;

           currentTime = millis();

           eeOutput = read2BytesFromEEPROM(positionCountAddress);
           delay(10);
           encoderCnt = eeOutput;

           digitalWrite(BottomLeftDrive, (1));                               //Turn on the lower Left drive FET
           delay(5);

           CCWcountEnable = 1;                                                //Allow tach pulses to be counted until motor drive is turned off

           CCWswitchState = digitalRead(CCWswitch);                           //Read the CCWswitch and record the CCW switch state CWswitchState will = 0 if switch is pressed
           CCWswDetectedFlag = CCWswitchState;   
                                        //Up date switch status
 

           currentTime2 = millis();
           prevTime_T2 = currentTime2;                                        //checks for a system failure if interrupts are missing for a while

           while (CCWswitchState == 0 || CCWpgm == 1)                         //Loop as long as the switch is pressed
           {

             if (CCWPower < 255)                                              //Ramp PWM up from minimum power to rotate the dome to 100% full power
             {
             CCWPower = CCWPower + 1;
             delay(RampSpeed);
             analogWrite(TopRightDrive, (CCWPower));
             delay(1);
             }

             // Task 1 : test millis
             currentTime = millis();

                if (currentTime - prevTime_T1 > interval_T1)                  //This timmer is used to periodically check the tach count
                {

                 currentTime2 = millis();
//   prevTime_T2 = currentTime2;                                      //checks for a system failure if interrupts are missing for a while                 

                   if (currentTime2 - prevTime_T2 > interval_T2)              //This timmer is used to check if tach intterrups are missing 
                   {

                       if(moveDecrementOld == moveDecrement)
                       {
                       CCWpgm = 0;
                       Serial.println(" ");
                       Serial.println(" ");
                       Serial.println(" ");                                                               
                       Serial.println(" ");                     
                       Serial.println("     ___________________________________ There has been a system failure ___________________________________");
                       Serial.println("     _________________________ Execute a home command then move to desired rotation _________________________");
                       Serial.println(" ");
                       Serial.println(" ");
                       Serial.println(" ");                                                               
                       Serial.println(" ");                                                               
                       }

                       moveDecrementOld = moveDecrement;
                       prevTime_T2 = currentTime2;                            //checks for a system failure if interrupts are missing for a while
                 }
  
                 if (largeMove == 1) 
                 {
                     if (moveDecrement + CCWcoast <= 0) 
                     {  
                     
                                                                             //Stop rotating when the encoder matches the desired angle//
                      CCWpgm = 0;
                     }

                  } 
                  else 
                  {
                      
                     if ((moveDecrement + CCWcoast) <= 0) 
                     {                                                        //Stop rotating when the encoder matches the desired angle
                     
                     CCWpgm = 0;
                     }
                  }

                   prevTime_T1 = currentTime;                                 //This timmer is used to periodically check the tach count
              }

             CCWswitchState = digitalRead(CCWswitch);                         //Update switch status

                if (digitalRead(encoderPin) == 0) 
                {
                  encoderInterruptEnable = 1;                                 //This enables interrupts after the sensor signal has gone negative. Interrupts were disable
                                                                              //once the interrupt was recognized on the positive transition of the tach sensor, preventing
                                                                              //false interrupts on the falling edge of the tach sensor.
                }
           }

            while (CCWPower > minPower)                                       //When the CCW switch is released decrement the PWM drive until it gets to
                                                                              //the minimum current to ensure dome rotation and then turn of the drive current
            {
             CCWPower = 0;

             delay(RampSpeed);
             analogWrite(TopRightDrive, (CCWPower));
             delay(1);

               if (digitalRead(encoderPin) == 0) 
               {
               encoderInterruptEnable = 1;                                    //This enables interrupts after the sensor signal has gone negative. Interrupts were disable
                                                                               //once the interrupt was recognized on the positive transition of the tach sensor, preventing
                                                                               //false interrupts on the falling edge of the tach sensor.
               }
            }
            analogWrite(TopRightDrive, (0));                                   //turn off motor
            digitalWrite(BottomLeftDrive, (0));
            delay(5);                                                          //Delay to make sure the turn off time before upper Right FET could be turned on.

            CCWcountEnable = 0;                                                //Allow tach pulses to be counted until motor drive is turned off

            delay(250);

          if ((encoderCnt + CCWcoast) < 0)                          
          {
          encoderCnt = 0;
          }

          write2BytesIntIntoEEPROM(positionCountAddress, encoderCnt);           //stor the rotation angle to EEPROM
          delay(10);

          CCWpgm = 0;

          decrement2 = 0;

          encoderInterruptEnable = 1;

         // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

         // --------------------------- Homing control routine ---------------------------
         /*       -------------- 8 foot dome --------------
          When the Home switch is pressed the dome will rotate to the home position
          There are 175 holes in the ring per 360 deg revolution of the dome
          There are 14 pins on the motor per rev
          174/14 = 12.43 revolutions of the motor per dome rotation
          24 encoder pulses per motor rev * 12.5 = 298.29 encoder pulses per dome rotation
          360 degrees / 298.29 pulses = Ratio degrees / encoder pulse
          example for 8 foot dome scale = 360 / 298.29 = 1.207         
          It takes about 60 sec for 1 rev of the dome
          21.2 inch / 24 = .8833 inches per encoder pulse
          .8833 / 200 ms pulses per second = 4.4165 inches per second
          During the Home routine the encoder interrupt is disabled so the count will not be recorded in EEPROM
          When at home the encoder count will be set to an angle such that the dome is pointed in the desired direction and stored in EEPROM

                  -------------- 12 foot dome -------------- 
          When the Home switch is pressed the dome will rotate to the home position
          There are 262 holes in the ring per 360 deg revolution of the dome
          There are 14 pins on the motor per rev
          262/14 = 18.71 revolutions of the motor per dome rotation
          24 encoder pulses per motor rev * 18.71 = 449 encoder pulses per dome rotation
          360 degrees / 449 pulses = Ratio degrees / encoder pulse
          example for 12 foot dome scale = 360 / 449 = .8018)
          It takes about 90 sec for 1 rev of the dome
          21.2 inch / 24 = .8833 inches per encoder pulse
          .8833 / 200 ms pulses per second = 4.4165 inches per second
          During the Home routine the encoder interrupt is disabled so the count will not be recorded in EEPROM
          When at home the encoder count will be set to an angle such that the dome is pointed in the desired direction and stored in EEPROM                
         */

            if (SwitchHome == 0)                                               //No de-bounce is used because not important
            {

             // Task 3 : test millis
             currentTime3 = millis();
             prevTime_T3 = currentTime3;                                       //This is a timmer to check Home Hall system for an error
   
             encoderInterruptEnable = 0;                                       //This turns off interrupts so the tach pulses are not counted

               if (SwitchHome == 0) 
               {

                   if (read2BytesFromEEPROM(positionCountAddress) >= (120/1.2) && read2BytesFromEEPROM(positionCountAddress) <= (300/1.2))  // accounts for the home sensor being offset from north                                       
                  //                                                                                           in tachs  (HomeOffset is in tachs) 

                   {

                     digitalWrite(BottomLeftDrive, 1);                        //Turn on the lower right drive FET                     
                     delay(5);
                     goHomeState = goHome;                                     //Read the CWswitch and record the CW switch state

                        if ((digitalRead(goHomeState)) == 1)                   //No de-bounce is used because not important
                       {
                        digitalWrite(BottomLeftDrive, 1);                     //Turn on the lower right drive FET                        
                        delay(5);
                      }

                       while (goHomeState == 1)                                //Loop as long as you are still moving toward home location
                        {
                           if (CWPower < 255)                                  //Ramp PWM up from minimum power to rotate the dome to 100% full power
                           {
                            CWPower = CWPower + 1;
                            delay(RampSpeed);
                            analogWrite(TopRightDrive, (CWPower));                            
                           }
                           delay(1);

                             if(HallErrorFlag == 0)
                             {
                              currentTime3 = millis();

                                 if (currentTime3 - prevTime_T3 > interval_T3) //This timmer is used to check if Hall system has failed 
                                 {

                                 digitalWrite(BottomRightDrive, 0);            //stop motor if the Hall system has failed 
                                 digitalWrite(BottomLeftDrive, 0);    

                                 Serial.println(" ");
                                 Serial.println(" ");
                                 Serial.println(" ");                                                               
                                 Serial.println(" ");                     
                                 Serial.println("     _____________________________________ There has been a system failure ________________________________________");
                                 Serial.println("     _____________ Dome has rotated more than a complete rotation, technical intervention is required _____________");
                                 Serial.println("     __________________________________ the Home HALL sensor system has failed ____________________________________");                     
                                 Serial.println(" ");
                                 Serial.println(" ");
                                 Serial.println(" ");                                                               
                                 Serial.println(" ");     

                                 prevTime_T3 = currentTime3;                   //This is a timmer to check Home Hall system for an error
                                 HallErrorFlag = 1; 
                                 }
                             }
                        }

                     //               ---------- Start to stop ------------

                         analogWrite(TopRightDrive, 0);                        //Turn off both upper left drive and lower right drive
                         digitalWrite(BottomLeftDrive, 0);                         
                         delay(5);                                            // Delay to make sure the turn off time before upper right FET could be turned on.

                          //              ----------  leave power at minimum until home magnet interrupt Then Stop ----------

                         stopAtHome = 1;
                         goHomeState = 1;
                         HomeSwDetectedFlag = 1;
   
                          write2BytesIntIntoEEPROM(positionCountAddress, (ParkAngle / Ratio));
                         } 
                         else  
                         {

                         digitalWrite(BottomRightDrive, 1);                    //Turn on the lower left drive FET                         
                         delay(5);
                          //            goHomeState = goHome;                 //Read the CWswitch and record the CW switch state
                          goHomeState = 1;                                    //Read the CWswitch and record the CW switch state

                          if ((digitalRead(goHomeState)) == 1)                //No de-bounce is used because not important
                          {
                           digitalWrite(BottomRightDrive, 1);                  //Turn on the lower left drive FET                           
                           delay(5);
                          }

                            while (goHomeState == 1)                          //Loop as long as you are still moving toward home location
                             {

                               if (CCWPower < 255)                           //Ramp PWM up from minimum power to rotate the dome to 100% full power
                                 {
                                 CCWPower = CCWPower + 1;
                                 delay(RampSpeed);
                                 analogWrite(TopLeftDrive, (CCWPower));                                 
                                 }
                                  delay(1);

                                    if(HallErrorFlag == 0)
                                    {
                                      currentTime3 = millis();

                                       if (currentTime3 - prevTime_T3 > interval_T3) //This timmer is used to check if Hall system has failed  
                                        {

                                        digitalWrite(BottomLeftDrive, 0); 
                                        digitalWrite(BottomRightDrive, 0);                                             

                                        Serial.println(" ");
                                        Serial.println(" ");
                                        Serial.println(" ");                                                               
                                        Serial.println(" ");                     
                                        Serial.println("     _____________________________________ There has been a system failure ________________________________________");
                                        Serial.println("     _____________ Dome has rotated more than a complete rotation, technical intervention is required _____________");
                                        Serial.println("     __________________________________ the Home HALL sensor system has failed ____________________________________");                     
                                        Serial.println(" ");
                                        Serial.println(" ");
                                        Serial.println(" ");                                                               
                                        Serial.println(" ");     

                                        prevTime_T3 = currentTime3;
                                        HallErrorFlag = 1; 
                                        }
                                    }
                             }

                            //               ---------- Start to stop ------------

                               analogWrite(TopLeftDrive, 0);                 //Turn off both upper right drive and lower left drive
                               digitalWrite(BottomRightDrive, 0);                               
                               delay(5);                                      //Delay to make sure the turn off time before upper right FET could be turned on.


                               stopAtHome = 1;

                               //              ----------  leave power at minimum until home magnet interrupt Then Stop ----------

                               stopAtHome = 1;                                //done in the homeStopHallInterrupt
                               goHomeState = 1;                               //done in the slowDownInterrupt

                               HomeSwDetectedFlag = 1;

                               write2BytesIntIntoEEPROM(positionCountAddress, (ParkAngle / Ratio));
                       
                         } 
               }
            } 
        }
      }

CCWPower = minPower;
CWPower = minPower; 

     if(angle == 0)
     { 
     Serial.println(" ");
     Serial.println(" ");
     Serial.println(" ");
     Serial.println(" ");
     Serial.println(" ");      
     Serial.println(" ");             
     Serial.println("******* Either Home/Park was entered or an illegal character was entered ******* ");
     Serial.println("*********** If an illegal entry Has Been Entered, Please Try Again ************ ");
     Serial.println(" ");
     }                       
  }
  
