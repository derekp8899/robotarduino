
#include <Timer.h>
#include <Encoder.h>
#include <Math.h>
#include <PID_v1.h>
#include <HX711.h>

// UPDATE_MOTORS|1:1:1?

//defining pins
#define P_LADDER_DIR          25 //for robot, 22 for test
#define P_LADDER_PWM         2 //for robot, 3 for test
#define P_LADDER_ENCA           5
#define P_LADDER_ENCB           3
#define P_SCOOPS_DIR         24 //for robot, 28 for test
#define P_SCOOPS_PWM        4 //for robot, 4 for test
#define P_DUMP_DIR           23 //for robot, 32 for test
#define P_DUMP_PWM          7 //for robot, 5 for test

//proximity sensors
#define P_HOPPER_PB1            0
#define P_HOPPER_PB4            0
#define P_HOPPER_PB2_WAKE       0

// ladder contact switches
#define P_LADDERTOP_INT         49 //The BOTTOM contact switch.  This means the ladder is all the way up
#define P_LADDERBOT_INT         50 //The TOP contact switch.  This means the ladder is all the way down

//motor speed values
#define C_DUMP_SPEED            220
#define C_LADDER_SPEED          150 //0 - 255
#define C_LADDER_SPEED_MULT     1.3 //0 - 255
#define C_SCOOP_SPEED           140

#define MAX_MOTOR_ACCEL_PWM 15.0
#define MAX_LADDER_ACCEL_PWM 10.0

#define POLL_INTERVAL 50

//latter and hopper hopper into digital value 0=low, 1=hight
int lattop = 0;
int latbot = 0;

//motor speed variables
int m_iLadderWantedPWM = 0;
int m_iScoopsWantedPWM = 0;
int m_iDumpWantedPWM = 0;
int m_iLadderLastPWM = 0;
int m_iScoopsLastPWM = 0;
int m_iDumpLastPWM = 0;
int m_AutoMine = 0;
int m_EStop = 0;


int hop_PB1 = 0;
int hop_PB4 = 0;
double d = 0;
int x = 0;
int y = 0;
//boolean checks
bool m_bLadderReset = false;
bool m_bIsLadderOverride = false;
bool m_bIsLadderTopTriggered = false;
bool m_bIsLadderBotTriggered = false;
bool m_bIsHopperPB1Triggered = false;
bool m_bIsHopperPB4Triggered = false;
bool enc = false;
bool re = false;
bool latCntctSwOverride = false;

// LADDER MOTOR
int LADDER_SPEED_AUTO;
// variables for hall limit sensor
// hall pins are relative looking at the mining arm from the front of the robot
const int HALLpinLEFT = 49;
const int HALLpinRIGHT = 50;

int LADDERcount = 0;
int TOParmLIMIT = 0;
int BOTTOMarmLIMIT = 12500; // enter bottom arm limit here
boolean calibrated = false;
boolean DECENTcomplete = false;
boolean BOTTOMreached = false;

// encoder arm motor
#define P_LADDER_ENCODER_A 3 
#define P_LADDER_ENCODER_B 5


// SCOOP MOTOR
const int P_SCOOPS_CURRENT = 0;
int SCOOP_SPEED_MAX_AUTO = 250;
double SCOOP_SPEED_AUTO = 0;
boolean wait = false; //used to keep the belt spinning while the arm is fully extended
int start_up_complete = 0;


// PID SCOOP motor
double SET_SCOOP_CURRENT = 6.0; // enter desired operating current in amps here
double FILTERED_SCOOP_CURRENT = 0;
// links and intial tuning parameters
double aggKp = 9.6401, aggKi = 175.2098, aggKd = 0.0; // these tuning parameters are adjusted to fit the specific process 
// This is the format: PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); 
PID BELTmotorPID(&FILTERED_SCOOP_CURRENT, &SCOOP_SPEED_AUTO, &SET_SCOOP_CURRENT, aggKp, aggKi, aggKd, DIRECT);
double consKp = 7.0, consKi = 100.0, consKd = 0; // these tuning parameters are after the belt reaches speed

// variables for current sensing scoop motor
double RAWsenseLADDER, current, voltage;
double ACSoffset = 2500;
double mVperAmp = 66;

// variables for Kalman filter on current sensor
double P_KALMAN = 0.9; // prior error covariance
double X_KALMAN = 2.5; // prior estimate of true value
double R_KALMAN = 0.55; // standard deviation
double K_KALMAN = 0; // Kalman gain

// variables for load cell
#define LEDpin 42//type pin number here
HX711 scaleLeft(47,45); //initialize Left load cell 
HX711 scaleRight(48,46);//initialize Right load cell
long valLeft = 0;
long offset1 = 730000; //offset for Left load cell
long valRight = 0;
long offset2 = 523000; //offset for Right load cell
long ave, threshold = 136000;
float LOADcount = 0;
long runave = 0;

//Declare encoders
Encoder encLadder(P_LADDER_ENCA, P_LADDER_ENCB);

Timer timer1;

int cyclesSinceTimeout = 0;

//This is called whenever the arduino recieves data from the robot-laptop
String inputString = "";
void serialEvent() {
   while (Serial.available()) {
      char inChar = (char)Serial.read();
      inputString += inChar;
      if (inChar == '?') {
         ProcessInput(inputString);
         inputString = "";
      }
   }
}

void ProcessInput(String inputString) {                                                                 //remoteinput
   cyclesSinceTimeout = 0;
   
   int colIndex1 = inputString.indexOf("|");
   int endIndex = inputString.indexOf("?");

   String commandType = inputString.substring(0, colIndex1);
   String commandData = inputString.substring(colIndex1 + 1, endIndex);

   if (commandType == "GET_STATUS") {
      Serial.println(getEncoderValues());
      Serial.println("?");
   }
   else if (commandType == "UPDATE_MOTORS") {
      int colIndex1 = commandData.indexOf(':');
      int colIndex2 = commandData.indexOf(':', colIndex1 + 1);
      int colIndex3 = commandData.indexOf(':', colIndex2 + 1);
      int colIndex4 = commandData.indexOf(':', colIndex3 + 1);
      int endIndex = commandData.indexOf('?');
      //There is miss-comunication if there is not 3 delimiters (indexOf returns -1 if not found)
      if (colIndex1 == -1 || colIndex2 == -1 || colIndex3 == -1 || colIndex4 == -1) {
          //Serial.println("Invalid input string: ");
          //Serial.println(inputString);
          Serial.println("?");
          return;
      }

      //Split the string into separate variables
      String _sLadder = commandData.substring(0, colIndex1);
      String _sScoops = commandData.substring(colIndex1 + 1, colIndex2);
      String _sDump = commandData.substring(colIndex2 + 1, colIndex3);
      String _AutoMine = commandData.substring(colIndex3 + 1, colIndex4);
      String _EStop = commandData.substring(colIndex4 +1, endIndex);

      //Parse the strings into ints and floats
      m_iLadderWantedPWM = (int)(_sLadder.toFloat() * C_LADDER_SPEED_MULT);
      m_iScoopsWantedPWM = (int)(_sScoops.toFloat()) * C_SCOOP_SPEED;
      m_iDumpWantedPWM = _sDump.toInt() * C_DUMP_SPEED;    //val of 0 or 1 right now
      //m_AutoMine = _AutoMine.toInt();
      if(_AutoMine.toInt()){m_AutoMine = 1;}
      if(_EStop.toInt()){m_EStop = 1;}
   }
   else if (commandType == "STOP") {             //B
      StopMotors();
      m_bIsLadderOverride = false;
   }
   else if (commandType == "DEPLOY") {
	   //DEPLOY_MINING_SYSTEM();
   }
   else if (commandType == "TARE_PRESSURE") {
      //scale.tare();
   }
   else if (commandType == "READ_PRESSURE") {
      //Serial.print(scale.get_units(), 1);
      Serial.println("?");
   }
   else if (commandType == "PENDING") {

   }
   else if (commandType == "WHOAMI") {
       Serial.println("ARD2__MINE_SYSTEM");
   }
   else if (commandType == "LADOVR") {
       int endIndex = commandData.indexOf('?');
       latCntctSwOverride = commandData.substring(0, endIndex).toInt();//
   }

}

//"l|s|ladderDownDistance"
String getEncoderValues() {
   int l = encLadder.read();

   String returnString = "";
   returnString += l;

   return returnString;
}

//This takes in the string from the robot-laptop and processes that data
void UpdateMotorValues() {                                                              //update motors
   if(m_EStop){
      //calibrated = false;
      //DECENTcomplete = false;
      //BOTTOMreached = false;
      //wait = false;
      //start_up_complete = 0;
      m_AutoMine = 0;
      m_EStop = 0;
      //start_up_complete = 0;
   }
   if(m_AutoMine){
      digitalWrite(P_SCOOPS_DIR, LOW);

   // begin the autonomous mining

   
  // create boolean variables
     if (LADDERcount >= BOTTOMarmLIMIT || digitalRead(HALLpinRIGHT) == LOW)
     {
       BOTTOMreached = true;
     }
  
     boolean TOPreached = (LADDERcount <= TOParmLIMIT && DECENTcomplete == true || digitalRead(HALLpinLEFT) == LOW && DECENTcomplete == true);
     
     double gap  = abs(SET_SCOOP_CURRENT-FILTERED_SCOOP_CURRENT);
     boolean too_far = (gap >= 0.1);
     
       if(calibrated == false)
       {
         LADDERcalibration();
       }
       else if (calibrated == true)
       {      

          // read the current draw from the scoops motor  
          CURRENTread();

          // determine which PID constants to use
          if(too_far)
         {
            //aggresive constants
            // these constants bring the scoops up to speed
            BELTmotorPID.SetTunings(aggKp, aggKi, aggKd);
         }     
         else
         {
            //conservative constants
            // these constants control the speed during the mining proces
            BELTmotorPID.SetTunings(consKp, consKi, consKd);
         }
         // compute the output value to the arm motor
          BELTmotorPID.Compute();
    
          //write to the belt pin
          analogWrite(P_SCOOPS_PWM,SCOOP_SPEED_AUTO);
          
          // this determines if the scoops are spinning fast enough to begin decent of the ladder
          if (SCOOP_SPEED_AUTO>200)
          {
            start_up_complete = 1;
          }
    
         // begin the decent of the mining ladder once the belt has reached max speed
         if(start_up_complete == 1)
         {
           // change the direction of the ladder to decending
           digitalWrite(P_LADDER_DIR, HIGH);
           LADDER_SPEED_AUTO = 14; //guess value
           analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);     
         }
   
         // monitor if the ladder range limit is exceeded (decending)
         if(BOTTOMreached)
         {
           // stop the mining ladder
           LADDER_SPEED_AUTO = 0;
           analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);
        
           //let the belt finish mining
           //only use the delay function once
           if(wait == false)
           {
             delay(4000);
             wait = true;
           }
           //change the direction of the mining ladder
           digitalWrite(P_LADDER_DIR, LOW);
        
           //indicate that the decent is completed
           DECENTcomplete = true;
        
           //Move the sensor away from the magnet
           LADDER_SPEED_AUTO = 100; //guess value
           analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);
         }
    
      // monitor if the arm range limit is exceeded (accending)
        if(TOPreached)
        {
         // stop the mining arm motor
         LADDER_SPEED_AUTO = 0;
         analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);
         //stop the mining belt motor
         SCOOP_SPEED_AUTO = 0;
         analogWrite(P_SCOOPS_PWM,SCOOP_SPEED_AUTO);
      
         // signal that the mining is complete
         x = 2;
         calibrated = false;
         DECENTcomplete = false;
         BOTTOMreached = false;
         wait = false;
         start_up_complete = 0;
         m_AutoMine = 0;
         // reset boolean variables
        }
      }   
   }else{
        cyclesSinceTimeout++;

    if (cyclesSinceTimeout > 8) {
        setPower(0, P_DUMP_PWM);
        setPower(0, P_LADDER_PWM);
        setPower(0, P_SCOOPS_PWM);
        m_iScoopsWantedPWM = 0;
        m_iDumpWantedPWM = 0;
        m_iLadderWantedPWM = 0;
        return;
    }

    int _iLadderSetPWM, _iScoopsSetPWM, _iDumpSetPWM;
    _iLadderSetPWM = _iScoopsSetPWM = _iDumpSetPWM = 0;
    
      CorrectPWMVals(&m_iLadderWantedPWM, 0, &m_iLadderLastPWM, &_iLadderSetPWM, MAX_LADDER_ACCEL_PWM);
      CorrectPWMVals(&m_iScoopsWantedPWM, 0, &m_iScoopsLastPWM, &_iScoopsSetPWM, MAX_MOTOR_ACCEL_PWM);
      CorrectPWMVals(&m_iDumpWantedPWM,   0, &m_iDumpLastPWM,   &_iDumpSetPWM, MAX_MOTOR_ACCEL_PWM);

   //if (!latCntctSwOverride) {
       latbot = digitalRead(P_LADDERBOT_INT);
       lattop = digitalRead(P_LADDERTOP_INT);
       if (latbot == LOW && _iLadderSetPWM < 0) //do not go further down
           _iLadderSetPWM = 0;
       if (lattop == LOW && _iLadderSetPWM > 0) //do not go further up
           _iLadderSetPWM = 0;
   //}
   
      _iLadderSetPWM = contain(_iLadderSetPWM, -C_LADDER_SPEED, C_LADDER_SPEED); //See contain(bottom) for description
      setDirection(_iLadderSetPWM, P_LADDER_DIR);
      setPower(_iLadderSetPWM, P_LADDER_PWM);

      _iDumpSetPWM = contain(_iDumpSetPWM, -C_DUMP_SPEED, C_DUMP_SPEED);
      setPower(_iDumpSetPWM, P_DUMP_PWM);

      _iScoopsSetPWM = contain(_iScoopsSetPWM, -C_SCOOP_SPEED, C_SCOOP_SPEED);
      setPower(_iScoopsSetPWM, P_SCOOPS_PWM);
   }
}

//Here we make sure the motors don't (de)accelerate too quickly
//Params: Wanted, Actual, LastActual, ValueWe'llGiveToTheMotor
//Actual speed is not working currently as Encoders aren't wired properly
void CorrectPWMVals(int *p_fWantedRPM, int *p_fActRPM, int *p_fLastSetRPM, int *p_fSetRPM, int multiplier) {
    float diff = (*p_fWantedRPM - *p_fLastSetRPM);

    if (abs(diff) > multiplier) {//accelerating or decelerating rapidly
                                          //am I decelerating or accelerating?
        if (diff >= 0) {//accelerating forward
            *p_fSetRPM = *p_fLastSetRPM + multiplier;
            //if (*p_fSetRPM < 0) *p_fSetRPM = 0.0f;
        }
        else {//accel in backward direction
                *p_fSetRPM = *p_fLastSetRPM - multiplier;
            //if (*p_fSetRPM > 0) *p_fSetRPM = 0.0f;
        }
    }

    else {
        *p_fSetRPM = *p_fLastSetRPM + diff;
    }
    *p_fLastSetRPM = *p_fSetRPM;
}

void auto_mine()
{
  
}

//This is automatically called ONCE when the arduino starts or is reset
void setup() {
   Serial.begin(9600);
   Serial.println("test");

   pinMode(P_LADDER_DIR, OUTPUT);
   pinMode(P_SCOOPS_DIR, OUTPUT);
   pinMode(P_DUMP_DIR, OUTPUT);
   pinMode(P_LADDERTOP_INT, INPUT);
   pinMode(P_LADDERBOT_INT, INPUT);
   pinMode(P_HOPPER_PB1, INPUT);
   pinMode(P_HOPPER_PB4, INPUT);
   pinMode(P_HOPPER_PB2_WAKE, OUTPUT);
   //initialize pin types
   // scoop motor
   pinMode(P_SCOOPS_PWM, OUTPUT);
   pinMode(P_SCOOPS_CURRENT, INPUT);

   // ladder motor
   pinMode(P_LADDER_PWM, OUTPUT);
   // encoder pins
   pinMode(P_LADDER_ENCODER_A, INPUT);
   pinMode(P_LADDER_ENCODER_B, INPUT);
   attachInterrupt(1, LadderEncoderEvent, CHANGE);
   // PID setup
  BELTmotorPID.SetMode(AUTOMATIC); // turns the PID controller on
  BELTmotorPID.SetOutputLimits(0,254); //sets the range the controller can set the output variable
   // hall sensors
   pinMode(HALLpinLEFT, INPUT);
   pinMode(HALLpinRIGHT, INPUT);
   digitalWrite(P_DUMP_DIR, HIGH);
   
   StopMotors();
   timer1.every(POLL_INTERVAL, UpdateMotorValues);

}

void StopMotors() {
    m_iDumpWantedPWM = 0;
    m_iLadderWantedPWM = 0;
    m_iScoopsWantedPWM = 0;
   m_bLadderReset = false;
   m_bIsLadderOverride = false;
}

//This is continually looping
void loop() {                                                                                                  //loop
   timer1.update();
   
   LOADcheck();
   
   //USED FOR IN PIT TESTING 5/8/18
   Serial.print("LADDERcount: ");
   Serial.print(LADDERcount);
   Serial.print("   ");
   Serial.print("Loadcell runave: ");
   Serial.print(runave);
   Serial.print("\n");

}
///These are just helper functions

//return modified val that is contained within min and max
float contain(float val, float p_fMin, float p_fMax) {
   return (val < p_fMin) ? p_fMin : (val > p_fMax) ? p_fMax : val;
}

//return modified val that is contained within min and max
int contain(int val, int p_fMin, int p_fMax) {
   return (val < p_fMin) ? p_fMin : (val > p_fMax) ? p_fMax : val;
}

void setDirection(int val, int pin) {
   digitalWrite(pin, (val > 0) ? LOW : HIGH);
}

void setDirection(float val, int pin) {
   digitalWrite(pin, (val > 0) ? LOW : HIGH);
}

void setPower(float val, int pin) {
   analogWrite(pin, (int)abs(val));
}

void setPower(int val, int pin) {
   analogWrite(pin, abs(val));
}

double distance() {
   double lat = encLadder.read();
   double x = (lat / 5787.8)*7.9797;
   double dis = x*sin(60) - 3.53;
   return dis;
}

// calibrate the encoder on the arm before mining
void LADDERcalibration()
{
   // set the direction pin for the ladder (acending for zeroing of encoder)
   digitalWrite(P_LADDER_DIR, LOW);
   
   // move the ladder to the top position
   LADDER_SPEED_AUTO = 100;
   analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);
   
   if(digitalRead(HALLpinLEFT) == LOW)
   {
    LADDERcount = 0;
    LADDER_SPEED_AUTO = 0;
    analogWrite(P_LADDER_PWM,LADDER_SPEED_AUTO);

    // signal that the calibration is complete
    calibrated  = true; //switch the value to true
   }
}

// current reading and filtering
void CURRENTread()
{
   //calculate the current
   RAWsenseLADDER = analogRead(P_SCOOPS_CURRENT);
   voltage = ((RAWsenseLADDER*5)/1023)*1000; //millivolts
   current = (ACSoffset-voltage)/mVperAmp; //Amps
        
   // filter the current
   // measurement update (correction)
   K_KALMAN = P_KALMAN/(P_KALMAN+R_KALMAN);
   X_KALMAN = X_KALMAN + K_KALMAN *(current-X_KALMAN);
   P_KALMAN = (1-K_KALMAN)*P_KALMAN;
   // assign the predicted true value
   FILTERED_SCOOP_CURRENT = X_KALMAN; 
}

// encoder event for the interrupt call
void LadderEncoderEvent()
{
  if (digitalRead(P_LADDER_ENCODER_A) == HIGH) {
    if (digitalRead(P_LADDER_ENCODER_B) == LOW) {
      LADDERcount++;
    } else {
      LADDERcount--;
    }
  } else {
    if (digitalRead(P_LADDER_ENCODER_B) == LOW) {
      LADDERcount--;
    } else {
      LADDERcount++;
    }
  }
}

void LOADcheck()
{
  valLeft= scaleLeft.read()-offset1;
  valRight=scaleRight.read()-offset2;
  ave=(valLeft+valRight)/2;
 
   runave = 0.5 * runave    +   0.5 * ave; //short term running average
   
  if (runave > threshold)
  {
    LOADcount  = LOADcount+1;
  }
  else
  {
    LOADcount = 0;
  }

  if(LOADcount > 30 )
  {
    digitalWrite(LEDpin, HIGH);
   
  }
    else
    {
      digitalWrite(LEDpin,LOW);
    }
}
