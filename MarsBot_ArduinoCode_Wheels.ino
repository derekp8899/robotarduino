#include <Timer.h>
#include <Encoder.h>
#include <Servo.h>

#include "constants.h"
#include "helperFunctions.h"

//Declare encoders
Encoder encFR(P_FR_ENCA, P_FR_ENCB);
Encoder encFL(P_FL_ENCA, P_FL_ENCB);
Encoder encBR(P_BR_ENCA, P_BR_ENCB);
Encoder encBL(P_BL_ENCA, P_BL_ENCB);

Servo panServo, tiltServo;

Timer timer1;

float m_fWantedRPM_FL = 0;
float m_fWantedRPM_FR = 0;
float m_fWantedRPM_BL = 0;
float m_fWantedRPM_BR = 0;
float m_fLastEncCER_FL = 0; //CER: Current Encoder Rotation
float m_fLastEncCER_FR = 0;
float m_fLastEncCER_BL = 0;
float m_fLastEncCER_BR = 0;
float m_fLastSetRPM_FL = 0;
float m_fLastSetRPM_FR = 0;
float m_fLastSetRPM_BL = 0;
float m_fLastSetRPM_BR = 0;
bool m_bSlip_FL = false;
bool m_bSlip_FR = false;
bool m_bSlip_BL = false;
bool m_bSlip_BR = false;
bool m_bRobotIsRunning = false;
bool commandsAreBeingSent = false;

int cyclesSinceTimeout = 0;

//This is called whenever the arduino recieves data from the robot-laptop
String inputString = "";
void serialEvent() {
    while(Serial.available()) {
      	char inChar = (char) Serial.read();
        inputString += inChar;
        if(inChar == '?') {
            ProcessInput(inputString);
            inputString = "";
            commandsAreBeingSent = true;
        }
    }
}

void ProcessInput(String inputString) {
    cyclesSinceTimeout = 0;

    int colIndex1 = inputString.indexOf("|");
    int endIndex = inputString.indexOf("?");

    String commandType = inputString.substring(0, colIndex1);
    String commandData = inputString.substring(colIndex1 + 1, endIndex);

    if(commandType == "GET_STATUS") {
        String wStatus = getWheelStatus();
        Serial.print(wStatus);
        Serial.print("?");
    }
    else if(commandType == "UPDATE_MOTORS") {
        UpdateMotorValues(commandData);
    }
    else if(commandType == "UPDATE_SERVOS") {
        UpdateServos(commandData);
    }
    else if (commandType == "PENDING") {

    }
    else if (commandType == "WHOAMI") {
        Serial.println("ARD1__DRIVE_SYSTEM");
    }

}

//"FL|FR|BL|BR|FLS|FRS|BLS|BRS"
//Slip: #==0(just right) #!=0(assume slipping)
String getWheelStatus() {
  String returnString = "";
  returnString += getEncoderValues();
  returnString += "|";
  //Attach slip values here...
  returnString += (m_bSlip_FL ? 1 : 0);
  returnString += "|";
  returnString += (m_bSlip_FR ? 1 : 0);
  returnString += "|";
  returnString += (m_bSlip_BL ? 1 : 0);
  returnString += "|";
  returnString += (m_bSlip_BR ? 1 : 0);

  return returnString;
}

//"FL|FR|BL|BR"
String getEncoderValues() {
  float fr = encFR.read();
  float fl = encFL.read();
  float br = encBR.read();
  float bl = encBL.read();
  String returnString = "";
  returnString += fr;
  returnString += "|";
  returnString += fl;
  returnString += "|";
  returnString += br;
  returnString += "|";
  returnString += bl;

  return returnString;
}

//This is called by timer1 and is for maintaining wheel direction and speed
void UpdateMotorsToValues() {
    cyclesSinceTimeout++;

    if (cyclesSinceTimeout > 8) {
        setPower(0.0f, P_FL_PWM);
        setPower(0.0f, P_FR_PWM);
        setPower(0.0f, P_BL_PWM);
        setPower(0.0f, P_BR_PWM);
        m_fWantedRPM_BL = 0;
        m_fWantedRPM_BR = 0;
        m_fWantedRPM_FL = 0;
        m_fWantedRPM_FR = 0;
        return;
    }

    float _fEnc_FL = encFL.read();//Encoder read values
    float _fEnc_FR = encFR.read();
    float _fEnc_BL = encBL.read();
    float _fEnc_BR = encBR.read();

    //After one rotation of the wheel, the encoder will read ~5787.8
    float _fEncCER_FL = _fEnc_FL / ENCODER_1ROT_VALUE;
    float _fEncCER_FR = _fEnc_FR / ENCODER_1ROT_VALUE;
    float _fEncCER_BL = _fEnc_BL / ENCODER_1ROT_VALUE;
    float _fEncCER_BR = _fEnc_BR / ENCODER_1ROT_VALUE;


    float _fActRPM_FL = 0;//actual RPMs
    float _fActRPM_FR = 0;
    float _fActRPM_BL = 0;
    float _fActRPM_BR = 0;
    float _fSetRPM_FL = 0;//values to set RPM at
    float _fSetRPM_FR = 0;
    float _fSetRPM_BL = 0;
    float _fSetRPM_BR = 0;
    
    //get measured rpm values
    UpdateRPMVals(&_fActRPM_FL, &_fEncCER_FL, &m_fLastEncCER_FL);
    UpdateRPMVals(&_fActRPM_FR, &_fEncCER_FR, &m_fLastEncCER_FR);
    UpdateRPMVals(&_fActRPM_BL, &_fEncCER_BL, &m_fLastEncCER_BL);
    UpdateRPMVals(&_fActRPM_BR, &_fEncCER_BR, &m_fLastEncCER_BR);
    

    //if actual is too far from set then set slippage flag
    //DetectSlippage(&m_bSlip_FL, _fActRPM_FL, _fSetRPM_FL);
    //DetectSlippage(&m_bSlip_FR, _fActRPM_FR, _fSetRPM_FR);
    //DetectSlippage(&m_bSlip_BL, _fActRPM_BL, _fSetRPM_BL);
    //DetectSlippage(&m_bSlip_BR, _fActRPM_BR, _fSetRPM_BR);

    //correct for smooth acceleration
    CorrectRPMVals(&m_fWantedRPM_FL, &_fActRPM_FL, &m_fLastSetRPM_FL, &_fSetRPM_FL);
    CorrectRPMVals(&m_fWantedRPM_FR, &_fActRPM_FR, &m_fLastSetRPM_FR, &_fSetRPM_FR);
    CorrectRPMVals(&m_fWantedRPM_BL, &_fActRPM_BL, &m_fLastSetRPM_BL, &_fSetRPM_BL);
    CorrectRPMVals(&m_fWantedRPM_BR, &_fActRPM_BR, &m_fLastSetRPM_BR, &_fSetRPM_BR);
        
    //Correction for a slow motor in absence of encoder function
    _fSetRPM_FL *= 1.2;

    setDirection(_fSetRPM_FL, P_FL_DIR);
    setDirection(_fSetRPM_FR, P_FR_DIR);
    setDirection(_fSetRPM_BL, P_BL_DIR);
    setDirection(_fSetRPM_BR, P_BR_DIR);
    
    if (commandsAreBeingSent) {
       setPower(_fSetRPM_FL, P_FL_PWM);
       setPower(_fSetRPM_FR, P_FR_PWM);
       setPower(_fSetRPM_BL, P_BL_PWM);
       setPower(_fSetRPM_BR, P_BR_PWM);
    }
    else {
       setPower(0.0f, P_FL_PWM);
       setPower(0.0f, P_FR_PWM);
       setPower(0.0f, P_BL_PWM);
       setPower(0.0f, P_BR_PWM);
    }

    if(isZero(_fSetRPM_FL) && isZero(_fSetRPM_BL) && isZero(_fSetRPM_BR) && isZero(_fSetRPM_FR))
      digitalWrite(P_DRIVE_BUFFER, HIGH);
    else
      digitalWrite(P_DRIVE_BUFFER, LOW);
}


//Params: Actual, ReadFromEncoder, LastActual
void UpdateRPMVals(float *p_fActRPM, float *p_fEncCER, float *p_fLastEncCER) {
    //Remember that this function is called every 100ms
    //100ms * 10 = 1sec * 60sec = 1min * changeInRotation = RPM
    *p_fActRPM = (*p_fEncCER - *p_fLastEncCER) * 60 * POLL_RATE;
    *p_fLastEncCER = *p_fEncCER;
}

void DetectSlippage(bool *p_bFlag, float p_fActRPM, float p_fSetRPM) {
    *p_bFlag = (abs(p_fActRPM - p_fSetRPM) > SLIPPAGE_RANGE);
}

//Here we make sure the wheels don't (de)accelerate too quickly
//Params: Wanted, Actual, LastActual, ValueWe'llGiveToTheMotor
void CorrectRPMVals(float *p_fWantedRPM, float *p_fActRPM, float *p_fLastSetRPM, float *p_fSetRPM) {
   float diff = (*p_fWantedRPM - *p_fLastSetRPM);

   //Serial.print("Act:"); Serial.print(*p_fActRPM); Serial.print(" Want:"); Serial.print(*p_fWantedRPM);

   if (abs(diff) > MAX_WHEEL_ACCEL_RPM) {//accelerating or decelerating rapidly
      //am I decelerating or accelerating?
       if (diff >= 0) {//accelerating forward
           if (*p_fLastSetRPM < 0) {
               //use faster decel
               *p_fSetRPM = *p_fLastSetRPM + MAX_WHEEL_DECEL_RPM;
               if (*p_fSetRPM > 0) *p_fSetRPM = 0.0f;
           }
           else
                *p_fSetRPM = *p_fLastSetRPM + MAX_WHEEL_ACCEL_RPM;
           //if (*p_fSetRPM < 0) *p_fSetRPM = 0.0f;
       }
       else {//accel in backward direction
           if (*p_fLastSetRPM > 0) {
               *p_fSetRPM = *p_fLastSetRPM - MAX_WHEEL_DECEL_RPM;
               if (*p_fSetRPM < 0) *p_fSetRPM = 0.0f;
           }
           else
               *p_fSetRPM = *p_fLastSetRPM - MAX_WHEEL_ACCEL_RPM;
           //if (*p_fSetRPM > 0) *p_fSetRPM = 0.0f;
       }
   }

   else {
      *p_fSetRPM = *p_fLastSetRPM + diff;
   }
   *p_fLastSetRPM = *p_fSetRPM;

   //I don't think encoders are working TODO
    //float error = (*p_fWantedRPM - *p_fActRPM);

    ////If the requested speed is greater then the max allowed (de)acceleration
    //if(error > MAX_WHEEL_ACCEL)
    //    *p_fSetRPM = error / abs(error) * MAX_WHEEL_ACCEL + *p_fLastSetRPM;
    //else
    //    *p_fSetRPM = error * HIGH_GAIN + *p_fLastSetRPM;
    //*p_fSetRPM = contain(*p_fSetRPM, -255.0, 255.0);
    //if (abs(*p_fSetRPM) < 20.0)
    //    *p_fSetRPM = 0;
    //*p_fLastSetRPM = *p_fSetRPM;

   //Serial.print(" set:"); Serial.println(*p_fSetRPM);

}

void UpdateServos(String inputString) {
  int colIndex1 = inputString.indexOf(':');
  int endIndex = inputString.indexOf('?');
  if(colIndex1 == -1) {
    Serial.print("Invalid input string: ");
    Serial.print(inputString);
    Serial.print("?");
    return;
  }
  String _sPan = inputString.substring(0, colIndex1);
  String _sTilt = inputString.substring(colIndex1 + 1, endIndex);
  int _iPan = _sPan.toInt();
  int _iTilt = _sTilt.toInt();

  _iPan = contain(_iPan, 0, 180);
  _iTilt = contain(_iTilt, 0, 180);

  panServo.write(_iPan);
  tiltServo.write(_iTilt);
}

//This takes in the string from the robot-laptop and processes that data
void UpdateMotorValues(String inputString) {
    int colIndex1 = inputString.indexOf(':');
    int colIndex2 = inputString.indexOf(':', colIndex1 + 1);
    int colIndex3 = inputString.indexOf(':', colIndex2 + 1);
    int endIndex = inputString.indexOf('?');
    //There is miss-comunication if there is not 7 delimiters (indexOf returns -1 if not found)
    if(colIndex1 == -1 || colIndex2 == -1 || colIndex3 == -1) {
       Serial.print("Invalid input string: ");
       Serial.print(inputString);
       Serial.print("?");
       return;
  	 }

    //Split the string into separate variables
    String _sLeftVel = inputString.substring(0, colIndex1);
    String _sRightVel = inputString.substring(colIndex1 + 1, colIndex2);
  
    //Parse the strings into ints and floats
    float _fLeftVel = _sLeftVel.toFloat() / WHEEL_SPEED_MULTIPLIER; // to clamp -1 <= _fLeftVel <= 1
    float _fRightVel = _sRightVel.toFloat() / WHEEL_SPEED_MULTIPLIER;
  
    // (m/s) * multiplier * (60s/1min) * (multiplier) * (1/m) -> multiplier/min
    float _fLeftSide = (MAX_SPEED * _fLeftVel * (60 * POLL_RATE)) / WHEEL_CIRCUM; //RPM calc
    float _fRightSide = (MAX_SPEED * _fRightVel * (60 * POLL_RATE)) / WHEEL_CIRCUM; //RPM calc

    m_fWantedRPM_FL = -(_fLeftSide);
    m_fWantedRPM_BL = -(_fLeftSide);
    m_fWantedRPM_FR = _fRightSide;
    m_fWantedRPM_BR = _fRightSide;
  	
    //The direction and power of the wheels will be set in the timer function (see setup())
}

//This is automatically called ONCE when the arduino starts or is reset
void setup() {
    Serial.begin(9600);
  	 pinMode(P_FR_DIR, OUTPUT);
  	 pinMode(P_FL_DIR, OUTPUT);
  	 pinMode(P_BR_DIR, OUTPUT);
  	 pinMode(P_BL_DIR, OUTPUT);
    pinMode(P_DRIVE_BUFFER, OUTPUT);

    panServo.attach(P_SERVO_PAN);
    tiltServo.attach(P_SERVO_TILT);

    //IF YOU CHANGE THIS INTERVAL, be sure to update UpdateRPMVals() accordingly
    timer1.every(POLL_INTERVAL, UpdateMotorsToValues);
}

//This is continually looping after setup() is called
void loop() {
    timer1.update();
}
