#include "constants.h"

//Endcoder list
/*
P_FR_ENCA P_FR_ENCB
P_FL_ENCA P_FL_ENCB
P_BR_ENCA P_BR_ENCB
P_BL_ENCA P_BL_ENCB
*/
// Encoder counts
volatile double FR_ENC_count = 0;
volatile double BR_ENC_count = 0;
volatile double FL_ENC_count = 0;
volatile double BL_ENC_count = 0;

// Wheel rotational values
volatile double FR_rot_current = 0;
volatile double BR_rot_current = 0;
volatile double FL_rot_current = 0;
volatile double BL_rot_current = 0;

double FR_rot_last = 0;
volatile double BR_rot_last = 0;
volatile double FL_rot_last = 0;
volatile double BL_rot_last = 0;

double FR_rot_diff = 0;
double BR_rot_diff = 0;
double FL_rot_diff = 0;
double BL_rot_diff = 0;

long FR_vel = 0;
double BR_vel = 0;
double FL_vel = 0;
double BL_vel = 0;

// Times for encoders
unsigned long enc_time_last;
unsigned long enc_time_current;
unsigned long check_time;
unsigned long check_period = 1000; //set check period time
 
unsigned long FR_time_diff = 0;
long BR_time_diff = 0;
long FL_time_diff = 0;
long BL_time_diff = 0;

void setup()
{
Serial.begin(9600);

enc_time_last = millis();
//initiatiing interrupts
attachInterrupt(digitalPinToInterrupt(P_FR_ENCA),front_right_wheel,CHANGE);
attachInterrupt(digitalPinToInterrupt(P_FL_ENCA),front_left_wheel,CHANGE);
attachInterrupt(digitalPinToInterrupt(P_BR_ENCA),back_right_wheel,CHANGE);
attachInterrupt(digitalPinToInterrupt(P_BL_ENCA),back_left_wheel,CHANGE);

}

void loop() 
{ 
  //check_time = millis;
  /*
  if (checktime>period)
  {
    if(FR_vel =
    //reset all velocity values
  }
  */
  
  if ( FR_rot_last != FR_rot_current)
  {
    //sample the time for this event
    enc_time_current = millis();
    //find the difference in the counts
    FR_rot_diff = (FR_rot_current - FR_rot_last);
    //find the difference in time
    FR_time_diff = (enc_time_current-enc_time_last);//miliseconds    
    //find the velocity
    // /ENCODER_1ROT_VALUE*1000;
    FR_vel = double(FR_rot_diff)/double(FR_time_diff); // in rotations per seconds

    //print for check
    Serial.print("Rot current: ");
    Serial.print(FR_rot_current);
    Serial.print("   ");
    Serial.print("Rot last: ");
    Serial.print(FR_rot_last);
    Serial.print("   ");
    Serial.print("FR_rot_diff: ");
    Serial.print(FR_rot_diff);
    Serial.print("   ");
    Serial.print("Front Right Velocity: ");
    Serial.print(FL_vel,6);
    Serial.print("FR Time diff    ");
    Serial.print(FR_time_diff);
    //reset the time value
    enc_time_last = enc_time_current;
    //reset the rotational value
    FR_rot_last = FR_rot_current;
    
  }


/*
Serial.print("   ");
Serial.print("Back Right: ");
Serial.print(BR_rot);

Serial.print("   ");
Serial.print("Back Left: ");
Serial.print(BL_rot);

Serial.print("   ");
*/

Serial.print("\n");
 
}

void front_right_wheel()
{
  //front right motor
  if (digitalRead(P_FR_ENCA) == HIGH) {
      if (digitalRead(P_FR_ENCB) == LOW) {
        FR_ENC_count++;
      } else {
        FR_ENC_count--;
      }
    } else {
      if (digitalRead(P_FR_ENCB) == LOW) {
        FR_ENC_count--;
      } else {
        FR_ENC_count++;
      }
      
    }
          
          FR_rot_current = FR_ENC_count;
}
void back_right_wheel()
{
  //back right motor
    if (digitalRead(P_BR_ENCA) == HIGH) {
      if (digitalRead(P_BR_ENCB) == LOW) {
        BR_ENC_count++;
      } else {
        BR_ENC_count--;
      }
    } else {
      if (digitalRead(P_BR_ENCB) == LOW) {
        BR_ENC_count--;
      } else {
        BR_ENC_count++;
      }
    }
          //convert to revolutions
      BR_rot_current = BR_ENC_count/ENCODER_1ROT_VALUE;  
}

void front_left_wheel()
{
  //front left motor
  if (digitalRead(P_FL_ENCA) == HIGH) {
      if (digitalRead(P_FL_ENCB) == LOW) {
        FL_ENC_count++;
      } else {
        FL_ENC_count--;
      }
    } else {
      if (digitalRead(P_FL_ENCB) == LOW) {
        FL_ENC_count--;
      } else {
        FL_ENC_count++;
      }
    }
              //convert to revolutions
          FL_rot_current = FL_ENC_count/ENCODER_1ROT_VALUE;
}
void back_left_wheel()
{
  //back left motor
    if (digitalRead(P_BL_ENCA) == HIGH) {
      if (digitalRead(P_BL_ENCB) == LOW) {
        BL_ENC_count++;
      } else {
        BL_ENC_count--;
      }
    } else {
      if (digitalRead(P_BL_ENCB) == LOW) {
        BL_ENC_count--;
      } else {
        BL_ENC_count++;
      }
    }
              //convert to revolutions
      BL_rot_current = BL_ENC_count/ENCODER_1ROT_VALUE;   
}


