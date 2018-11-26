#include <Encoder.h>
#include "constants.h"


//Declare encoders
Encoder encFR(P_FR_ENCA, P_FR_ENCB);
Encoder encFL(P_FL_ENCA, P_FL_ENCB);
Encoder encBR(P_BR_ENCA, P_BR_ENCB);
Encoder encBL(P_BL_ENCA, P_BL_ENCB);

float fr;
float fl;
float br;
float bl;

void setup()
{
Serial.begin(9600);

}


void loop() 
{
  /*
  fr = encFR.read();
  fl = encFL.read();
  br = encBR.read();
  bl = encBL.read();

  
Serial.print("Front Right: ");
Serial.print(fr);
Serial.print("   ");
Serial.print("Front Left: ");
Serial.print(fl);
Serial.print("   ");
Serial.print("Back Right: ");
Serial.print(br);
Serial.print("   ");
Serial.print("Back Left: ");
Serial.print(bl);
Serial.print("   ");
Serial.print("\n");
*/

  EncoderRead(P_FR_ENCA, P_FR_ENCB);
}

void  EncoderRead(int pinA, int pinB)
{
  int amount = 0;
  if (digitalRead(pinA) == HIGH && digitalRead(pinB) == LOW)
  {
    amount++;
  }
  else if (digitalRead(pinA) == LOW && digitalRead(pinB) == LOW)
  {
    amount--;
  }

  Serial.println(amount);
    
}




