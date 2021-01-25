#include "Rotary.h"
#include <TimerOne.h>

#define ratio 1/26                             // Ratio gear_box`
#define PI 3.1415926535897932384626433832795
#define Time_Interval 70000.0                // 70000 us                             


Rotary rotary = Rotary(2, 3);                    // pin definition
int ST = (60000000/Time_Interval);               //(60/Time Interval(s))
float RPP = 0.05;                               //Revolution per Pulses 
unsigned int counter= 0;                               // count Pulses per Revolution  
volatile byte flag = 0;
float rpm;  
float T = 0;
void setup() 
{
  Serial.begin(115200);
  Timer1.initialize(Time_Interval);
  Timer1.attachInterrupt(timerOneIsr);
  attachInterrupt(0, rotate, CHANGE);
  attachInterrupt(1, rotate, CHANGE);
}

void loop()
{ 
   Speed() ;
}

void Speed() 
{ 
  if (flag == true)
  { 
    rpm = (counter*ST*ratio*RPP);  // Speed of Motor (rpm)
    Serial.println(rpm);    
    counter = 0 ;
    flag = false ;

  } 
}    

void rotate() 
{
  unsigned char result = rotary.process();
  if (result == DIR_CW) 
  {
    counter++;
  } 
  else if (result == DIR_CCW) 
  {
    counter++; 
  }
}

void timerOneIsr()
{
  flag = true; 
}
