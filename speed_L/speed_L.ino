#include <ros.h>
#include <std_msgs/Float32.h>
#include "Rotary.h"
#include <TimerOne.h>

Rotary rotary = Rotary(2, 3);                    // pin definition
#define ratio 1/26                             // Ratio gear_box`
#define PI 3.1415926535897932384626433832795
#define Time_Interval 100000.0                // 100 ms
#define ST  600                              //(60/Time Interval(s)) 
#define RPM_TO_RPS (2*PI)/60 

//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
std_msgs::Float32 msg;
ros::Publisher pub("speed_L", &msg);  //topic

float PPR = 0.05;                               //Pulses per Revolution
long counter= 0;                               // count Pulses per Revolution  
volatile byte flag = 0;
float rpm = 0;
float T = 0;

void setup() 
{
  nh.initNode();
  nh.advertise(pub);
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
    rpm = (counter*ST*ratio*PPR); // Speed of Motor (rpm)
    msg.data = rpm ;
    pub.publish(&msg);
    counter = 0 ;
    flag = false ;
  } 
   nh.spinOnce();
}    

void rotate() 
// rotate is called anytime the rotary inputs change state.
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
