#include <L298N.h>
#include <ros.h>


//pin definition
#define ENB 10
#define IN1 8
#define IN2 7
//pin definition
#define ENA 9 
#define IN3 6
#define IN4 5


//create a motor instance
L298N motor_R(ENB, IN1, IN2);
L298N motor_L(ENA, IN3, IN4);

//create the ros node nh. The node will be used to publish to Arduino
ros::NodeHandle nh;
//std_msgs::Float32 msg;
//ros::Publisher pub("MPU", &msg);  //topic

unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
float a[3],w[3],angle[3],T;
void setup() {
  // initialize Serial:
  Serial.begin(115200);
  Serial1.begin(115200);
  TCCR2B = TCCR2B & B11111000 | B00000011; //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  motor_R.setSpeed(110); // an integer between 0 and 255
  motor_L.setSpeed(110); // an integer between 0 and 255
 // nh.initNode();
//  nh.advertise(pub);
}

void loop() {
   motor_R.forward();
   motor_L.forward();
    while (Serial1.available()) {
   // char inChar = (char)Serial1.read(); Serial.print(inChar); //Output Original Data, use this code 
    Re_buf[counter]=(unsigned char)Serial1.read(); 
    if(counter==0&&Re_buf[0]!=0x55) return;                 
    counter++;       
    if(counter==11)          
    {    
       counter=0;               
       sign=1;
    }
      
  }
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {       
  switch(Re_buf [1])
  {
  case 0x51:
    a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
    a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
    a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
    break;
  case 0x52:
    w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
    w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
    w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
    break;
  case 0x53:
          angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
    angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*180;
    angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
//      msg.data = angle[2];
//      pub.publish(&msg);
        break;
  } 
    }
  }
  nh.spinOnce(); 
}
