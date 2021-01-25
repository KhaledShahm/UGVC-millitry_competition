#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include <sstream> 
#include <bits/stdc++.h>
#include <time.h>

using namespace std;
#define dist_to_dest_THRESHOLD 5
#define DIRECTION_THRESHOLD 5
#define LIDAR_VIEW_RANGE 30
#define OBSTACLE_DISTANCE_THRESHOLD 2
#define ESCAPE_ANGLE 45
#define PI           3.14159265358979323846  /* pi */
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(y) ((y)*M_PI/180)
#define left   2
#define right  1



vector<float> lidar(720);
float my_direction, dest_angle;
pair<float, float> my_coord ;
pair<float, float>  new_coord;
vector< pair<float, float> > destinations;
pair<float, float> destination ;
float dist_to_dest, direction_between;
bool s = false ;

//ofstream f("logs.txt");

void delay (float s)
{
    int sec = int(s*1000000);
    usleep(sec);
}

bool within_dist_to_dest_threshold()
{
    return dist_to_dest<= dist_to_dest_THRESHOLD;
}

bool within_direction_threshold()
{
    return direction_between<= DIRECTION_THRESHOLD;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //f<<"scancallback"<<endl;
    //lidar[0]= scan->ranges[0];
   // lidar[719-(2*ESCAPE_ANGLE)]= scan->ranges[719-(2*ESCAPE_ANGLE)];

  for(int i=0; i<720; i++) lidar[i]= scan->ranges[i];
}

void get_data(const geometry_msgs::Pose::ConstPtr& msg)
{
    //f<<"get data::"<<endl;
    my_direction= msg->position.x ;
    my_coord.first = msg->position.y ; 
    my_coord.second = msg->position.z ; 
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

float courseTo(float lat1, float long1, float lat2, float long2)
{
    //cout<<"courseto start"<<endl;
    float dlon = DEG2RAD(long2-long1);
    lat1 =DEG2RAD(lat1);
    lat2 = DEG2RAD(lat2);
    float a1 = sin(dlon) * cos(lat2);
    float a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0)
    {
      a2 += (2* PI);
    }
    //cout<<"courseto end"<<endl;
    return RAD2DEG(a2);
}

float change_angle_format(float angle)
{
    if(angle<= 270) return 90- angle;
    else return 450- angle;
}

float distanceBetween(float lat1 ,float long1,float lat2, float long2 )
{
 float alpha = DEG2RAD(lat2-lat1);
 float delta = DEG2RAD(long2-long1);
 lat1 = DEG2RAD(lat1);
 lat2 = DEG2RAD(lat2);
 float a = sin(alpha/2)*sin(alpha/2) + cos(lat1)*cos(lat2)*sin(delta/2)*sin(delta/2);
 float c = 2*atan2(sqrt(a),sqrt(1-a));
 float ans= c * 6371000;
 //cout<<"distanceBetween:: "<<ans<<endl;
 //f<<"distanceBetween:: "<<ans<<endl;
 return ans;
 }

float cross_product(float my_direction, float destination)
{
    float my_direction_x= cos(DEG2RAD(my_direction));
    float my_direction_y= sin(DEG2RAD(my_direction));
    float destination_x= cos(DEG2RAD(destination));
    float destination_y= sin(DEG2RAD(destination));

    return ( my_direction_x* destination_y) - (my_direction_y* destination_x);
}


void turn_right(ros::Publisher chatter_pub)
{
      //f<<"turn_right::"<<endl;
      ros::Rate loop_rate(10);   
      std_msgs::Int32 msg;
      msg.data = 2;
     // ROS_INFO("%d", msg.data);
   // cout<<"go forward \n"<<endl;
      chatter_pub.publish(msg);            //publish (msg) in topic  Right
      ros::spinOnce();
      loop_rate.sleep();
}

void turn_left(ros::Publisher chatter_pub)
{
      //f<<"turn_left::"<<endl;
      ros::Rate loop_rate(10);   
      std_msgs::Int32 msg;
      msg.data = 1;
     // ROS_INFO("%d", msg.data);
   // cout<<"go forward \n"<<endl;
      chatter_pub.publish(msg);            //publish (msg) in topic  Right
      ros::spinOnce();
      loop_rate.sleep();
}

void turn(ros::Publisher chatter_pub){

    //f<<"turn:: my_direction:"<<my_direction<<"  dest_angle:"<<dest_angle<<endl;

    if( cross_product(my_direction, dest_angle)> 0) 
        turn_left(chatter_pub); 
    else 
        turn_right(chatter_pub);

}

bool obstacle_found_at_angle(float angle)
{
    //cout<<"obstacle_at_angle start :: "<<"angle:"<<angle<<endl;

    int start_angle= (angle- (LIDAR_VIEW_RANGE));
    int end_angle=  (angle+ (LIDAR_VIEW_RANGE));
    for(int i= start_angle; i<= end_angle; i++)
        if(lidar[i%720] < OBSTACLE_DISTANCE_THRESHOLD && lidar[i%720] !=0 )
         return true;  
    return false;
    ros::spinOnce();
}

void escape_obstacle(char d ,ros::Publisher chatter_pub )
{
  switch (d)
  {
    case 1 :
            turn_right(chatter_pub);
            delay(6);
            break ;
    case 2 :
            turn_left(chatter_pub);
            delay(6);
            break ;
  }
}

bool obstcal(ros::Publisher chatter_pub)
{
  bool found= false;
    for(int i=719-(LIDAR_VIEW_RANGE); i< 719+(LIDAR_VIEW_RANGE); i++)
       if(lidar[i%720] < OBSTACLE_DISTANCE_THRESHOLD && lidar[i%720] !=0  )
       {
        cout<<"obstcal found"<<endl; 
        found= true;
             if (obstacle_found_at_angle(719-(2*ESCAPE_ANGLE))) 
            {
                escape_obstacle(left,chatter_pub);     
            }
            else
            {
                escape_obstacle(right,chatter_pub);
            }
       }

    cout<<"no obstacle "<<endl;
    return found;
}

// int MyAngleToLidarAngle(float angle)
// {
//     angle*=2;
//     if(angle<= 0) return -1*angle;
//     else return 720- angle;
// }

/*pair<float, float> jump_to_Right(float angle)
{
    if(angle<=180) pair<float, float> new_coord= make_pair(my_coord.first/OBSTACLE_DISTANCE_lat_THRESHOLD, my_coord.second /OBSTACLE_DISTANCE_long_THRESHOLD);
    else if(angle< 0) pair<float, float> new_coord= make_pair(my_coord.first * OBSTACLE_DISTANCE_lat_THRESHOLD, my_coord.second *OBSTACLE_DISTANCE_long_THRESHOLD);
    return new_coord;
}

pair<float, float> jump_to_Left(float angle)
{
    if(angle<=180) pair<float, float> new_coord= make_pair(my_coord.first*OBSTACLE_DISTANCE_lat_THRESHOLD, my_coord.second *OBSTACLE_DISTANCE_long_THRESHOLD);
    else if(angle< 0) pair<float, float> new_coord= make_pair(my_coord.first / OBSTACLE_DISTANCE_lat_THRESHOLD, my_coord.second /OBSTACLE_DISTANCE_long_THRESHOLD);
    return new_coord;
}
pair<float, float> escape_obstacle(float my_direction, pair<float, float> my_coord)
{
    if(!obstacle_found_at_angle( 719- (2*ESCAPE_ANGLE))) return jump_to_Left(my_direction+ ESCAPE_ANGLE);
    else return jump_to_Right(my_direction- ESCAPE_ANGLE);
}*/

void go(ros::Publisher chatter_pub)
{
      //f<<"go "<<endl;
      ros::Rate loop_rate(10);   
      std_msgs::Int32 msg;
      msg.data = 3;
     // ROS_INFO("%d", msg.data);
     // cout<<"go forward \n"<<endl;
      chatter_pub.publish(msg);            //publish (msg) in topic  Right
      ros::spinOnce();
      loop_rate.sleep();
    //cout<<"go end"<<endl;
}

void stop(ros::Publisher chatter_pub)
{
      ros::Rate loop_rate(10);   
      std_msgs::Int32 msg;
      msg.data = 4;
     // ROS_INFO("%d", msg.data);
     // cout<<"go forward \n"<<endl;
      chatter_pub.publish(msg);            //publish (msg) in topic  Right
      ros::spinOnce();
    loop_rate.sleep();
}

void calc()
{ 
    //f<<"calc:: my_coord.first:"<<my_coord.first<<"  my_coord.second:"<<my_coord.second<<endl;
    float h = courseTo(my_coord.first, my_coord.second, destination.first, destination.second);
    dest_angle = change_angle_format(h) ;
    //cout<< dest_angle <<endl; 
    direction_between= abs(my_direction- dest_angle);
    dist_to_dest= distanceBetween(my_coord.first, my_coord.second, destination.first, destination.second);
    //f<<"calc:: dist_to_dest:"<<dist_to_dest<<endl;
    //cout<<"calc:: direction_between"<<direction_between<<endl;
    //f<<"calc:: direction_between"<<direction_between<<endl; 
}


int main(int argc, char **argv)
{
    
 // Add latitude and longitude --> 6 position
    destinations.push_back(make_pair(30.081461, 31.298083));
    destinations.push_back(make_pair(30.08157364, 31.29833187));
    destinations.push_back(make_pair(30.08128731, 31.29856412));
    destinations.push_back(make_pair(30.08117062, 31.29778525));
    destinations.push_back(make_pair(30.08082592, 31.29800403));
    destinations.push_back(make_pair(30.08124291, 31.29799704));
    ros::init(argc, argv, "Arduino1_data"); 
    ros::init(argc, argv, "ydlidar_client");
    ros::init(argc, argv, "talker");           //init node         

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000); 
    ros::Subscriber sub2       = n.subscribe("sum", 1000, get_data); 
    ros::Subscriber sub        = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Rate loop_rate(10);  


   //f<<"file created"<<endl;
   delay(3); 
  while(!destinations.empty())
    {
        destination= destinations[0];
        //f<<"current destination:: x:"<<destination.first<<"  y:"<<destination.second<<endl;
        calc();     
        if( within_dist_to_dest_threshold())
        {
            destinations.erase(destinations.begin());
            continue;
        }

        adjust_direction:   
        while( !within_direction_threshold())
        {
            //f<<"turn"<<endl;
            turn(chatter_pub);
            calc();
        }
        
        adjust_Obstcal :
        bool y = obstcal(chatter_pub);
        ros::spinOnce();
        if(y)
        {
            go(chatter_pub);
            delay(3);
            continue;
         }

         while(!within_dist_to_dest_threshold())
        {
            go( chatter_pub);
            delay (1);
            calc();
            if (!within_direction_threshold()) goto adjust_direction ;
        }
                

   /* while(1)
    {
        adjust_Obstcal : 
        bool y = obstcal();
        ros::spinOnce();
        if(y)
        {
            cout<<"start_obstcal"<<endl;
            stop(chatter_pub);
            ros::spinOnce();
            
           // go(chatter_pub);
            //delay(5);
         }*/
    }
    stop(chatter_pub);
    cout<<"done"<<endl;
    //f<<"done"<<endl;
    return 0;

}

