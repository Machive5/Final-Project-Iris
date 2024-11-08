#include <ros/ros.h>
//#include <FP_Magang/PC2BS.h>
//#include <FP_Magang/BS2PC.h>
#include <geometry_msgs/Point.h>
#include <iostream>

using namespace std;
using namespace ros;

void bs2pcHandler(const geometry_msgs::Point& msg){
    ROS_INFO(" th robot=[%f] theta tujuan=[%f] selisih=[%f]", msg.x, msg.y, msg.z);
}

int main(int argc, char** argv){
    init(argc,argv,"check");
    NodeHandle nh;
    Subscriber sub = nh.subscribe("/check",1,bs2pcHandler);
    spin();
}