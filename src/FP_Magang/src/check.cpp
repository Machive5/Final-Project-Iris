#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <iostream>

using namespace std;
using namespace ros;

void bs2pcHandler(const FP_Magang::BS2PC& msg){
    ROS_INFO("status=[%f] x=[%f] y=[%f] el=[%f] er=[%f] th=[%f]", msg.status, msg.tujuan_x, msg.tujuan_y, msg.enc_left, msg.enc_right, msg.th);
}

int main(int argc, char** argv){
    init(argc,argv,"check");
    NodeHandle nh;
    Subscriber sub = nh.subscribe("/bs2pc",1,bs2pcHandler);
    spin();
}