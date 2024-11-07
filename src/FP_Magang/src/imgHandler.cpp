#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <FP_Magang/BS2PC.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;
using namespace ros;

class NodeObject{
    private:
        NodeHandle nh;
        Publisher pub;
        Subscriber sub;
    
    public:
        NodeObject(){
            sub = nh.subscribe("/imgRequest",1,&NodeObject::listener,this);
            pub = nh.advertise<sensor_msgs::Image>("/img",1);
        }

        void listener(const std_msgs::Bool::ConstPtr &msg){
            srand(time(0));
            int index = (rand()%3)+1;

            Mat img = imread("./src/FP_Magang/src/img/bola" + to_string(index)+".jpg");

            sensor_msgs::ImagePtr m = cv_bridge::CvImage(std_msgs::Header(),"rgb8",img).toImageMsg();
            ROS_INFO("sended");
            pub.publish(m);
            spinOnce();
        }
};

int main(int argc, char** argv){
    init(argc, argv, "imgHandler");

    NodeObject Node;
    spin();


    return 0;
}