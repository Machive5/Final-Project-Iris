#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include<iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace cv;
using namespace ros;

class NodeObject{
    private:
        NodeHandle nh;
        Publisher pub;
        Subscriber sub;
        // float robotPos[2] = {0,0}
        // float destination[2] = {-99,-99};
    
    public:
        NodeObject(){
            sub = nh.subscribe("/img",1,&NodeObject::listener,this);
            pub = nh.advertise<geometry_msgs::Point>("/destination",1);
        }

        void listener(const sensor_msgs::ImagePtr &msg){

            // if ((this.destination[0] != -99 && this.destination[0] != -99) && ()){

            // }
            
            Mat img = cv_bridge::toCvShare(msg,"rgb8")->image;
            Mat resized;
            Mat HSV;
            Mat thres;

            vector<vector<Point>> contours;
            Point2f center;
            float radius=0;
            int h[2] = {80,255};
            int s[2] = {58,255};
            int v[2] = {160,255};
            
            resize(img, resized, Size(900,600), INTER_LINEAR);

            cvtColor(resized, HSV, COLOR_RGB2HSV);
            inRange(HSV, Scalar(h[0], s[0], v[0]), Scalar(h[1],s[1],v[1]), thres);
            findContours(thres, contours, RETR_TREE, CHAIN_APPROX_NONE);
            drawContours(resized, contours, -1, Scalar(0,255,255),2);

            for (int i=0 ; i<contours.size(); i++){
                Point2f cntr_cmpr;
                float r_cmpr;
                minEnclosingCircle(contours[i], cntr_cmpr, r_cmpr);

                if(radius<r_cmpr){
                    center = cntr_cmpr;
                    radius = r_cmpr;
                }
            }

            publish(center.x, center.y);
            
        }
        
        // void updatePos(float er, float el){
        //     robotPos[0] = (er*sin(45*M_PI/180))+(el*sin(45*M_PI/180));
        //     robotPos[1] = (er*cos(45*M_PI/180))-(el*cos(45*M_PI/180));
        // }

        void publish(float x, float y){
            geometry_msgs::Point msg;

            msg.x = x;
            msg.y = y;

            pub.publish(msg);
        }
};

int main(int argc, char** argv){
    init(argc, argv, "process");

    NodeObject Node;
    spin();
    return 0;
}
