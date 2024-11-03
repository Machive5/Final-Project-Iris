#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>

using namespace std;
using namespace cv;
using namespace ros;

//global variable
FP_Magang::BS2PC latest;

//system
void move(float x, float y, float t, Publisher &pub){
    cout<<"called"<<endl;
    FP_Magang::PC2BS msg;
    msg.motor1 = (x*2/3)+(y*0)+(t*1/3);
    msg.motor2 = -(x*1/3)-(y*0.5774)+(t*1/3);
    msg.motor3 = -(x*1/3)+(y*0.5774)+(t*1/3);
    pub.publish(msg);

    cout<<msg.motor1<<", "<<msg.motor2<<", "<<msg.motor3<<endl;
    spinOnce();
}

char getKey(){
    // Declare a termios structure to store terminal settings
    struct termios old_tio, new_tio;

    // Get current terminal settings and save them for later restoration
    tcgetattr(STDIN_FILENO, &old_tio);
    
    // Create a copy of the current settings, which we will modify
    new_tio = old_tio;

    // Disable canonical mode (ICANON) and echo (ECHO)
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 1;              // Minimum number of characters to read
    new_tio.c_cc[VTIME] = 0;

    // Set the new attributes (TCSANOW means to apply the changes immediately)
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval timeout = {0, 0};

    char ch;
    if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0){
        read(STDIN_FILENO, &ch, 1);
    }

    return ch;
}


//request handler
void status1(Publisher &pub){
    
    char key = getKey();
    cout<<key<<endl;

    if(key=='w'){
        move(-100,0,0,pub);
    }else if(key=='s'){
        move(100,0,0,pub);
    }else if(key=='a'){
        move(0,-100,0,pub);
    }else if(key=='d'){
        move(0,100,0,pub);
    }else if(key=='q'){
        move(0,0,100,pub);
    }else if(key=='e'){
        move(0,0,-100,pub);
    }
}

void bs2pcHandler(const FP_Magang::BS2PC& msg){
    ROS_INFO("status=[%f] x=[%f] y=[%f] el=[%f] er=[%f] th=[%f]", msg.status, msg.tujuan_x, msg.tujuan_y, msg.enc_left, msg.enc_right, msg.th);
    latest = msg;
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv){
    init(argc, argv, "publisher");

    termios old_tio;
    tcgetattr(STDIN_FILENO, &old_tio);

    NodeHandle nh;
    Subscriber sub = nh.subscribe("/bs2pc",1,bs2pcHandler);
    Publisher pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs",1);
    // spin();
    Rate rate(1);
    
    while(ok()){
        if (latest.status == 1){
            status1(pub);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

        spinOnce();
        rate.sleep();
    }

    return 0;
}