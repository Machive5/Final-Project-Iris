#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace ros;

//global variable
FP_Magang::BS2PC latestMsg;
float robotPos[2] = {0,0};
float ballPos[2] = {0,0};
bool grab = false;

//system----------------------------------------------------------------------------
void ballPloter(Publisher &pub){
    FP_Magang::PC2BS msg;
    msg.bola_x = ballPos[0];
    msg.bola_y = ballPos[1];
    pub.publish(msg);
    spinOnce();
}

void shoot(Publisher &pub, float distance){
    float ct = latestMsg.th;
    float x = ballPos[0] + (distance*sin(ct*M_PI/180));
    float y = ballPos[1] + (distance*cos(ct*M_PI/180));

    if (x+27.7430 < 0){
        x = -27.7430;
    }else if (x-926.866219 > 0){
        x = 926.866219;
    }

    if (y+31.4311 < 0){
        y = -31.4311;
    }else if(y-627.356796 > 0){
        y = 627.356796;
    }

    ballPos[0] = x;
    ballPos[1] = y;

    FP_Magang::PC2BS msg;
    msg.bola_x = x;
    msg.bola_y = y;
    pub.publish(msg);
    spinOnce();
}

void move(float x, float y, float t, Publisher &pub){
    //cout<<"called"<<endl;
    FP_Magang::PC2BS msg;

    float ct = -latestMsg.th; //last theta
    float sinus = sin(ct*M_PI/180);
    float cosinus = cos(ct*M_PI/180);

    float cx = x;
    float cy = y;
    

    x = (cx*cosinus) + (cy*-sinus);
    y = (cx*sinus) + (cy*cosinus);

    //cout<<robotPos[0]+(x/50)<<","<<robotPos[1]+(y/50)<<endl;

    if (((robotPos[0]+(x/50))+27.7430 < 0) || ((robotPos[0]+(x/50))-926.866219 > 0)){
        x = 0;
    }
    if (((robotPos[1]+(y/50))+31.4311 < 0) || ((robotPos[1]+(y/50))-627.356796 > 0)){
        y = 0;
    }

    msg.motor1 = (x*2/3)-(y*0)+(t*-1/3);
    msg.motor2 = -(x*1/3)+(y*0.5774)+(t*-1/3);
    msg.motor3 = -(x*1/3)-(y*0.5774)+(t*-1/3);
    msg.bola_x = ballPos[0];
    msg.bola_y = ballPos[1];
    pub.publish(msg);
    spinOnce();
    //cout<<msg.motor1<<", "<<msg.motor2<<", "<<msg.motor3<<endl;
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

void updatePos(float er, float el){
    robotPos[0] = (er*sin(45*M_PI/180))+(el*sin(45*M_PI/180));
    robotPos[1] = (er*cos(45*M_PI/180))-(el*cos(45*M_PI/180));
}

float calcDistance(float x1,float x2,float y1,float y2){
    return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}

//request handler------------------------------------------------------------------
void status1(Publisher &pub){
    
    char key = getKey();
    //cout<<key<<endl;

    if(key=='w'){
        move(0,1000,0,pub);  
    }
    else if(key=='s'){
        move(0,-1000,0,pub);  
    }
    else if(key=='a'){
        move(-1000,0,0,pub);  
    }
    else if(key=='d'){
        move(1000,0,0,pub);  
    }
    else if(key=='q'){
        move(0,0,-1000,pub);  
    }
    else if(key=='e'){
        move(0,0,1000,pub);  
    }
    else if(key=='z'){
        if (calcDistance(robotPos[0], ballPos[0], robotPos[1], ballPos[1])<=30){
            grab = true;
        }
    }else if(key=='x'){
        grab = false;
    }
    else if(key=='p' && grab==true){
        grab = false;
        shoot(pub,300);
    }
    else if(key=='o' && grab==true){
        grab = false;
        shoot(pub,100);
    }

    if (grab == true){
        ballPos[0]=robotPos[0];
        ballPos[1]=robotPos[1];
        ballPloter(pub);
    }
}

void status2(Publisher &pub){
    float x = ballPos[0] - robotPos[0];
    float y = ballPos[1] - robotPos[1];
    float th = latestMsg.th-(atan(y/x)*180/M_PI);

    move(x,y,th,pub);
}

void bs2pcHandler(const FP_Magang::BS2PC& msg){
    ROS_INFO("status=[%f] x=[%f] y=[%f] el=[%f] er=[%f] th=[%f]", msg.status, msg.tujuan_x, msg.tujuan_y, msg.enc_left, msg.enc_right, msg.th);
    latestMsg = msg;
    updatePos(msg.enc_right,msg.enc_left);
    //ROS_INFO("x=[%f] y=[%f]", robotPos[0], robotPos[1]);
}

void destinationHandler(const geometry_msgs::Point& msg){
    ROS_INFO("msg rechieved x=[%f] y=[%f]", msg.x, msg.y);
    ballPos[0] = msg.x;
    ballPos[1] = msg.y;
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv){
    init(argc, argv, "control");

    termios old_tio;
    tcgetattr(STDIN_FILENO, &old_tio);

    NodeHandle nh;
    Subscriber sub = nh.subscribe("/bs2pc",1,bs2pcHandler);
    Subscriber imSub = nh.subscribe("/destination",1,destinationHandler);
    Publisher pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs",50);
    // spin();
    Rate rate(50);
    
    while(ok()){
        spinOnce();
        if (latestMsg.status == 1){
            status1(pub);
        }
        if (latestMsg.status == 2){
            ballPloter(pub);
            status2(pub);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        rate.sleep();
    }

    return 0;
}