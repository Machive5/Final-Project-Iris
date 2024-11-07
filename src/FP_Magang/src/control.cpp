#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/BS2PC.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace ros;

class Robot{
    private:
        NodeHandle nh;
        Subscriber sub; 
        Subscriber imSub;
        Publisher pub;
        Publisher reqPub;
        float interval = 0;

        void updatePos(float er, float el){
            robotPos[0] = (er*sin(45*M_PI/180))+(el*sin(45*M_PI/180));
            robotPos[1] = (er*cos(45*M_PI/180))-(el*cos(45*M_PI/180));
        }

        float sigmoid(float interval){
            return 1.0 / (1.0 + exp(-2.5 * (2*interval - 1)));
        }

    public:
        FP_Magang::BS2PC latestMsg;
        float robotPos[2] = {0,0};
        float ballPos[2] = {0,0};
        float destination[2] = {0,0}; 
        int phase = 0;
        bool grab = false;
        bool manual = true;

        Robot(){
            sub = nh.subscribe("/bs2pc",1,&Robot::bs2pcHandler,this);
            imSub = nh.subscribe("/destination",1,&Robot::destinationHandler,this);
            pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs",50);
            reqPub = nh.advertise<std_msgs::Bool>("/imgRequest",50);
        }

        void bs2pcHandler(const FP_Magang::BS2PC& msg){

            if (msg.status == 3){
                destination[0] = msg.tujuan_x;
                destination[1] = msg.tujuan_y;
            }

            if (latestMsg.status != msg.status && (msg.status == 2 || msg.status == 4)){
                phase = 0;
                std_msgs::Bool m;
                m.data = true;
                reqPub.publish(m);
            }

            latestMsg = msg;
            updatePos(msg.enc_right,msg.enc_left);
        }

        void destinationHandler(const geometry_msgs::Point& msg){
            ballPos[0] = msg.x;
            ballPos[1] = msg.y;

            if (latestMsg.status == 2){
                destination[0] = ballPos[0];
                destination[1] = ballPos[1];
            }
            else if (latestMsg.status == 4){
                float x = ballPos[0] - robotPos[0];
                float y = ballPos[1] - robotPos[1];

                destination[0] = ballPos[0] - 100*cos(atan2(y,x));
                destination[1] = ballPos[1] - 100*sin(atan2(y,x));

                phase = 1;
            }
        }

        void ballPloter(){
            FP_Magang::PC2BS msg;
            msg.bola_x = ballPos[0];
            msg.bola_y = ballPos[1];
            pub.publish(msg);
            spinOnce();
        }

        void shoot(float distance){
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

        void move(float x, float y, float t){
            FP_Magang::PC2BS msg;

            if (manual) {
                float ct = -latestMsg.th; //last theta
                float sinus = sin(ct*M_PI/180);
                float cosinus = cos(ct*M_PI/180);

                float cx = x;
                float cy = y;
                
                x = (cx*cosinus) + (cy*-sinus);
                y = (cx*sinus) + (cy*cosinus);
            }

            if (!manual && interval < 1){
                interval += 0.02;
            }else{
                interval = 1;
            }  

            if (((robotPos[0]+(x/50))+27.7430 < 0) || ((robotPos[0]+(x/50))-926.866219 > 0)){
                x = 0;
            }
            if (((robotPos[1]+(y/50))+31.4311 < 0) || ((robotPos[1]+(y/50))-627.356796 > 0)){
                y = 0;
            }

            (round(x)==0)? x=0 : x=x;
            (round(y)==0)? y=0 : y=y;
            (round(t)==0)? t=0 : t=t;


            if ((round(x) == 0) && (round(y) == 0) && (round(t/10) == 0)){
                interval = 0;
                if (latestMsg.status == 4){
                    phase++;
                }
                return;
            }

            msg.motor1 = ((x*2/3)+(y*0)+(t*1/3))*sigmoid(interval);
            msg.motor2 = ((x*-1/3)+(y*(sqrt(3)/3))+(t*1/3))*sigmoid(interval);
            msg.motor3 = ((x*-1/3)+(y*-(sqrt(3)/3))+(t*1/3))*sigmoid(interval);

            msg.bola_x = ballPos[0];
            msg.bola_y = ballPos[1];
            pub.publish(msg);
            spinOnce();
        }
};


//system----------------------------------------------------------------------------

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
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

    return ch;
}

float calcDistance(float x1,float x2,float y1,float y2){
    return sqrt(pow((x2-x1),2)+pow((y2-y1),2));
}

//request handler------------------------------------------------------------------
void status1(Robot &robot){
    
    char key = getKey();

    if(key=='w'){
        robot.move(0,1000,0);  
    }
    else if(key=='s'){
        robot.move(0,-1000,0);  
    }
    else if(key=='a'){
        robot.move(-1000,0,0);  
    }
    else if(key=='d'){
        robot.move(1000,0,0);  
    }
    else if(key=='q'){
        robot.move(0,0,-1000);  
    }
    else if(key=='e'){
        robot.move(0,0,1000);  
    }
    else if(key=='z'){
        if (calcDistance(robot.robotPos[0], robot.ballPos[0], robot.robotPos[1], robot.ballPos[1])<=30){
            robot.grab = true;
        }
    }else if(key=='x'){
        robot.grab = false;
    }
    else if(key=='p' && robot.grab==true){
        robot.grab = false;
        robot.shoot(300);
    }
    else if(key=='o' && robot.grab==true){
        robot.grab = false;
        robot.shoot(100);
    }

    if (robot.grab == true){
        robot.ballPos[0]=robot.robotPos[0];
        robot.ballPos[1]=robot.robotPos[1];
        robot.ballPloter();
    }
}

void status2_3(Robot &robot){
    float x = robot.destination[0] - robot.robotPos[0];
    float y = robot.destination[1] - robot.robotPos[1];
    if (x==0 && y==0){
        return;
    }

    float th = 90 - (atan2(y,x)*180/M_PI);

    robot.move(x,y,(th-robot.latestMsg.th));
}

void status4(Robot &robot, float &svDeg){
    float x;
    float y;
    float th;

    if (robot.phase == 1){
        robot.manual = false;
        x = robot.destination[0] - robot.robotPos[0];
        y = robot.destination[1] - robot.robotPos[1];
        th = 90 - (atan2(y,x)*180/M_PI);
        svDeg = robot.latestMsg.th;
        robot.move(x,y,(th-(robot.latestMsg.th)));
    }
    else if (robot.phase == 2){
        if (abs((robot.latestMsg.th - svDeg)/720) >= 1){
            robot.phase = 3;
        }

        robot.manual = true;
        x = M_PI * 100;
        y = 0;
        th = 360;
        robot.move(x,y,-th);
    }
    else if (robot.phase == 3){
        robot.manual = false;
        x = robot.ballPos[0] - robot.robotPos[0];
        y = robot.ballPos[1] - robot.robotPos[1];
        th = 0;
        svDeg = 0;
        robot.move(x,y,th);
    }

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv){
    init(argc, argv, "control");
    // spin();
    Robot robot;
    float svDeg = 0;

    Rate rate(50);
    while(ok()){
        spinOnce();
        if (robot.latestMsg.status == 1){
            robot.manual = true;
            status1(robot);
            robot.move(0,0,0);
        }
        else if (robot.latestMsg.status == 2 || robot.latestMsg.status == 3){
            robot.manual = false;
            if (robot.latestMsg.status == 2){
                robot.ballPloter();
            }
            status2_3(robot);
        }
        else if (robot.latestMsg.status == 4){
            robot.manual = false;
            status4(robot,svDeg);
        }
        rate.sleep();
    }

    return 0;
}