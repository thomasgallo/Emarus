/* Arduino programm for control Motor and Encorder
 *  Used for the Last experimental Project
 *  Erasmus/Emaro Robotics 2019 Genova
 */

/* usefull distances */
#define WHEEL_SEP_WIDTH 0.105 //(21cm/2)
#define WHEEL_SEP_LENGTH 0.0475 //(9.5cm/2)
#define WHEEL_RAD 0.04 //(radius of 4 cm)
#define MAX_ROT 30.0


/* Front Left motor (FL) configuration */
#define SPEEDFL 3 //gray between arduino and motor driver
#define OUTFLA 25 //purple between arduino and motor driver
#define OUTFLB 24 //blue between arduino and motor driver
#define INFLA 23 //green between arduino and motor
#define INFLB 22 //orange between arduino and motor


/* Front Right motor (FR) configuration */
#define SPEEDFR 4 //gray between arduino and motor driver
#define OUTFRA 31 //purple between arduino and motor driver
#define OUTFRB 30 //blue between arduino and motor driver
#define INFRB 29//green between arduino and motor
#define INFRA 28 //orange between arduino and motor


/* rear Left motor (RL) configuration */
#define SPEEDRL 5 //gray between arduino and motor driver
#define OUTRLA 37 //purple between arduino and motor driver
#define OUTRLB 36 //blue between arduino and motor driver
#define INRLA 35 //green between arduino and motor
#define INRLB 34 //orange between arduino and motor


/* rear Right motor (RR) configuration */
#define SPEEDRR 6 //gray between arduino and motor driver
#define OUTRRA 43 //purple between arduino and motor driver
#define OUTRRB 42 //blue between arduino and motor driver
#define INRRB 41 //green between arduino and motor
#define INRRA 40 //orange between arduino and motor


#define USE_USBCON //for arduino mega working with roserial
#include <ros.h>
#include <geometry_msgs/Twist.h> //for cmd_vel

//ROS node
ros::NodeHandle motors_control_;

//Get usefull motors velocity references
double x,y,rz;

//for safety
unsigned long prev_time;
unsigned long curr_time;

//for requested velocity of each wheels
double wheel_FL;
double wheel_FR;
double wheel_RL;
double wheel_RR;


//Calback
void twistCallback( const geometry_msgs::Twist& cmd_vel){
    x = cmd_vel.linear.x;
    y = cmd_vel.linear.y;
    rz = cmd_vel.angular.z;
    prev_time = millis();
}


//Subscriber
ros::Subscriber<geometry_msgs::Twist> sub_vel_("/cmd_vel", twistCallback);

//redifined the function map because we want double
double map_func(double x, double in_min, double in_max, double out_min, double out_max) {
  return constrain((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 0, 255);
}

void setup() {

    //ROS config
    motors_control_.initNode(); //Initialize ROS node
    motors_control_.subscribe(sub_vel_);
    
    //Front Left Motor
    pinMode(SPEEDFL, OUTPUT);
    pinMode(OUTFLA, OUTPUT);
    pinMode(OUTFLB, OUTPUT);
    pinMode(INFLA, INPUT);
    pinMode(INFLB, INPUT);

    //Fron Right Motor
    pinMode(SPEEDFR, OUTPUT);
    pinMode(OUTFRA, OUTPUT);
    pinMode(OUTFRB, OUTPUT);
    pinMode(INFRA, INPUT);
    pinMode(INFRB, INPUT);

    //Rear Left Motor
    pinMode(SPEEDRL, OUTPUT);
    pinMode(OUTRLA, OUTPUT);
    pinMode(OUTRLB, OUTPUT);
    pinMode(INRLA, INPUT);
    pinMode(INRLB, INPUT);

    //Rear Right Motor
    pinMode(SPEEDRR, OUTPUT);
    pinMode(OUTRRA, OUTPUT);
    pinMode(OUTRRB, OUTPUT);
    pinMode(INRRA, INPUT);
    pinMode(INRRB, INPUT);
}

void loop() {

    //compute the requested velocity
    wheel_FL = (1.0/WHEEL_RAD) * (y-x-(WHEEL_SEP_WIDTH + WHEEL_SEP_LENGTH)*rz);
    wheel_FR = (1.0/WHEEL_RAD) * (y+x+(WHEEL_SEP_WIDTH + WHEEL_SEP_LENGTH)*rz);
    wheel_RL = (1.0/WHEEL_RAD) * (y+x-(WHEEL_SEP_WIDTH + WHEEL_SEP_LENGTH)*rz);
    wheel_RR = (1.0/WHEEL_RAD) * (y-x+(WHEEL_SEP_WIDTH + WHEEL_SEP_LENGTH)*rz);

    curr_time = millis() - prev_time;
    
    if((x == 0 && y == 0 && rz == 0) || curr_time >= 5000 ){ //brake
        digitalWrite(OUTFLA, LOW);
        digitalWrite(OUTFLB, LOW);
        digitalWrite(OUTFRA, LOW);
        digitalWrite(OUTFRB, LOW);
        digitalWrite(OUTRLA, LOW);
        digitalWrite(OUTRLB, LOW);
        digitalWrite(OUTRRA, LOW);
        digitalWrite(OUTRRB, LOW);
    }
    else{
        //Front left
        if(wheel_FL >= 0){ //forward
            digitalWrite(OUTFLB, LOW);
            digitalWrite(OUTFLA, HIGH);
            analogWrite(SPEEDFL, map_func(fabs(wheel_FL), 0, MAX_ROT, 0, 255));
        }
        else{ //backward
            digitalWrite(OUTFLA, LOW);
            digitalWrite(OUTFLB, HIGH);
            analogWrite(SPEEDFL, map_func(fabs(wheel_FL), 0, MAX_ROT, 0, 255));
        }

        //front right
        if(wheel_FR >= 0){ //forward
            digitalWrite(OUTFRB, LOW);
            digitalWrite(OUTFRA, HIGH);
            analogWrite(SPEEDFR, map_func(fabs(wheel_FR), 0, MAX_ROT, 0, 255));
        }
        else{ //backward
            digitalWrite(OUTFRA, LOW);
            digitalWrite(OUTFRB, HIGH);
            analogWrite(SPEEDFR, map_func(fabs(wheel_FR), 0, MAX_ROT, 0, 255));
        }
        
        //rear left
        if(wheel_RL >= 0){ //forward
            digitalWrite(OUTRLB, LOW);
            digitalWrite(OUTRLA, HIGH);
            analogWrite(SPEEDRL, map_func(fabs(wheel_RL), 0, MAX_ROT, 0, 255));
        }
        else{ //backward
            digitalWrite(OUTRLA, LOW);
            digitalWrite(OUTRLB, HIGH);
            analogWrite(SPEEDRL, map_func(fabs(wheel_RL), 0, MAX_ROT, 0, 255));
        }

        //rear right
        if(wheel_RR >= 0){ //forward
            digitalWrite(OUTRRB, LOW);
            digitalWrite(OUTRRA, HIGH);
            analogWrite(SPEEDRR, map_func(fabs(wheel_RR), 0, MAX_ROT, 0, 255));
        }
        else{ //backward
            digitalWrite(OUTRRA, LOW);
            digitalWrite(OUTRRB, HIGH);
            analogWrite(SPEEDRR, map_func(fabs(wheel_RR), 0, MAX_ROT, 0, 255));
        }
    }
    motors_control_.spinOnce();
    delay(100);
}
