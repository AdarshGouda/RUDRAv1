
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//initializing all the variables

#define LOOPTIME  100                          //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter


int mPinRight[6] = {8, 25, 24, 22, 23, 13};
int mPinLeft[6] = {7, 26, 27, 29, 28, 10};

int pid_enable = 0;


const int PIN_ENCOD_A_MOTOR_LEFT1 = 18;
const int PIN_ENCOD_B_MOTOR_LEFT1 = 16;
const int PIN_ENCOD_A_MOTOR_LEFT2 = 19;
const int PIN_ENCOD_B_MOTOR_LEFT2 = 14;
const int PIN_ENCOD_A_MOTOR_RIGHT1 = 2;
const int PIN_ENCOD_B_MOTOR_RIGHT1 = 4;
const int PIN_ENCOD_A_MOTOR_RIGHT2 = 3;
const int PIN_ENCOD_B_MOTOR_RIGHT2 = 5;

unsigned long lastMilli = 0;


const double radius = 0.04;                 //Wheel radius, in m
const double wheelbase = 0.23;              //Wheelbase, in m


const double encoder_cpr = 330;

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s

double speed_act_left1 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left1 = 0;                    //Command speed for left wheel in m/s


double speed_act_left2 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left2 = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s

double speed_act_right1 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right1 = 0;                   //Command speed for right wheel in m/s


double speed_act_right2 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right2 = 0;                   //Command speed for right wheel in m/s

const double max_speed = 1.20;
//const double min_speed = 0.85;                //Max speed in m/s

int PWM_leftMotor1 = 0;                     //PWM command for left motor
int PWM_rightMotor1 = 0;                    //PWM command for right motor
int PWM_leftMotor2 = 0;                     //PWM command for left motor
int PWM_rightMotor2 = 0;                    //PWM command for right motor

int pwm_set = 80;

bool L1disable = 0;
bool L2disable = 0;
bool R1disable = 0;
bool R2disable = 0;

volatile float pos_left1 = 0;       //Left motor encoder position
volatile float pos_right1 = 0;      //Right motor encoder position
volatile float pos_left2 = 0;       //Left motor encoder position
volatile float pos_right2 = 0;      //Right motor encoder position

float speedL1 = 0.0;
float speedR1 = 0.0;
float speedL2 = 0.0;
float speedR2 = 0.0;

// PID Parameters


const double PID_left_param1[] =  { 4.0, 0.01, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param1[] = { 4.0, 0.01, 0.001 }; //Respectively Kp, Ki and Kd for right motor PID
const double PID_left_param2[] =  { 4.0, 0.01, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param2[] = { 4.0, 0.01, 0.001 }; //Respectively Kp, Ki and Kd for right motor PID

PID PID_leftMotor1 (&speed_act_left1,  &speed_cmd_left1,  &speed_req_left,  PID_left_param1[0],  PID_left_param1[1],  PID_left_param1[2],  DIRECT);   //Setting up the PID for left motor
PID PID_rightMotor1(&speed_act_right1, &speed_cmd_right1, &speed_req_right, PID_right_param1[0], PID_right_param1[1], PID_right_param1[2], DIRECT);   //Setting up the PID for right motor
PID PID_leftMotor2 (&speed_act_left2,  &speed_cmd_left2,  &speed_req_left,  PID_left_param2[0],  PID_left_param2[1],  PID_left_param2[2],  DIRECT);   //Setting up the PID for left motor
PID PID_rightMotor2(&speed_act_right2, &speed_cmd_right2, &speed_req_right, PID_right_param2[0], PID_right_param2[1], PID_right_param2[2], DIRECT);   //Setting up the PID for right motor



ros::NodeHandle nh;

//function that will be called when receiving command from host

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type




void setup() {

  for (int i = 0; i < 6; i++) {
    pinMode(mPinLeft[i], OUTPUT);
    pinMode(mPinRight[i], OUTPUT);
  }

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  analogWrite(mPinLeft[0], 0);
  analogWrite(mPinLeft[5], 0);
  analogWrite(mPinRight[0], 0);
  analogWrite(mPinRight[5], 0);

  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT1, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT1, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT1, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT1, HIGH);
  attachInterrupt(5, encoderLeftMotor1, RISING); //pin18
  //attachInterrupt(3, encoderLeftMotor1, RISING); //pin 20

  pinMode(PIN_ENCOD_A_MOTOR_LEFT2, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT2, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT2, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT2, HIGH);
  attachInterrupt(4, encoderLeftMotor2, RISING); //Pin19
  //attachInterrupt(2, encoderLeftMotor2, RISING); //Pin21

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT1, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT1, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT1, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT1, HIGH);
  attachInterrupt(0, encoderRightMotor1, RISING);

  pinMode(PIN_ENCOD_A_MOTOR_RIGHT2, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT2, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT2, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT2, HIGH);
  attachInterrupt(1, encoderRightMotor2, RISING);

  //setting PID parameters

  PID_leftMotor1.SetSampleTime(95);
  PID_rightMotor1.SetSampleTime(95);
  PID_leftMotor1.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor1.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor1.SetMode(AUTOMATIC);
  PID_rightMotor1.SetMode(AUTOMATIC);

  PID_leftMotor2.SetSampleTime(95);
  PID_rightMotor2.SetSampleTime(95);
  PID_leftMotor2.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor2.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor2.SetMode(AUTOMATIC);
  PID_rightMotor2.SetMode(AUTOMATIC);

}

void loop() {



  nh.spinOnce();



  if ((millis() - lastMilli) >= LOOPTIME)

  { // enter timed loop

    lastMilli = millis();

    if (abs(pos_left1) < 5) {                                                  //Avoid taking in account small disturbances
      speed_act_left1 = 0;
    }
    else {
      speed_act_left1 = ((pos_left1 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_left2) < 5) {                                                  //Avoid taking in account small disturbances
      speed_act_left2 = 0;
    }
    else {
      speed_act_left2 = ((pos_left2 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_right1) < 5) {                                                 //Avoid taking in account small disturbances
      speed_act_right1 = 0;
    }
    else {
      speed_act_right1 = ((pos_right1 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    if (abs(pos_right2) < 5) {                                                 //Avoid taking in account small disturbances
      speed_act_right2 = 0;
    }
    else {
      speed_act_right2 = ((pos_right2 / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    pos_left1 = 0;
    pos_right1 = 0;
    pos_left2 = 0;
    pos_right2 = 0;

    //int pwm_set = 4100;

    if (pid_enable == 1) {

      PID_leftMotor1.Compute();
      PID_leftMotor2.Compute();

      PID_rightMotor1.Compute();
      PID_rightMotor2.Compute();

      speedL1 = abs(speed_cmd_left1);
      speedR1 = abs(speed_cmd_right1);
      speedL2 = abs(speed_cmd_left2);
      speedR2 = abs(speed_cmd_right2);

    } else {

      speedL1 = abs(speed_req_left);
      speedR1 = abs(speed_req_right);
      speedL2 = abs(speed_req_left);
      speedR2 = abs(speed_req_right);

    }

    if (speedL1 <= 0.80) {

      PWM_leftMotor1 = 0;
    }
    else {

      PWM_leftMotor1 = constrain((1595.5 *  speedL1 * speedL1 * speedL1 - 4041.3 * speedL1 * speedL1 + 3534.1 * speedL1 - 974.66), 0, 255);

    }

    if (speedR1 <= 0.80) {

      PWM_rightMotor1 = 0;
    }
    else {
      PWM_rightMotor1 = constrain((672.02 * speedR1 * speedR1 * speedR1 - 1493.3 * speedR1 * speedR1 + 1168.0 * speedR1 - 238.37), 0, 255);

    }


    if (speedL2 <= 0.80) {

      PWM_leftMotor2 = 0;
    }
    else {

      PWM_leftMotor2 = constrain((736.24 *  speedL2 * speedL2 * speedL2 - 1431.7 * speedL2 * speedL2 + 981.11 * speedL2 - 156.27), 0, 255);

    }

    if (speedR2 <= 0.80) {

      PWM_rightMotor2 = 0;
    }
    else {
      PWM_rightMotor2 = constrain((62.955 * speedR2 * speedR2 * speedR2 + 105.69 * speedR2 * speedR2 - 145.87 * speedR2 + 120.57), 0, 255);

    }

    //Motor Left 1

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      analogWrite(mPinLeft[0], 0);
      digitalWrite(mPinLeft[1], 0);
      digitalWrite(mPinLeft[2], 0);

    }
    else if (L1disable == 1) {

      analogWrite(mPinLeft[0], 0);
      digitalWrite(mPinLeft[1], 0);
      digitalWrite(mPinLeft[2], 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      analogWrite(mPinLeft[0], 0);
      digitalWrite(mPinLeft[1], 0);
      digitalWrite(mPinLeft[2], 0);
    }
    else if (speed_req_left > 0) {                         //Going forward

      analogWrite(mPinLeft[0], abs(PWM_leftMotor1));
      digitalWrite(mPinLeft[1], 1);
      digitalWrite(mPinLeft[2], 0);

    }
    else {                                               //Going backward
      analogWrite(mPinLeft[0], abs(PWM_leftMotor1));
      digitalWrite(mPinLeft[1], 0);
      digitalWrite(mPinLeft[2], 1);
    }

    //Motor Left 2

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      analogWrite(mPinLeft[5], 0);
      digitalWrite(mPinLeft[3], 0);
      digitalWrite(mPinLeft[4], 0);

    }
    else if (L2disable == 1) {
      analogWrite(mPinLeft[5], 0);
      digitalWrite(mPinLeft[3], 0);
      digitalWrite(mPinLeft[4], 0);

    }
    else if (speed_req_left == 0) {                       //Stopping

      analogWrite(mPinLeft[5], 0);
      digitalWrite(mPinLeft[3], 0);
      digitalWrite(mPinLeft[4], 0);
    }
    else if (speed_req_left > 0) {                         //Going forward

      analogWrite(mPinLeft[5], abs(PWM_leftMotor2));
      digitalWrite(mPinLeft[3], 0);
      digitalWrite(mPinLeft[4], 1);

    }
    else {                                               //Going backward
      analogWrite(mPinLeft[5], abs(PWM_leftMotor2));
      digitalWrite(mPinLeft[3], 1);
      digitalWrite(mPinLeft[4], 0);
    }

    //Motor Right 1

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      analogWrite(mPinRight[0], 0);
      digitalWrite(mPinRight[1], 0);
      digitalWrite(mPinRight[2], 0);

    }

    else if (R1disable == 1) {
      analogWrite(mPinRight[0], 0);
      digitalWrite(mPinRight[1], 0);
      digitalWrite(mPinRight[2], 0);
    }
    else if (speed_req_right == 0) {                       //Stopping

      analogWrite(mPinRight[0], 0);
      digitalWrite(mPinRight[1], 0);
      digitalWrite(mPinRight[2], 0);
    }
    else if (speed_req_right > 0) {                         //Going forward

      analogWrite(mPinRight[0], abs(PWM_rightMotor1));
      digitalWrite(mPinRight[1], 1);
      digitalWrite(mPinRight[2], 0);

    }
    else {                                               //Going backward
      analogWrite(mPinRight[0], abs(PWM_rightMotor1));
      digitalWrite(mPinRight[1], 0);
      digitalWrite(mPinRight[2], 1);
    }

    //Motor Right 2

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command

      analogWrite(mPinRight[5], 0);
      digitalWrite(mPinRight[3], 0);
      digitalWrite(mPinRight[4], 0);

    }
    else if (R2disable == 1) {

      analogWrite(mPinRight[5], 0);
      digitalWrite(mPinRight[3], 0);
      digitalWrite(mPinRight[4], 0);

    }
    else if (speed_req_right == 0) {                       //Stopping

      analogWrite(mPinRight[5], 0);
      digitalWrite(mPinRight[3], 0);
      digitalWrite(mPinRight[4], 0);
    }
    else if (speed_req_right > 0) {                         //Going forward

      analogWrite(mPinRight[5], abs(PWM_rightMotor2));
      digitalWrite(mPinRight[3], 1);
      digitalWrite(mPinRight[4], 0);

    }
    else {                                               //Going backward
      analogWrite(mPinRight[5], abs(PWM_rightMotor2));
      digitalWrite(mPinRight[3], 0);
      digitalWrite(mPinRight[4], 1);
    }


    //----------------------------------

    if ((millis() - lastMilli) >= LOOPTIME) {     //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    //----------------------------------

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }

    publishSpeed(LOOPTIME);

  }

}
//---------------------------------------------------------------------------------------------------------------------

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = (speed_act_left1 + speed_act_left1) / 2;  //left wheel speed (in m/s)
  speed_msg.vector.y = (speed_act_right1 + speed_act_right1) / 2 ; //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}


//----------------------------------------------------------------------------------------------------------------------

//Left motor encoder counter
void encoderLeftMotor1() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT1) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT1)) pos_left1--;
  else pos_left1++;
  //Serial.println(pos_left1);
}

//Right motor encoder counter
void encoderRightMotor1() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT1) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT1)) pos_right1--;
  else pos_right1++;
}

void encoderLeftMotor2() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT2) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT2)) pos_left2--;
  else pos_left2++;
}

//Right motor encoder counter
void encoderRightMotor2() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT2) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT2)) pos_right2--;
  else pos_right2++;
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
