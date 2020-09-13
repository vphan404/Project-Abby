#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_PWMServoDriver.h>

#define enA 5 // Motors
#define in1 6
#define in2 7

#define frontTrig 10 // Ultrasounds Front
#define frontEcho 9

ros::NodeHandle nh;
std_msgs::Int32MultiArray ultrasonic_ranges;
Adafruit_PWMServoDriver pwm;

long int distances[5];
long last_time_action_done_millis = 0;
float csThrottleValue = 0.0;
float csSteeringValue = 0.0;

void setDriveCommand(const geometry_msgs::Twist& command) {
 csThrottleValue = (float)command.linear.x;
 csSteeringValue = (float)command.angular.z;
}

ros::Publisher ultrasonic_pub("arduino/ultrasonic_ranges", &ultrasonic_ranges);
ros::Subscriber<geometry_msgs::Twist> command_sub("cmd_vel", &setDriveCommand);

void fillRanges() {
  ultrasonic_ranges.data[0] = 1000; //readUltrasound(frontTrig, frontEcho);
  ultrasonic_ranges.data[1] = 1000;
  ultrasonic_ranges.data[2] = 1000;
  ultrasonic_ranges.data[3] = 1000;
  ultrasonic_ranges.data[4] = 1000;
}

void setSteerAngle(float steerAngle) {
  float center = 345.0f;
  float radius = 250.0f;
  int convertedSteerAngle = center + radius * -steerAngle;
  pwm.setPWM(0, 0, convertedSteerAngle);
}

void setThrottleESC(float throttle) { //ESC
  float center = 330.0f;
  float radius = 100.0f;
  int convertedThrottle = center + radius * throttle;
  pwm.setPWM(1, 0, convertedThrottle);
}

int readUltrasound(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) * 0.01715;
}

void setup() {
  ultrasonic_ranges.data_length = 5;
  ultrasonic_ranges.data = distances;

  pwm = Adafruit_PWMServoDriver();
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);

  pinMode(enA, OUTPUT); // Motors
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  pinMode(frontTrig, OUTPUT); // Ultrasound
  digitalWrite(frontTrig, LOW);
  pinMode(frontEcho, INPUT);

  nh.initNode();
  nh.advertise(ultrasonic_pub);

  pwm.setPWM(1, 0, 330);
  delay(1000);
}

void loop() {
  if (millis() > last_time_action_done_millis + 300) {
     last_time_action_done_millis = millis();

     fillRanges();
     ultrasonic_pub.publish( &ultrasonic_ranges );
  }

  setSteerAngle(csSteeringValue);
  setThrottleESC(csThrottleValue);

  nh.subscribe(command_sub);
  nh.spinOnce();
}
