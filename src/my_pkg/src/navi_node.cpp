#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <sstream>
using namespace std;

// --- Global Variables ---
// Sensor data and motor control
#define frekv 60
ros::Publisher move_pub;
ros::Subscriber lidar_sub;
ros::Subscriber odom_sub;
ros::Subscriber dpos_sub;
float distanceData[361]; // Lidar data
float currentPos[2]; // The current position
float currentAngle; // The current orientation
float desiredPos[2]; // The desired end position
float desiredAngle; // The desired end orientation
// Phisical parameters
#define distanceError 0.02
#define angleError 0.02
#define size 0.10 // Radius of the robot (burger)
#define aura 0.12 // Safe area around the robot
#define maxRot 1.82 //Datasheet: 1.82 rad/s;
#define maxVel 0.22 //Datasheet: 0.22 m/s;
// Navigation variables
float targetPos[2]; // The current desired position
float previousPos[2]; // The endpoints of the current guideline
float passedtarget; // true if (passedtarget >= 1)
// State machine
int state = 0;
#define idle 0
#define movingToTarget 1
#define turning 2
#define exit 3
// Math constants
#define pi 3.14159
#define deg2rad 0.0174533
#define rad2deg 57.2958
// Controller parameters from Matlab script
#define ka1 0.1599112
#define ka2 0.0063929
float k1 = ka1*frekv;
float k2 = ka2*frekv*frekv;

// --- Functions ---
void init(); // Setting initial values for global variables
void start(); // Resetting the Navigation variables
void move(float vt, float vr); // Moving the robot, vt: foward speed, vr: angular velocity
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg); // Getting data from LiDAR
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg); // Getting the odometry data
void dpos_callback(const geometry_msgs::Point32::ConstPtr& msg); // Getting thr desired position data

float calcSpeed(); // calculating current velocity
int targetClear(float target[2]); // Checking wether the target is approchable (Returns -1 if true !!!)
bool getNewTarget(); // Calculating new target position
void calcTarget(int direction, float distance); // Calculating possible new target position

void turn(); // Turning to desired angle
void goToTarget(); // Moveing to target position with P controllers
void followLine(); // Moveing to target position with controller from Simulink model

float distance(float p1[2], float p2[2]);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navi_node");
	ros::NodeHandle n;
	move_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	lidar_sub = n.subscribe("/scan", 100, laser_callback);
	odom_sub = n.subscribe("/odom", 100, odom_callback);
	dpos_sub = n.subscribe("/dpos", 100, dpos_callback);
	ros::Rate loop_rate(frekv);
	init();
	ros::Duration(1).sleep();
	int count = 0;
	while (ros::ok()) {
		// Calling callback functions
		ros::spinOnce();
		// Logic
		switch (state) {
			case idle:
				if(distance(currentPos, desiredPos) > size) {
					start();
					state = movingToTarget;
				}
				move(0, 0);
				break;
			case movingToTarget:
				if (distance(currentPos, desiredPos) < distanceError) {
					move(0, 0);
					state = turning;
				}
				else {
					if (count % frekv/5 == 0) {
						if (passedtarget >= 1) {
							targetPos[0] = desiredPos[0];
							targetPos[1] = desiredPos[1];
							ROS_INFO("Target Passed");
						}
						getNewTarget();
					}
					followLine();
				}
				break;
			case turning:
				if (abs(desiredAngle - currentAngle) < angleError) {
					move(0, 0);
					state = idle;
					ROS_INFO("ON TARGET");
				}
				turn();
				break;
			case exit:
				ROS_INFO("-+-+-+-");
				move(0, 0);
				ros::shutdown();
			break;
			default:
				ROS_INFO("DEFAULT CASE ERROR");
				state = exit;
		}
		++count;
		// Hopefully not a negative waiting time
		loop_rate.sleep();
		if (count % frekv == 0) {
			ROS_INFO("%d", count/frekv);
		}
	}
	return 0;
}
//----------------------------------------------------------------------------
void init() {
	desiredPos[0] = 0;
	desiredPos[1] = 0;
	desiredAngle = 0;
	currentPos[0] = 0;
	currentPos[1] = 0;
	currentAngle = 0;
	for (int i=0; i<361; i++) distanceData[i] = 3.5;
	previousPos[0] = 0;
	previousPos[1] = 0;


}
void start() {
	previousPos[0] = currentPos[0];
	previousPos[1] = currentPos[1];
	targetPos[0] = desiredPos[0];
	targetPos[1] = desiredPos[1];
	passedtarget = 0;
}
//----------------------------------------------------------------------------
void move(float vt, float vr) {
	geometry_msgs::Twist msg;
	msg.linear.x = vt;
	msg.angular.z = vr;    
	move_pub.publish(msg);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	distanceData[360] = msg->ranges[0];
	for (int i=0; i<360; i++) {
		if (msg->ranges[i] < size) {
			state = exit;
			ROS_INFO("Object too close!");
		}
		distanceData[i] = msg->ranges[i];
	}
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	currentPos[0] = msg->pose.pose.position.x;
	currentPos[1] = msg->pose.pose.position.y;
	currentAngle = 2*atan(msg->pose.pose.orientation.z / msg->pose.pose.orientation.w);
}

void dpos_callback(const geometry_msgs::Point32::ConstPtr& msg) {
	// in terminal: rostopic pub /dpos geometry_msgs/Point32 '{x: 1.0, y: 1.0, z: 3.14}'
	desiredPos[0] = msg->x;
	desiredPos[1] = msg->y;
	desiredAngle = msg->z;
	if (desiredAngle > pi || desiredAngle < -pi) desiredAngle = 0;
}
//----------------------------------------------------------------------------
float calcSpeed() {
	float end = distance(currentPos, desiredPos);
	return fmin(1, end/size)*maxVel;
}

int targetClear(float target[2]) {
	int targetDirection = (int)floor(rad2deg*(atan2(target[1]-currentPos[1], target[0]-currentPos[0]) - currentAngle));
	if (targetDirection < 0) targetDirection = 360 + targetDirection;
	float safeDistance = distance(target, currentPos) + size;
	if (targetDirection < 90) {
		for (int i=targetDirection+270; i<360; i++) {
			if (distanceData[i] < safeDistance) {
				if (sin(deg2rad*abs(targetDirection-i+360))*distanceData[i] < size) return i;
			}
		}
	}
	if (targetDirection > 270) {
		for (int i=0; i<targetDirection-270; i++) {
			if (distanceData[i] < safeDistance) {
				if (sin(deg2rad*(abs(targetDirection-i-360)))*distanceData[i] < size) return i;
			}
		}
	}
	for (int i=max(0, targetDirection-90); i<min(361, targetDirection+90); i++) {
		if (distanceData[i] < safeDistance) {
			if (sin(deg2rad*abs(targetDirection-i))*distanceData[i] < size) return i;
		}
	}
	return -1;
}

bool getNewTarget() {
	// Fiding current target direction
	int obsticleIndex = targetClear(targetPos);
	if (obsticleIndex == -1) return 0;
	float targetDirection = atan2(targetPos[1]-currentPos[1], targetPos[0]-currentPos[0]) - currentAngle;
	int targetIndex = (int)(rad2deg*targetDirection);
	if (targetIndex < 0) targetIndex += 360;
	if (targetIndex > 360) targetIndex -= 360;
	
	// Finding points of obsticle
	bool mark[360]; //part of the obsticle
	for (int i=0; i<360; i++) {
		mark[i] = false;
	}
	vector<int> check;
	check.push_back(obsticleIndex);
	mark[obsticleIndex] = true;
	while (!check.empty()) {
		float a = distanceData[check.back()];
		int alpha = check.back();
		check.pop_back();
		for (int i=0; i<360; i++) {
			if (!mark[i]) {
				float b = distanceData[i];
				int beta = i;
				if ( sqrt(a*a+b*b-2*a*b*cos(deg2rad*(alpha-beta))) < 2*aura) {
					check.push_back(i);
					mark[i] = true;
				}
			}
		}	
	}
	// Finding the edge of obsticle
	int leftSide = -1;
	int rightSide = -1;
	float leftAngle = -1.0;
	float rightAngle = 4.0;
	for (int i=0; i<360; i++) {
		if (mark[i]) {
			float temp = deg2rad*(i-(i>180)*360)+asin(aura/distanceData[i]);
			if (temp > leftAngle) {
				leftAngle = temp;
				leftSide = i;
			}
			temp = deg2rad*(i-(i>180)*360)-asin(aura/distanceData[i]);
			if (temp < rightAngle) {
				rightAngle = temp;
				rightSide = i;
			}
		}
	}
	if (leftAngle > 2*pi && rightAngle < 0) {
		ROS_INFO("No way to get to target");
		state = exit;
		return 1;
	}
	// Create new target
	int leftDiff = min(abs(leftSide-targetIndex), 360-abs(leftSide-targetIndex));
	int rightDiff = min(abs(rightSide-targetIndex), 360-abs(rightSide-targetIndex));
	int newDirection;
	float newDistance;
	if (leftDiff < rightDiff) {
		newDirection = leftSide + (int)ceil(rad2deg*asin(aura/distanceData[leftSide]));
		if (newDirection > 360) newDirection -= 360;
		newDistance = sqrt(distanceData[leftSide]*distanceData[leftSide]-aura*aura)+aura;
	}
	else {
		newDirection = rightSide - (int)ceil(rad2deg*asin(aura/distanceData[rightSide]));
		if (newDirection < 0) newDirection += 360;
		newDistance = sqrt(distanceData[rightSide]*distanceData[rightSide]-aura*aura)+aura;
	}
	calcTarget(newDirection, newDistance);
	return 1;
}

void calcTarget(int direction, float distance) {
	float angle = (float)direction*deg2rad;
	float cCP = cos(currentAngle);
	float sCP = sin(currentAngle);
	float xRel = cos(angle)*distance;
	float yRel = sin(angle)*distance;
	previousPos[0] = currentPos[0];
	previousPos[1] = currentPos[1];
	targetPos[0] = currentPos[0] + (cCP*xRel-sCP*yRel);
	targetPos[1] = currentPos[1] + (sCP*xRel+cCP*yRel);
}
//----------------------------------------------------------------------------
void turn() {
	float angleP = 2;
	float angleDiff = desiredAngle - currentAngle;
	if (angleDiff > pi) angleDiff -= 2*pi;
	if (angleDiff < -1*pi) angleDiff += 2*pi;
	move(0, fmin(maxRot, angleP*angleDiff));
}

void goToTarget() {
	float currentVel = calcSpeed();
	float velP = 1;
	float angleP = 2;
	float angleDiff = atan2(targetPos[1]-currentPos[1], targetPos[0]-currentPos[0]) - currentAngle;
	if (angleDiff > pi) angleDiff -= 2*pi;
	if (angleDiff < -1*pi) angleDiff += 2*pi;
	float dist = distance(desiredPos, currentPos);
	move(fmin(currentVel, velP*dist), fmin(maxRot, angleP*angleDiff));
}

void followLine() {
	float teta = atan2(targetPos[1]-previousPos[1], targetPos[0]-previousPos[0]);
	float fi = currentAngle - teta;
	if (fi > pi) fi = fi-2*pi;
	if (fi < -pi) fi = fi+2*pi;
	float cvx = currentPos[0]-previousPos[0];
	float cvy = currentPos[1]-previousPos[1];
	float tvx = targetPos[0]-previousPos[0];
	float tvy = targetPos[1]-previousPos[1];
	float h = -sin(teta)*cvx+cos(teta)*cvy;
	float s2 = tvx*tvx+tvy*tvy;
	float x = cvx*tvx+cvy*tvy;
	if (s2 != 0) passedtarget = x/s2;
	else passedtarget = 2;
	float vt = calcSpeed();
	if (fi*copysign((float)1, h) > pi/4) vt = 0.01;
	float vr;
	vr = k1*fi+k2/vt*h;
	if (vr > maxRot) vr = maxRot;
	if (vr < -1*maxRot) vr = -1*maxRot;
	move(vt, -1*vr);
}
//----------------------------------------------------------------------------
float distance(float p1[2], float p2[2]) {
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]));
}
