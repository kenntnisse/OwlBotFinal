/*
Last Editor(s): Shunzo Hida
Last Edit: 5:37 UTC-7 01/19/2024
Language: Arduino
Board: Arduino Nano
*/

/*
Description:
 * Header file for the library for the Elegoo Owl Smart Robot.
 * Utilizes two TB6612 motors, a gyro/accelorometer MPU6050, and an ultrasonic sensor with unknown specs.
 * Currently, can calculate rotation and pseudo displacement
	* Displacement is in millisecond * analog power to motor * battery format.
	* Rotation should be accurate, with a calibration threshold of 3 degrees
 * TODO: do PID control (instead of abruptly changing power to motors)
*/




#include "arduino.h"
#include "Wire.h"
#include "cppQueue.h"
//#include "Vector.h"



struct Point{
	byte x, y;
	Point(byte a, byte b){
    x = a;
		y = b;
	}
	Point(){
		x = 0;
		y = 0;
	}
	bool operator==(Point const &p){
		return ((x == p.x) && (y == p.y)); 
	}
	bool operator!=(Point const &p){
		return !((x == p.x) && (y == p.y));
	}
};

class OwlBot
{
  #define DEGTORAD PI/180
private:
	const int MPU = 0x68; // MPU6050 (accelorometer and gyroscope)
	int curA = 0;		  // current speeds of motors.
	int curB = 0;
	const int errorTimes = 200; // used to calibrate
	const int GYRO = 0x43;
	const int ACC = 0x3B;
	//double X, Y, Z;
	double AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ, AccGyroErrorX, AccGyroErrorY;
	byte defSpd = 100;
	int defPause = 1000;
	double gyroAngleX = 0.0;
	double gyroAngleY = 0.0;
	int prevTime = millis();
	int curTime = millis();
	byte battery = 0;
	long distance = 50000; // default distance for spd of motor * millisecond * battery power
	int adjustDist = 0;
  int X, Y, Z;
  double turnMod = 1.0;
  bool PID = false;
  bool linearPID = false;
  int uDelay = 1;
  int startBattery;
  bool smoother = false;
public:
  bool turning = false;
  int curDir = 0;
	int pause = 0;
	long long counter = 0;
	double degThreshold = 3.0; // 2.0 makes it jiggle a lot
	int distThreshold = 500;
  int adjSpd = 50;
	double yaw, roll, pitch;
	double position, velocity;
	double deviation = 0;
	bool adjusting = false;
	byte STANDBY, POWERA, POWERB, DIRA, DIRB, BATTERY; // pins
	OwlBot(byte, byte, byte, byte, byte, byte);
	OwlBot();
	// initialize
	void wake();

	void setDistance(long long);
	void setDegThreshold(double);
	void setDistThreshold(int);
	void setSpeed(byte);
	void setPause(int);
	void setAdjust(int);
  void setDir(int);
  void setTurnMod(double);
  void setPID(bool);
  void setLinearPID(bool);
  void setDelay(int);
  void setSmoother(bool);

	//controls motors
	void motorControl(int, int);

	// sends commands to motors based on counter
	void move();

	//alters counter
	void forward();
	void backward();

	// sends commands to motors based on yaw
	void turn();

	//alters yaw
	void left();
	void right();

	// stops and delays for a very long time
	void kill();

	// pause
	void wait();

	// adjust relative to wall in front
  void setAdjSpd(int);
	void adjust();



	// MPU6050 gyroscope
	void get(int);
	void update();
	void calibrate();


	// ultrasonic sensor (unknown specs)
	long ultrasonic();


};





