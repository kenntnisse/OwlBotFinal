/*
Last Editor(s): Shunzo Hida
Last Edit: 20:49 UTC-7 11/27/2023
Language: Arduino
Board: Arduino Nano
*/

/*
Description:
 * Header file for the library for the Elegoo Owl Smart Robot.
 * Utilizes two TB6612 motors and a gyro/accelorometer MPU6050.
 * Currently, can calculate rotation and pseudo displacement
   * Displacement is in millisecond * analog power to motor * battery format.
   * Rotation should be accurate, with a calibration threshold of 3 degrees
 * TODO: do PID control (instead of abruptly changing power to motors)
*/

/*
User Guide:
 * The array orders[n] holds commands from 0-4.
   * You will have to change n to the number of commands.
 * Alter the distance it will travel, either by executing the setDistance(int) function
	 or directly altering the distance int.
*/
#include "arduino.h"
#include "Wire.h"
class OwlBot
{
private:
	const int MPU = 0x68; // MPU6050 (accelorometer and gyroscope)
	int curA = 0;		  // current speeds of motors.
	int curB = 0;
	const int errorTimes = 200; // used to calibrate
	const int GYRO = 0x43;
	const int ACC = 0x3B;
	double X, Y, Z;
	double AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ, AccGyroErrorX, AccGyroErrorY;
	int defSpd = 100;
  int defPause = 1000;
	double gyroAngleX = 0.0;
	double gyroAngleY = 0.0;
	double prevTime = millis();
	double curTime = millis();
	int battery = 0;
	long distance = 50000; // default distance for spd of motor * millisecond * battery power
#define KILL 0
#define FORWARD 1
#define BACK 2
#define RIGHT 3
#define LEFT 4
#define PAUSE 5
	int orders[7] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
		1, 3, 1, 4, 5, 1, 0
	};
	int curCommand = 0; // current step executing
  int adjustDist = 0;
public:
  long pause = 0;
	long long counter = 0;
	double degThreshold = 3.0; // 2.0 makes it jiggle a lot
	int distThreshold = 500;
	double yaw, roll, pitch;
	double position, velocity;
  double deviation = 0;
  bool adjusting = false;
	int STANDBY, POWERA, POWERB, DIRA, DIRB, BATTERY; // pins
	OwlBot(int, int, int, int, int, int);
	OwlBot();
  // initialize
	void wake();

	void setDistance(long long);
	void setDegThreshold(double);
	void setDistThreshold(int);
	void setSpeed(int);
  void setPause(int);
  void setAdjust(int);

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
  void adjust();

  //sends next command
	void next();


  // MPU6050 gyroscope
	void get(int);
	void update();
	void calibrate();


  // ultrasonic sensor (unknown specs)
  unsigned long ultrasonic();
  void ultrasonic(uint64_t *u);
};