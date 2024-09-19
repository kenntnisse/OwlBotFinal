/*
Last Editor(s): Shunzo Hida
Last Edit: 20:49 UTC-7 11/27/2023
Language: Arduino
Board: Arduino Nano
*/

/*
Description:
 * Code for the Elegoo Owl Smart Robot.
 * Utilizes two TB6612 motors and a gyro/accelorometer MPU6050.
 * Currently, can calculate rotation and pseudo displacement
   * Displacement is in millisecond * analog power to motor * battery format.
   * Rotation should be accurate, with a calibration threshold of 3 degrees
 * TODO: format code to a library, do PID control (instead of abruptly changing power to motors)
*/

/*
User Guide:
 * The array orders[n] holds commands from 0-4. 
   * You will have to change n to the number of commands.
 * Alter the distance it will travel, either by executing the setDistance(int) function
     or directly altering the distance int.
*/

// #include "arduino.h"
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
	double gyroAngleX = 0.0;
	double gyroAngleY = 0.0;
	double prevTime = millis();
	double curTime = millis();
	int battery = 0;
	int distance = 1000000; // default distance for spd of motor * millisecond * battery power
	#define KILL 0
	#define FORWARD 1
	#define BACK 2
	#define RIGHT 3
	#define LEFT 4
	int orders[6] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
		1, 3, 1, 4, 1, 0
	};
	int curCommand = 0; // current step executing
public:
	long counter = 0;
	double degThreshold = 3.0; // 2.0 makes it jiggle a lot
	int distThreshold = 500;
	double yaw, roll, pitch;
	double position, velocity;
	int STANDBY, POWERA, POWERB, DIRA, DIRB, BATTERY; // pins
	OwlBot(int a, int b, int c, int d, int e, int f)
	{ // should be 4, 9, 6, 8, 7, A3
		STANDBY = a;
		POWERA = b;
		POWERB = c;
		DIRA = d;
		DIRB = e;
		BATTERY = f;
	}
	OwlBot()
	{
		STANDBY = 4;
		POWERA = 9;
		POWERB = 6;
		DIRA = 8;
		DIRB = 7;
		BATTERY = A3;
	}
	void wake()
	{
		X = Y = Z = AccErrorX = AccErrorY = AccErrorZ = GyroErrorX = GyroErrorY = GyroErrorZ = AccGyroErrorX = AccGyroErrorY = yaw = roll = pitch = 0.0;
		velocity = position = yaw = roll = pitch = 0.0;

		pinMode(STANDBY, OUTPUT); // used when sending commands to motors
		pinMode(POWERA, OUTPUT);  // spins motor A at a specific speed(right)
		pinMode(POWERB, OUTPUT);  // spins motor B at a specific speed(left)
		pinMode(DIRA, OUTPUT);	  // Determines direction for motor A: HIGH == backward, LOW == forward
		pinMode(DIRB, OUTPUT);	  // Determines direction for motor B: HIGH == backward, LOW == forward
		pinMode(BATTERY, INPUT);  // Analog input for BATTERY voltage
		Serial.begin(9600);
		Serial.println("Started");
		Wire.begin();				 // Initialize comunication
		Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
		Wire.write(0x6B);			 // Talk to the register 6B
		Wire.write(0x00);			 // Make reset - place a 0 into the 6B register
		Wire.endTransmission(true);	 // end the transmission

		calibrate();
		delay(20);
	}

	void setDistance(long d){
		distance = d;
	}
	void setDegThreshold(double d){
		degThreshold = d;
	}
	void setDistThreshold(int d){
		distThreshold = d;
	}
	void setSpeed(int d){
		defSpd = d;
	}
	void motorControl(int spdA, int spdB)
	{													 // right, left
		static long prev = millis();
		static long cur = millis();
		prev = cur;
		cur = millis();

		if (curA == curB){ // only if it was moving in a straight line
			counter += (cur - prev) * battery * curA/1000;
		}


		digitalWrite(STANDBY, (spdA != 0 && spdB != 0)); // when no speed, kills the motors.

		digitalWrite(DIRA, (spdA < 0));
		analogWrite(POWERA, abs(spdA));

		digitalWrite(DIRB, (spdB < 0));
		analogWrite(POWERB, abs(spdB));

		curA = spdA;
		curB = spdB;
	}

	void move()
	{
		if (counter > 0)
		{
			motorControl(-defSpd, -defSpd);
		}
		else if (counter < 0)
		{
			motorControl(defSpd, defSpd);
		}
		else
			motorControl(0, 0);
	}
	void forward()
	{
		counter -= distance;
	}
	void backward()
	{
		counter += distance;
	}

	void turn()
	{
		if (yaw < 0)
		{ // right is negative, so turn left to correct
			motorControl(defSpd, -defSpd);
		}
		else if (yaw > 0)
		{ // left is positive, so turn right to correct
			motorControl(-defSpd, defSpd);
		}
		else
			motorControl(0, 0); // most likely won't be used
	}

	void left()
	{ // turning left is positive
		yaw -= 90;
	}

	void right()
	{ // turning right is negative
		yaw += 90;
	}

  void kill(){
    motorControl(0,0);
    delay(100000);
  }
	void next()
	{
		// get next command
		motorControl(0, 0);

		switch (orders[curCommand++]) {
		case 0:
			delay(10000); break;
		case 1:
			forward(); break;
		case 2:
			backward(); break;
		case 3:
			right(); break;
		case 4:
			left(); break;
		default:
			delay(10000);
		}
	}

	void get(int r)
	{ // updates properties with x, y, and z vals, depending on gyro or accel
		Wire.beginTransmission(MPU);
		Wire.write(r);
		Wire.endTransmission(false);
		Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
		X = (Wire.read()) << 8 | Wire.read(); // X-axis value
		Y = (Wire.read()) << 8 | Wire.read(); // Y-axis value
		Z = (Wire.read()) << 8 | Wire.read(); // Z-axis value
	}

	void update()
	{
		get(ACC);
		// For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
		double AccX = X / 16384.0 - AccErrorX; // X-axis value // in g's
		double AccY = Y / 16384.0 - AccErrorY; // Y-axis value
		double AccZ = Z / 16384.0 - AccErrorZ; // Z-axis value
		// Calculating Roll and Pitch from the accelerometer data
		double accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);		 // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
		double accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI); // AccErrorY ~(-1.58)

		// === Read gyroscope data === //
		prevTime = curTime;							  // Previous time is stored before the actual time read
		curTime = millis();							  // Current time actual time read
		double elapsed = (curTime - prevTime) / 1000; // Divide by 1000 to get seconds

		get(GYRO);
		double GyroX = X / 131.0 - GyroErrorX; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
		double GyroY = Y / 131.0 - GyroErrorY;
		double GyroZ = Z / 131.0 - GyroErrorZ;

		// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
		gyroAngleX += GyroX * elapsed; // deg/s * s = deg
		gyroAngleY += GyroY * elapsed;
		yaw += GyroZ * elapsed;
		// Complementary filter - combine acceleromter and gyro angle values
		roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
		pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

		// velocity += (AccY + AccX + AccZ) * 9.81; // g's to m/s^2, then integrate to velocity
		// position += velocity; // integrate velocity to position
		position += 0.5 * (AccY) * 9.81 * pow(elapsed, 2); // not very exact

		battery = analogRead(BATTERY); // 0-1023, maybe use in conjunction with a timer to determine distance
		// Print the values on the serial monitor

		Serial.print(yaw); Serial.print("/");
		Serial.print(counter); Serial.print("/");
    Serial.println(battery);
	}

	void calibrate()
	{
		// yaw = 0.0;
		// roll = 0.0;
		// pitch = 0.0;
		//  We can call this funtion in the setup section to calculate the accelerometer and gyro data error.
		//  From here we will get the error values used in the above equations printed on the Serial Monitor.
		//  Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
		//  Read accelerometer values errorTimes times
		for (int i = 0; i < errorTimes; i++)
		{
			get(ACC);
			double AccX = X / 16384.0;
			double AccY = Y / 16384.0;
			double AccZ = Z / 16384.0;
			// Sum all readings
			AccErrorX += AccX; //
			AccErrorY += AccY; //
			AccErrorZ += AccZ;
			AccGyroErrorX += ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
			AccGyroErrorY += ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
		}
		// Divide the sum by 200 to get the error value
		AccErrorX /= errorTimes;
		AccErrorY /= errorTimes;
		AccErrorZ /= errorTimes;
		AccGyroErrorX /= errorTimes;
		AccGyroErrorY /= errorTimes;
		// Read gyro values 200 times
		for (int i = 0; i < errorTimes; i++)
		{
			get(GYRO);
			// Sum all readings
			GyroErrorX += (X / 131.0);
			GyroErrorY += (Y / 131.0);
			GyroErrorZ += (Z / 131.0);
		}
		// Divide the sum by 200 to get the error value
		GyroErrorX /= errorTimes;
		GyroErrorY /= errorTimes;
		GyroErrorZ /= errorTimes;
		// Print the error values on the Serial Monitor
		Serial.print("AccErrorX: ");
		Serial.println(AccErrorX);
		Serial.print("AccErrorY: ");
		Serial.println(AccErrorY);
		Serial.print("GyroErrorX: ");
		Serial.println(GyroErrorX);
		Serial.print("GyroErrorY: ");
		Serial.println(GyroErrorY);
		Serial.print("GyroErrorZ: ");
		Serial.println(GyroErrorZ);
	}
};

OwlBot owl;
int targetTime = 15000; // target time (in ms)
void setup()
{
	owl.wake();
	delay(20);
}

void loop()
{
  if (millis() >= targetTime) owl.kill();
	owl.update();
	// owl.motorControl(100,100);

	if (abs(owl.yaw) > owl.degThreshold)
	{
		owl.turn();
	}
	else if (abs(owl.counter) > owl.distThreshold)
	{
		owl.move();
	}
	else
		owl.next();
}