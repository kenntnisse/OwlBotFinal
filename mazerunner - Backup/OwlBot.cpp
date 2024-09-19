/*
Last Editor(s): Shunzo Hida
Last Edit: 09:32 UTC-7 11/28/2023
Language: Arduino
Board: Arduino Nano
*/

/*
Description:
* Library for the Elegoo Owl Smart Robot.
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

#include "OwlBot.h"

OwlBot::OwlBot(int a, int b, int c, int d, int e, int f)
{ // should be 4, 9, 6, 8, 7, A3
	STANDBY = a;
	POWERA = b;
	POWERB = c;
	DIRA = d;
	DIRB = e;
	BATTERY = f;
}
OwlBot::OwlBot()
{
	STANDBY = 4;
	POWERA = 9;
	POWERB = 6;
	DIRA = 8;
	DIRB = 7;
	BATTERY = A3;
}
void OwlBot::wake()
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

void OwlBot::setDistance(long long d) {
	distance = d;
}
void OwlBot::setDegThreshold(double d) {
	degThreshold = d;
}
void OwlBot::setDistThreshold(int d) {
	distThreshold = d;
}
void OwlBot::setSpeed(int d) {
	defSpd = d;
}
void OwlBot::setPause(int d){
  defPause = d;
}
void OwlBot::motorControl(int spdA, int spdB)
{													 // right, left

	static long prev = millis();
	static long cur = millis();
	prev = cur;
	cur = millis();
  if (curA == 0 && curB == 0){
    pause -= cur-prev;
  }
	else if (curA == curB) { // only if it was moving in a straight line
    int d = (cur - prev) * battery * curA / 1000;
		counter += d;
    deviation += d*sin(yaw*180/PI);
	} 


	digitalWrite(STANDBY, (spdA != 0 && spdB != 0)); // when no speed, kills the motors.

	digitalWrite(DIRA, (spdA < 0));
	analogWrite(POWERA, abs(spdA));

	digitalWrite(DIRB, (spdB < 0));
	analogWrite(POWERB, abs(spdB));
	curA = spdA;
	curB = spdB;
}

void OwlBot::move()
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
void OwlBot::forward()
{
	counter -= distance;
}
void OwlBot::backward()
{
	counter += distance;
}

void OwlBot::turn()
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

void OwlBot::left()
{ // turning left is positive
	yaw -= 90;
}

void OwlBot::right()
{ // turning right is negative
	yaw += 90;
}

void OwlBot::kill() {
	motorControl(0, 0);
	delay(1000000);
}

void OwlBot::wait(){
  pause += defPause;
}

void OwlBot::adjust(){
  int d = ultrasonic();
  if (abs(d-adjustDist)<30) adjusting = false; // close enough
  if (d < adjustDist){ // too close
    counter+=10;
  } else  // too far
    counter-=10;
  //move(); // act on it
}

void OwlBot::setAdjust(int d){
  adjustDist = d;
}
void OwlBot::next()
{
	// get next command
	motorControl(0, 0);

	switch (orders[curCommand++]) {
	case 0:
		delay(1000000); break;
	case 1:
		forward(); break;
	case 2:
		backward(); break;
	case 3:
		right(); break;
	case 4:
		left(); break;
  case 5:
    wait(); break;
	default:
		delay(1000000);
	}
}

void OwlBot::get(int r)
{ // updates properties with x, y, and z vals, depending on gyro or accel
	Wire.beginTransmission(MPU);
	Wire.write(r);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
	X = (Wire.read()) << 8 | Wire.read(); // X-axis value
	Y = (Wire.read()) << 8 | Wire.read(); // Y-axis value
	Z = (Wire.read()) << 8 | Wire.read(); // Z-axis value
}

void OwlBot::update()
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
	Serial.print((long)counter); Serial.print("/");
	Serial.println(battery);
}

void OwlBot::calibrate()
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

unsigned long OwlBot::ultrasonic()  // ultrasonic sensor (unknown specs), returns distance in mm (sometimes)
{
  unsigned long dat[2] = {0};
  Wire.requestFrom(0x07, 1); //从器件读取一位数
  if (Wire.available() > 0)
  {
    dat[0] = Wire.read();
  }
  Wire.requestFrom(0x07, 1); //从器件读取一位数
  if (Wire.available() > 0)
  {
    dat[1] = Wire.read();
  }
  //Serial.print((dat[0]));
  return ((dat[0] << 8) | dat[1]);
}

void OwlBot::ultrasonic(uint64_t *u){
  unsigned dat[2] = {0};
  Wire.requestFrom(0x07, 1); //从器件读取一位数
  if (Wire.available() > 0)
  {
    dat[0] = Wire.read();
  }
  Wire.requestFrom(0x07, 1); //从器件读取一位数
  if (Wire.available() > 0)
  {
    dat[1] = Wire.read();
  }
  *u = ((dat[0] << 8) | dat[1]);
}
