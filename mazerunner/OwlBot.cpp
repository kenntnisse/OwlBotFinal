/*
Last Editor(s): Shunzo Hida
Last Edit: 5:37 UTC-7 01/19/2024
Language: Arduino
Board: Arduino Nano
*/

/*
Description:
* Library for the Elegoo Owl Smart Robot.
* Utilizes two TB6612 motors, a gyro/accelorometer MPU6050, and an ultrasonic sensor with unknown specs.
* Currently, can calculate rotation and pseudo displacement
	* Displacement is in millisecond * analog power to motor * battery format.
	* Rotation should be accurate, with a calibration threshold of 3 degrees
* TODO: do PID control (instead of abruptly changing power to motors)
*/


#include "OwlBot.h"


OwlBot::OwlBot(byte a, byte b, byte c, byte d, byte e, byte f)
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


// initializes variables, activates wire and pinModes, and calibrates.
void OwlBot::wake()
{
	X=Y=Z=AccErrorX = AccErrorY = AccErrorZ = GyroErrorX = GyroErrorY = GyroErrorZ = AccGyroErrorX = AccGyroErrorY = yaw = roll = pitch = 0.0;
	velocity = position = yaw = roll = pitch = 0.0;

	pinMode(STANDBY, OUTPUT); // used when sending commands to motors
	pinMode(POWERA, OUTPUT);  // spins motor A at a specific speed(right)
	pinMode(POWERB, OUTPUT);  // spins motor B at a specific speed(left)
	pinMode(DIRA, OUTPUT);	  // Determines direction for motor A: HIGH == backward, LOW == forward
	pinMode(DIRB, OUTPUT);	  // Determines direction for motor B: HIGH == backward, LOW == forward
	pinMode(BATTERY, INPUT);  // Analog input for BATTERY voltage
	//Serial.begin(9600);
	//Serial.println(F("Started"));
	Wire.begin();				 // Initialize comunication
	Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
	Wire.write(0x6B);			 // Talk to the register 6B
	Wire.write(0x00);			 // Make reset - place a 0 into the 6B register
	Wire.endTransmission(true);	 // end the transmission
  Wire.requestFrom(0x07,1);
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
void OwlBot::setSpeed(byte d) {
	defSpd = d;
}
void OwlBot::setPause(int d) {
	defPause = d;
}

void OwlBot::setAdjust(int d) {
	adjustDist = d;
}
void OwlBot::setDir(int d){
  curDir = d;
}
void OwlBot::setTurnMod(double d){
  turnMod = d;
}
void OwlBot::setPID(bool b){
  PID = b;
}
void OwlBot::setLinearPID(bool b){
  linearPID = b;
}
// the base movement control
void OwlBot::motorControl(int spdA, int spdB)
{													 // right, left
  if (smoother){
    spdA = (spdA*3+curA*1)/4;
    spdB = (spdB*3+curB*1)/4;
  }
  /*
  if (turning == false && spdA != spdB){
    spdA = (spdA*1+curA*1)/2;
    spdB = (spdB*1+curB*1)/2;
  }*/
  if (turning == false){
    if (yaw < 0) spdB *= 0.95; // drifting right
    else if (yaw > 0) spdA *= 0.95;
  }

	static long prev = millis();
	static long cur = millis();
	prev = cur;
	cur = millis();
	if (curA == 0 && curB == 0) {
		pause -= cur - prev;
	}
	else if (curA == curB || abs(curA-curB)/curA < 0.1) { // only if it was moving in a straight line
		long long d = (cur - prev) * pow(analogRead(BATTERY)/100.0,1.60) * ((curA+spdA+curB+spdB)/4) / 10;
		counter += d;
		deviation += d * sin(yaw*DEGTORAD);
	}


	digitalWrite(STANDBY, (spdA != 0 && spdB != 0)); // when no speed, kills the motors.

	digitalWrite(DIRA, (spdA < 0)); // right
	analogWrite(POWERA, min(abs(spdA), 255));

	digitalWrite(DIRB, (spdB < 0)); // left
	analogWrite(POWERB, min(abs(spdB),255));

	curA = spdA;
	curB = spdB;
}

void OwlBot::move()
{
	if (counter > 0)
	{
    if (linearPID)
		  motorControl(-defSpd * pow((-cos(DEGTORAD*4*90/distance*abs(counter))/4.0+.65), 0.5), -defSpd * pow((-cos(DEGTORAD*4*90/distance*abs(counter))/4.0+.65), 0.5));
    else
		  motorControl(-defSpd, -defSpd);
	}
	else if (counter < 0)
	{
    if (linearPID)
		  motorControl(defSpd * pow((-cos(DEGTORAD*4*90/distance*abs(counter))/4.0+.65), 0.5), defSpd * pow((-cos(DEGTORAD*4*90/distance*abs(counter))/4.0+.65), 0.5));
    else
		  motorControl(defSpd, defSpd);
	}
	else
		motorControl(0, 0);
}
void OwlBot::forward()
{
	counter -= distance;
  turning = false;
}
void OwlBot::backward()
{
	counter += distance;
  turning = false;
}

void OwlBot::turn()
{
	if (yaw < 0)
	{ // right is negative, so turn left to correct
    if (PID) //trig functions are in radians.
		  motorControl(defSpd*turnMod*pow((-.8*cos(DEGTORAD*4*yaw)/4.0+.45),0.5), -defSpd*turnMod*pow((-.8*cos(DEGTORAD*4*yaw)/4.0+.45),0.5));
      //(0.5+.5*(cos(abs(yaw))+sin(abs(yaw)))/sqrt(2)) 
      //min(abs(yaw/90+.5), abs(yaw/-90+1.5))
    else
		  motorControl(defSpd*turnMod, -defSpd*turnMod);
	}
	else if (yaw > 0)
	{ // left is positive, so turn right to correct
    if (PID)
		  motorControl(-defSpd*turnMod*pow((-.8*cos(DEGTORAD*4*yaw)/4.0+.45),0.5), defSpd*turnMod*pow((-.8*cos(DEGTORAD*4*yaw)/4.0+.45),0.5));
       //0.5+log(abs(yaw))/9.0, (0.7+log(abs(yaw))/15.0)
       //(pow(cos(yaw)+sin(yaw), 2)/2+.4)
    else
		  motorControl(-defSpd*turnMod, defSpd*turnMod);
	}
	else
		motorControl(0, 0); // most likely won't be used
}

void OwlBot::left()
{ // turning left is positive
	yaw -= 90;
  turning = true;
}

void OwlBot::right()
{ // turning right is negative
	yaw += 90;
  turning = true;
}

void OwlBot::kill() {
	motorControl(0, 0);
	delay(10000000);
}

void OwlBot::wait() {
	pause += defPause;
}

void OwlBot::setAdjSpd(int s){
  adjSpd = s;
}
void OwlBot::adjust() {
	int d = ultrasonic();
	//if (abs(d - adjustDist) < 20) adjusting = false; // close enough

  counter += (adjustDist-d) * distance/250;
	//if (d < adjustDist) { // too close
	//	counter += adjSpd;
	//}
	//else  // too far
	//	counter -= adjSpd;
	//move(); // act on it
  adjusting = false;
}

void OwlBot::setDelay(int d){
  uDelay = d; // delay before ultrasonic.
}
void OwlBot::setSmoother(bool b){
  smoother=b;
}


void OwlBot::get(int r)
{ // updates properties with x, y, and z vals, depending on gyro or accel
	Wire.beginTransmission(MPU);
	Wire.write(r);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
  //delay(10);
	X = (Wire.read()) << 8 | Wire.read(); // X-axis value
	Y = (Wire.read()) << 8 | Wire.read(); // Y-axis value
	Z = (Wire.read()) << 8 | Wire.read(); // Z-axis value
}

void OwlBot::update()
{
  /*
	Wire.beginTransmission(MPU);
	Wire.write(ACC);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers	// For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
	double AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorX; // X-axis value // in g's
	double AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorY; // Y-axis value
	double AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 - AccErrorZ; // Z-axis value
  */
  get(ACC);
	double AccX = X / 16384.0 - AccErrorX; // X-axis value // in g's
	double AccY = Y / 16384.0 - AccErrorY; // Y-axis value
	double AccZ = Z / 16384.0 - AccErrorZ; // Z-axis value
	// Calculating Roll and Pitch from the accelerometer data
	double accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);		 // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
	double accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI); // AccErrorY ~(-1.58)

	// === Read gyroscope data === //
	prevTime = curTime;							  // Previous time is stored before the actual time read
	curTime = millis();							  // Current time actual time read
	double elapsed = (curTime - prevTime) / 1000.0; // Divide by 1000 to get seconds
  //Serial.println(elapsed*1000);
  /*
	Wire.beginTransmission(MPU);
	Wire.write(GYRO);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
  //delay(10);
	double GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorX; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	double GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorY;
	double GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorZ;
*/
  get(GYRO);
	double GyroX = X / 131.0 - GyroErrorX; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	double GyroY = Y / 131.0 - GyroErrorY;
	double GyroZ = Z / 131.0 - GyroErrorZ;
	// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
	gyroAngleX += GyroX * elapsed; // deg/s * s = deg
	gyroAngleY += GyroY * elapsed;
  static double prevYawChange = 0;
	yaw += (GyroZ * elapsed+prevYawChange)/2;
  prevYawChange = GyroZ * elapsed;
	// Complementary filter - combine acceleromter and gyro angle values
	roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
	pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

	// velocity += (AccY + AccX + AccZ) * 9.81; // g's to m/s^2, then integrate to velocity
	// position += velocity; // integrate velocity to position
	position += 0.5 * (AccY) * 9.81 * pow(elapsed, 2); // not very exact

	battery = analogRead(BATTERY); 
	// Print the values on the serial monitor

	Serial.print(yaw); Serial.print(F("/\n"));
	//Serial.print((long)counter); Serial.print(F("/"));
	//Serial.print(battery); Serial.print(F("/"));
	//Serial.println(deviation);
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
    startBattery += analogRead(BATTERY);
    /*
    Wire.beginTransmission(MPU);
    Wire.write(ACC);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
		// Sum all readings
    
    int AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    int AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    int AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    */
    get(ACC);
    int AccX = X / 16384.0;
    int AccY = Y / 16384.0;
    int AccZ = Z / 16384.0;
		AccErrorX += AccX; //
		AccErrorY += AccY; //
		AccErrorZ += AccZ;
		AccGyroErrorX += ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
		AccGyroErrorY += ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
	}
  startBattery/=errorTimes;
	// Divide the sum by 200 to get the error value
	AccErrorX /= errorTimes;
	AccErrorY /= errorTimes;
	AccErrorZ /= errorTimes;
	AccGyroErrorX /= errorTimes;
	AccGyroErrorY /= errorTimes;
	// Read gyro values 200 times
	for (int i = 0; i < errorTimes; i++)
	{
    /*
    Wire.beginTransmission(MPU);
    Wire.write(GYRO);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);		  // Read 6 registers total, each axis value is stored in 2 registers
		// Sum all readings
		GyroErrorX += ((Wire.read() << 8 | Wire.read()) / 131.0);
		GyroErrorY += ((Wire.read() << 8 | Wire.read()) / 131.0);
		GyroErrorZ += ((Wire.read() << 8 | Wire.read()) / 131.0);
    */
    get(GYRO);
		GyroErrorX += X / 131.0;
		GyroErrorY += Y / 131.0;
		GyroErrorZ += Z / 131.0;
	}
	// Divide the sum by 200 to get the error value
	GyroErrorX /= errorTimes;
	GyroErrorY /= errorTimes;
	GyroErrorZ /= errorTimes;
	// Print the error values on the Serial Monitor
	//Serial.print(F("AccErrorX: ")); Serial.println(AccErrorX);
	//Serial.print(F("AccErrorY: ")); Serial.println(AccErrorY);
	//Serial.print(F("GyroErrorX: ")); Serial.println(GyroErrorX);
	//Serial.print(F("GyroErrorY: ")); Serial.println(GyroErrorY);
	//Serial.print(F("GyroErrorZ: ")); Serial.println(GyroErrorZ);
}

long OwlBot::ultrasonic()  // ultrasonic sensor (unknown specs), returns distance in mm (sometimes)
{
  while (Wire.available() > 0) Wire.read();
  static int switched = -1;
	unsigned long dat[2] = { 0 };
	Wire.requestFrom(0x07, 1); //从器件读取一位数
  delay(uDelay); // can't be immediate
	if (Wire.available() > 0){
	  dat[0] = Wire.read();
	}
  Wire.requestFrom(0x07, 1); //从器件读取一位数
  delay(uDelay);
	if (Wire.available() > 0){
	  dat[1] = Wire.read();
	}
  
	//Serial.print((unsigned long)(dat[1]));
	//Serial.print((dat[0]));	Serial.print(F("/"));
	//Serial.print((dat[1]));	Serial.print(F("/"));
  if (millis()<2000) return 1500;
  if (switched == -1) switched = (dat[0] < dat[1]) ? 0 : 1;
  //Serial.print(((dat[0] << 8) | dat[1]));
  return ((switched == 0) ? ((dat[0] << 8) | dat[1]) : ((dat[1] << 8) | dat[0]));
  //return (dat[0] < dat[1]) ? ((dat[0] << 8) | dat[1]) : ((dat[1] << 8) | dat[0]);
}



//////////
// Pathfinders ahead.
//////////
