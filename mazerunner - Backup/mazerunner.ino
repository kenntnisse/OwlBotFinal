/*
Last Editor(s): Shunzo Hida
Last Edit: 20:49 UTC-7 12/05/2023
Language: Arduino
Board: Arduino Nano
*/

/* Description:
 * Code for the Elegoo Owl Smart Robot.
 * Utilizes two TB6612 motors and a gyro/accelorometer MPU6050.
 * Currently, can calculate rotation and pseudo displacement
   * Displacement is in millisecond * analog power to motor * battery/1000 format.
   * Rotation should be accurate, with a calibration threshold of 3 degrees
 * TODO: do PID control (instead of abruptly changing power to motors)
   * use ultrasonic to detect walls
 * TODO(hardware): upgrade wheels
*/

/* Issues:
 * Sometimes, the ultrasonic sensor detects 56325 instead of 1500 as the max
   * if 1500, its in mm (max 150 cm)




/* User Guide:
 * The array orders[n] holds commands from 0-4. 
   * You will have to change n to the number of commands.
 * Alter the distance it will travel, either by executing the setDistance(int) function
     or directly altering the distance int.
*/



// full battery: 142(on), 121(off), 16000, ~1.66 v
// used battery: 123(on), 105(off), 17000, ~1.45 v



#include "OwlBot.h"


OwlBot owl;
int targetTime = 60000; // target time (in ms)
long long distance = 16000;
const bool pauseBetweenCommands = true;
int orders[72] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
  6, 1, 4, 1, 1, 3, 1, 1, 1,
  1, 1, 1, 4, 1, 1, 4, 1,
  1, 3, 1, 1, 3, 1, 1, 3,
  3, 1, 1, 4, 1, 1, 4, 1,
  1, 3, 1, 1, 3, 1, 1, 4,
  1, 1, 4, 1, 1, 3, 3, 1,
  1, 3, 1, 1, 4, 1, 1, 4,
  1, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0
};
void setup()
{
	owl.wake();
	delay(20);
  owl.setDistance(distance); // 25 cm
  owl.setDistThreshold(500);
  owl.setDegThreshold(3.0); // 3 degrees
  owl.setSpeed(150); // 115/255
  owl.setPause(10); // 1.1 seconds
  owl.setAdjust(125); // 20 cm away from wall
}

#define KILL 0
#define FORWARD 1
#define BACK 2
#define RIGHT 3
#define LEFT 4
#define PAUSE 5
#define START 6
#define END 7
#define ADJUST 8
#define ENDTURN 9
/*
int orders[72] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
  1, 1, 1, 5, 1, 5, 1, 5,
  1, 5, 1, 5, 1, 5, 1, 4,
  1, 5, 1, 4, 1, 5, 1, 3,
  1, 5, 1, 3, 1, 5, 1, 4,
  1, 5, 1, 4, 1, 5, 1, 3,
  3, 5, 1, 5, 1, 3, 1, 5, 
  1, 0, 1, 5, 1, 5, 1, 4,
  1, 5, 1, 4, 1, 5, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};
*/
/*
int orders[72] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};*/

int curCommand = 0; // current step executing
volatile uint64_t ultra;
void loop()
{
  static bool run = false;
  if (millis() >= targetTime) owl.kill();
  if (!run) if (signal()) run = true;
  //owl.ultrasonic(ultra);
  Serial.print((unsigned long)owl.ultrasonic()); Serial.print("/"); Serial.print(run); Serial.print("/");
	owl.update();
	// owl.motorControl(100,100);

  if (run){ // if the signal has been received
    if (abs(owl.yaw) > owl.degThreshold)
    { // the right direction is THE most important thing
      owl.turn();
    }
    else if (abs(owl.counter) > owl.distThreshold)
    { // position change
      owl.move();
    } else if (owl.pause > 0){ // nothing else
      owl.motorControl(0,0);
    } else if (owl.adjusting){ // look at a wall(if possible)
      owl.adjust();
    }
    else // next command
      next();
  }



}


void next()
{
	// get next command
	owl.motorControl(0, 0);
  if (pauseBetweenCommands) owl.wait();
	switch (orders[curCommand++]) {
    case 0: //kill
      delay(10000000); break;
    case 1: // forward
      owl.forward(); break;
    case 2: // backward
      owl.backward(); break;
    case 3: // turn right
      owl.right(); break;
    case 4: // turn left
      owl.left(); break;
    case 5: // pause
      owl.wait(); break;
    case 6: // bc it starts at the edge
      owl.setDistance(distance/2-10); owl.forward(); owl.setDistance(distance); break;
    case 7: // because it measures distance from center of wheels (may have to adjust for dowel being on side)
      owl.setDistance(distance/2-10); owl.backward(); owl.setDistance(distance); break;
    case 8: // adjust against a wall
      owl.adjusting = true; break;
    default:
      delay(10000000);
	}
  //curCommand++;
}

bool signal(){
  long long d = owl.ultrasonic();
  //owl.ultrasonic(ultra);
  //return (((ultra > 15000 && ultra < 20000) || ultra < 100) && millis() > 2000);
  return (((d > 15000 && d < 20000) || d < 50) && millis() > 2000);
}