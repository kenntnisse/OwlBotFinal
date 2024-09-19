
/*
Last Editor(s): Shunzo Hida
Last Edit: 5:37 UTC-7 01/19/2024
Language: Arduino
Board: Arduino Nano
*/

/* Description:
 * Code for the Elegoo Owl Smart Robot.
 * Utilizes two TB6612 motors, a gyro/accelorometer MPU6050, and an ultrasonic sensor with unknown specs.
 * Currently, can calculate rotation and pseudo displacement
   * Displacement is in millisecond * analog power to motor * battery/1000 format.
   * Rotation should be accurate, with a calibration threshold of 3 degrees
 * TODO: do PID control (instead of abruptly changing power to motors)
   * use ultrasonic to detect walls
   * 
 * TODO(hardware): upgrade wheels
*/

/* Issues:
 * Sometimes, the ultrasonic sensor detects 56325 instead of 1500 as the max
   * if 1500, its in mm (max 150 cm)
   * Likely, it is reversing the inputs 
     * correct: 5 << 8 | 220
     * incorrect: 220 << 8 | 5
   * Fixed.
 * Description & user guide out of date
*/



/* User Guide:
 * Input orders into the int array orders[72]
 * alter default distances in setup() with all the set() functions
 * Wave pencil in front of sensor to start it, or wait some time.
 * It auto corrects in this order: angle, distance, pause, adjust, then next command
*/



// full battery: 142(on), 121(off), 16000, ~1.66 v
// used battery: 123(on), 105(off), 17000, ~1.45 v



#include "OwlBot.h"


//OwlBot owl; // the robot


//command descriptions
#define KILL 0 // 0 speed & extra long delay
#define FORWARD 1 // drive forward
#define BACK 2 // drive backward
#define RIGHT 3 // rotate right
#define LEFT 4 // rotate left
#define PAUSE 5 // pause
#define START 6 // use to place center of rotation on the correct spot(since dowel starts on point)
#define END 7 // plan to use to microadjust
#define ADJUST 8 // to move relative to the wall
#define ENDTURN 9 // turning might make the dowel more accurate

cppQueue orders(sizeof(byte), 150, FIFO);
//byte blah[100];
/*
const byte orders[72] = { // 0 = stop (kill), 1 = forward, 2 = back, 3 = right, 4 = left
  6, 1, 1, 1, 1, 1, 1, 1,
  4, 1, 1, 1, 1, 1, 1, 4,
  1, 1, 1, 1, 1, 1, 4, 1,
  1, 1, 1, 4, 1, 1, 1, 1,
  4, 1, 1, 4, 1, 1, 4, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};*/
const Point waypoints[] = { // x, y, should be even (make uneven to cut down on time, since only the dowel needs to enter the gate zone)
  {4,6},{4,2},{2,2},{0,6}, {4,3}
  //{2,6},{2,4},{2,6},{2,4},{2,6},{2,4},{2,6}
  //{6,6}, {2,6}, {0,6},{0,4},{4,0},{5,0}

};
int curCommand = 0;

const bool grid[7][7] = {
// 0  1  2  3  4  5  6
  {0, 0, 0, 0, 0, 0, 0}, // 0
  {1, 1, 0, 1, 0, 1, 0}, // 1
  {0, 0, 0, 1, 0, 1, 0}, // 2
  {0, 1, 0, 1, 0, 1, 0}, // 3
  {0, 1, 0, 1, 0, 1, 0}, // 4
  {0, 1, 0, 1, 0, 1, 0}, // 5
  {0, 0, 0, 1, 0, 1, 0}  // 6
};

/* // copy and paste to reset grid.
const bool grid[7][7] = {
// 0  1  2  3  4  5  6
  {0, 0, 0, 0, 0, 0, 0}, // 0
  {0, 1, 0, 1, 0, 1, 0}, // 1
  {0, 0, 0, 0, 0, 0, 0}, // 2
  {0, 1, 0, 1, 0, 1, 0}, // 3
  {0, 0, 0, 0, 0, 0, 0}, // 4
  {0, 1, 0, 1, 0, 1, 0}, // 5
  {0, 0, 0, 0, 0, 0, 0}  // 6
};
*/
OwlBot owl;
const long long targetTime = 20000; // target time (in ms)
const long distance = 16500; //14500 analogRead ^1.6 true linPID
const int pause = 100;
const bool pauseBetweenCommands = true; // use to lengthen time
const bool debug = true;
const int totalPause = 0;
int turns = 0;
int forwards = 1;
const int speed = 255;

const int turnTime = 650;
const int moveTime = 255.0/speed * 750; //950 at 200 spd, 750 at 255
void setup()
{
  pinMode(2, INPUT);
  if (debug) Serial.begin(9600);
	owl.wake(); // initialize
  owl.setDistance(distance); // try to make it 25 cm
  owl.setDistThreshold(distance/20); // when it tries to correct linear position
  owl.setDegThreshold(2.5); // 3 degrees, then it corrects
  owl.setSpeed(speed); // speed of motors, goes from 0-255 (but anything under ~90 might not have enough strength)
  owl.setPause(pause); // in milliseconds
  owl.setAdjust(210); // distance from ultrasonic sensor
  owl.setDir(0); //north east south west = 0 1 2 3
  owl.setTurnMod(0.68);// turning is slippery, gyroscope is better when slow
  owl.setPID(true); // turn slower when smaller angle
  owl.setLinearPID(true); // when false, set dist to 18000, speed to 200
  owl.setDelay(1);
  owl.setSmoother(false);
  byte temp;
  /* 
  temp = 1;
  for (int i = 0; i < 2; i++){
    orders.push(&temp);
  } 
  //temp = 3;
  //for (int i = 0; i < 8; i++) orders.push(&temp);
  curCommand=50;
  //*/
  //*
  temp = 6; // to align center of rotation
  orders.push(&temp);
  temp = 1;
  //for(int i = 0; i < 20; i++)
  orders.push(&temp);
  for (int i = 1; i < (sizeof(waypoints)/sizeof(Point)); i++){
    genDir(pathfind(waypoints[i-1], waypoints[i]));
  }
  curCommand = 50;
  temp = 7;
  orders.push(&temp);
  //orders.push(&temp);
  
  /*
  temp = 6;
  orders.push(&temp);
  for (int i = 0; i < 3; i++){
    temp = 1;
    for (int j = 0; j < 3; j++) orders.push(&temp);
    temp = 3;
    orders.push(&temp);
    orders.push(&temp);
    temp = 1;
    for (int j = 0; j < 3; j++) orders.push(&temp);
    temp = 4;
    orders.push(&temp);
    orders.push(&temp);
  }
  /*byte a = 1;
  byte b = 2;
  orders.push(&a);
  orders.push(&b);*/
  //owl.genDir(&orders, owl.pathfind(grid, waypoints[0], waypoints[1]));
  //temp = 8; //testing adjust
  //orders.push(&temp);
  //genDir(pathfind(waypoints[0], waypoints[1]));


}


/*
int orders[72] = { // copy/paste to reset commands
  6, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};*/

void loop()
{
  //Serial.println("working...");
  static bool run = false;
  static long long startTime = 0;
  //if (millis() >= targetTime) owl.kill(); // if target time is reached, kills bot
  if (!run) if (signal()){
    run = true;
    startTime = millis();
    calcPause(startTime);
  } // run

	owl.update(); // update direction, location, etc.

  if (run){ // if the signal has been received
    //if (owl.ultrasonic() < 200 && millis()-startTime > 2000){ // so that the trigger doesn't interfere
    //  owl.adjust();
    //}
    if (abs(owl.yaw) > owl.degThreshold)
    { // the right direction is THE most important thing
      //Serial.println("turning!");
      owl.turn();
    }
    else if (abs(owl.counter) > owl.distThreshold)
    { // position change
      //Serial.println("moving!");
      owl.move();
    } else if (owl.pause > 0){ // nothing else
      //Serial.println("waiting!");
      owl.motorControl(0,0);
    } else if (owl.adjusting){ // look at a wall(if possible)
      //Serial.println("adjusting!");
      owl.adjust();
    }
    else // next command
      next();
  }
}

void next()
{
	// get next command
	//owl.motorControl(0, 0);
  if (pauseBetweenCommands) owl.wait(); // to lengthen time
  byte command;
  if (!orders.pop(&command)){
    if (curCommand+1 >= sizeof(waypoints)/sizeof(Point)) {owl.kill();}
    genDir(pathfind(waypoints[curCommand], waypoints[curCommand+1]));
    curCommand++;
    if (curCommand+1 == sizeof(waypoints)/sizeof(Point)){
      byte temp = 6;
      orders.push(&temp);
    }
    if (!orders.pop(&command)) {
      owl.kill();
    }
  }
	switch (command) { // look at next order, activate it, then increment
    case 0: //kill
      owl.kill(); break;
    case 1: // forward
      owl.setDistance(distance); owl.forward(); break;
    case 2: // backward
      owl.setDistance(distance); owl.backward(); break;
    case 3: // rotate right
      owl.right(); break;
    case 4: // rotate left
      owl.left(); break;
    case 5: // pause
      owl.wait(); break;
    case 6: // bc it starts at the edge
      owl.setDistance(distance*10.0/25); owl.forward(); break;
    case 7: // because it measures distance from center of wheels (may have to adjust for dowel being on side)
      owl.setDistance(distance*19.0/25); owl.forward(); break;
    case 8: // adjust against a wall
      owl.adjusting = true; break;
    default:
      owl.kill();
	}
}



bool signal(){ // if ready to begin
  static bool read = false;
  if (digitalRead(2) == HIGH && millis()>2000) read = true;
  return (millis()> 2000 && digitalRead(2) == LOW && read);
  int d = (owl.ultrasonic()+owl.ultrasonic())/2; // distance from ultrasonic sensor
  return (d < 30 && millis() > 3000); // if something is sensed, and 10s has passed
}




void genDir(cppQueue path){
	enum DIRS {NORTH, EAST, SOUTH, WEST};
  Point prevP, curP;
  path.pop(&prevP);
	while (path.pop(&curP)){
    int newDir;
		if (curP.y-prevP.y == 1) newDir = SOUTH;
		else if (curP.y-prevP.y == -1) newDir = NORTH;
		else if (curP.x-prevP.x == 1) newDir = EAST;
		else if (curP.x-prevP.x == -1) newDir = WEST;
		int rotations = (newDir-owl.curDir); // accessing a field like this is bad.
		if (rotations == -3) rotations += 4;
		else if (rotations == 3) rotations -= 4;
    int x = 0; int y = 0;
    switch (owl.curDir){ // left
      case (SOUTH):
        x++; break;
      case (NORTH):
        x--; break;
      case (EAST):
        y--; break;
      case (WEST):
        y++; break;
      default:
        break;
    }
    if (rotations == -2){ // turning left
      Point left(prevP.x+x, prevP.y+y);
      if (!valid(left) || 
         (left.y < 0 || left.y >= 7 || // edges are never walls
			    left.x < 0 || left.x >= 7)){ // if left is a wall, turn right instead
        rotations = 2;
      }
    } else if (rotations == 2){ // turning right
      // if right is a wall, turn left instead
      Point right(prevP.x-x, prevP.y-y);
      if (!valid(right) || 
         (right.y < 0 || right.y >= 7 || // edges are never walls
			    right.x < 0 || right.x >= 7)){ // if left is a wall, turn right instead
        rotations = -2;
      }
    }
		if (rotations < 0){
			for (int j = 0; j > rotations; j--){
        byte temp = 4;
        turns++;
				orders.push(&temp); // turn counterclockwise
			}
		} else if (rotations > 0){
			for (int j = 0; j < rotations; j++){
        byte temp = 3;
        turns++;
				orders.push(&temp); // turn clockwise (3)
			}
		}
		owl.curDir = newDir;
    byte temp = 1;
    forwards++;
		orders.push(&temp); // move forward
    prevP = curP;
	}
}



bool valid(Point node){
	return (!grid[node.y][node.x] && 
			node.y >= 0 && node.y < 7 &&
			node.x >= 0 && node.x < 7);
}
cppQueue pathfind(Point start, Point goal){
	bool visited[7][7] = {0};
	Point parent[7][7];
	cppQueue q(sizeof(Point), 50, FIFO);
  Point g = goal;
	q.push(&g);
	visited[goal.y][goal.x] = true;

	const int dx[4] = {0, 0, 1, -1};
	const int dy[4] = {1, -1, 0, 0};
	while (!q.isEmpty()){
    Point cur;
		q.pop(&cur);
		for (int i = 0; i < 4; i++){
			Point neighbor(cur.x+dx[i], cur.y+dy[i]);

      //Serial.print(F("Encountered: ")); Serial.print(neighbor.x); Serial.print(F(", ")); Serial.println(neighbor.y);
      //Serial.print(F("  Valid: ")); Serial.println(valid(neighbor)); Serial.print(F("  Visited: ")); Serial.println(visited[neighbor.y][neighbor.x] ? "true":"false");
      //Serial.print(F("  Grid: ")); Serial.println(grid[neighbor.y][neighbor.x]);
			if (valid(neighbor) && !visited[neighbor.y][neighbor.x]){
        //Serial.print(F("  Added: ")); Serial.print(neighbor.x); Serial.print(F(", ")); Serial.println(neighbor.y);

				visited[neighbor.y][neighbor.x] = true;
				parent[neighbor.y][neighbor.x] = cur;
				q.push(&neighbor);
			}
		}
	}
  cppQueue path(sizeof(Point), 20, FIFO);
	//Vector<Point> path;
  //Point storage[10];
  //path.setStorage(storage);

	for (Point cur = start; cur != goal; cur = parent[cur.y][cur.x]){
		path.push(&cur);
    //Serial.print(cur.x); Serial.print(F(", ")); Serial.println(cur.y);
	}
  path.push(&goal);
  //Serial.print(goal.x); Serial.print(F(", ")); Serial.println(goal.y);
/*
	for (int i = 0; i < path.size(); i++){
		//cout << p.y << ", " << p.x << endl;
    Serial.print(path[i].x); Serial.print(F(", ")); Serial.println(path[i].y);
	}*/

  /*
  extern int __heap_start,*__brkval;
  int v;
  Serial.println((int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval));  
  */
  //return cppQueue(sizeof(byte), 1, FIFO);
  //return genDir(path);
  //genDir(&orders, path);
  return path;
}



void calcPause(int startTime){
  long long nOrders = max(orders.getCount(), 1);//or getRemainingCount();
  long long remTime = max(targetTime-turns*turnTime-forwards*moveTime, 0);
  owl.setPause(remTime/nOrders);
  //Serial.println((int)(remTime/nOrders));
  //Serial.println(turns*turnTime);
  //Serial.println(forwards*moveTime);
  //Serial.println((int)remTime);
}