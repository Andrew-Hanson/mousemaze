//Authors: Gonçalo Azinheira and João Castro, students at University of  the Algarve(Ualg);
//Project: Mazemaster3000;
//Subject: A finite state machine-powered  maze solver. The starting point of the maze must be in one of the left corners and  the ending must be in one of the right corners.

//----------------------------------------------------------------------------
#include "NavigationController.c"

// sensor constants
#define LEFT_ECHO     7
#define LEFT_TRIGGER  6
#define LEFT_R        1.01325
#define LEFT_OFFSET   -1.549

#define FRONT_ECHO    4
#define FRONT_TRIGGER 5
#define FRONT_R       0.982
#define FRONT_OFFSET  -0.732

#define RIGHT_ECHO    2
#define RIGHT_TRIGGER 10
#define RIGHT_R       1.00
#define RIGHT_OFFSET  0.15

// sensor variables
double rightdist = 0;
double leftdist = 0;
double frontdist = 0;
int dir;

// motor constants
int In1 = 8;   //right      // Modified - From A5 to 8    Brake - HIGH == on, LOW == off
int In2 = 13;  //right      // Modified - From A4 to 13   Direction - Low == reverse, HIGH == forward
int EnA = 11;  //right      // Modified - From 10 to 11   PWM

int In3 = 9;   //left       // Modified - From A3 to 9    Brake - HIGH == on, LOW == off     
int In4 = 12;  //left       // Modified - From A2 to 12   Direction - Low == forward, HIGH == reverse
int EnB = 3;   //left       //  Modified - From 11 to 3   PWM

int esqdist;
int dirdist;
int fdist;


void setup() {
  // HC-05 default baud rate
  Serial.begin(38400);
  //button=0;


  // Motor config
  pinMode(In1,  OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA,  OUTPUT);
  pinMode(EnB, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  analogWrite(EnA,  0);
  analogWrite(EnB, 0);


  // Sensor Distances
  rightdist = 0.0;
  leftdist = 0.0;
  frontdist = 0.0;

  // Help to Serial
  Serial.println("************************");
  Serial.println("Accepted Values: ");
  Serial.println("Turn Right:  1");
  Serial.println("Go Straight: 2");
  Serial.println("Turn Left:   3");
  Serial.println("Turn Around: 4");
  Serial.println("************************");
}


//-----------------------------------------------
//**********************************
// Stafford
// function to get distant of nearest wall to given sensor
// pinTrig - pin to activate sensor
// pinEcho - ADC value for time taken to 
// R - in text file
// offset - in text file
double getDistance(int pinTrig, int pinEcho, double R, double offset) {
  double distance;
  double duration;
  
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  // max range = ~20ms
  duration = pulseIn(pinEcho, HIGH, 20000);
  
  // no signal received / timeout
  if (duration == 0) return 0;
  
  distance = (duration * 0.0342)/2.0;
  
  // Optional: implement Median filtering / EPA
  
  // Adjusted distance
  return distance * R + offset;
}

//-----------------------------------------------
//**********************************
//Each  funcion uses different values in the analogWrite and delay. That's because the motors  aren't exactly the same and the battery will slightly lose power.

// used for moving forward
void  walk() //Makes the cars go straight
{
  digitalWrite(In1, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);   // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  analogWrite(EnA, 110);    //Top down: right
  analogWrite(EnB, 120);    //Top down: left 110
  delay(300); // og val 150
  //digitalWrite(In1, HIGH);  // Brake - HIGH == on, LOW == off
  //digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

void slightLeft()
{
  digitalWrite(In1, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);   // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  analogWrite(EnA, 100);    //Top down: right
  analogWrite(EnB, 0);    //Top down: left
  delay(100);
  //digitalWrite(In1, HIGH);  // Brake - HIGH == on, LOW == off
  //digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

void slightRight()
{
  digitalWrite(In1, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);   // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  analogWrite(EnA, 0);    //Top down: right
  analogWrite(EnB, 100);    //Top down: left
  delay(100);
  //digitalWrite(In1, HIGH);  // Brake - HIGH == on, LOW == off
  //digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

// case 2
void Center()
{
  // Right has an open spot 
  if (((rightdist < WALL_VAL_ERROR) || (rightdist > (MAX_VAL_SIDE_WALL + WALL_VAL_ERROR))) && (leftdist < (MAX_VAL_SIDE_WALL + WALL_VAL_ERROR)))
  {
    if (leftdist < ((MAX_VAL_SIDE_WALL -  (WALL_VAL_ERROR )) / 2))
    {
      slightRight();
    }
    else if (leftdist > ((MAX_VAL_SIDE_WALL -  (WALL_VAL_ERROR)) / 2))
    {
      slightLeft();
    }
  }
  else
  {
    if( leftdist < (rightdist - (WALL_VAL_ERROR)))
    {
      slightRight();
    }
    else if( leftdist > (rightdist + (WALL_VAL_ERROR)))
    {
      slightLeft();
    }
  }
  walk();
  walk();
}


// Case 1
void TurnR()  // straight, turn right
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 140);  // Motor A
  analogWrite(EnB, 170);  // Motor B
  delay(100);

  // Turn Right
  analogWrite(EnA, 0);
  analogWrite(EnB, 145); //145

  // Motor A: right - Brake, Direction
  digitalWrite(In1, HIGH);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, HIGH);  // Direction - Low == forward, HIGH == reverse

  // Motor B: left - Brake, Direction
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In4, HIGH);  // Direction - Low == reverse, HIGH == forward

  // Duration
  delay(500); //620

  // Turn off
  analogWrite(EnB, 0);
  walk();
  walk();
}

// Case 3
void TurnL() // straight, turn left
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 160); // Motor A
  analogWrite(EnB, 180);  // Motor B
  delay(100);

  // Turn left
  analogWrite(EnA, 140); //140
  analogWrite(EnB, 0);

  // Motor A: right - Brake, Direction
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  
  // Motor B: left - Brake, Direction
  digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  digitalWrite(In4, LOW);   // Direction - Low == reverse, HIGH == forward

  // Duration
  delay(400); //600

  // Turn off
  analogWrite(EnA, 0);
  walk();
  walk();
}

// case 4
void TurnB()  // straight, turn left
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, LOW);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 140); // Motor A
  analogWrite(EnB, 190);  // Motor B
  //delay(1200); 800
  delay(800);

  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

//This is the function  that measures the distances using the ultrassonic sensors.
//As the ending of  the maze is always in the right corner, the function will give priority the right  . It will only start measuring the front distance
//if the right sensor detects  an obstacle (values below 20cm). The same goes for the left sensor, it will only  start measuring if the right and front sensors detect an obstacle.
//This type  of functioning prevents the car to waste time in measuring distances that aren't  be necessary.
//The function pings() returns a value, called dir, between 1 and  4. 1 - No obstacle on the right, 2 - Obstacle on the right, 3 - Obstacles in front  and right and 4 - Obstacles in all directions.

// Output state value which determines which action is taken
// 1 right, 2 walk, 3 left, 4 back
int pings(){
  //const  int D = 20;
  
  // Get the distance in each direction
  rightdist = getDistance(RIGHT_TRIGGER, RIGHT_ECHO, RIGHT_R, RIGHT_OFFSET);
  leftdist = getDistance(LEFT_TRIGGER, LEFT_ECHO, LEFT_R, LEFT_OFFSET);
  frontdist = getDistance(FRONT_TRIGGER, FRONT_ECHO, FRONT_R, FRONT_OFFSET);

  Serial.println("************************");
  Serial.print("L: ");
  Serial.println(leftdist);
  Serial.print("F: ");
  Serial.println(frontdist);
  Serial.print("R: ");
  Serial.println(rightdist);
  Serial.print("Next: ");
  Direction dir = GetNextMov(leftdist, frontdist, rightdist);

  // convert direction int spesific case
  // left -> 3
  // forward -> 2
  // right -> 1
  // back -> 4
  // complete -> 5
  switch (dir)
  {
    case LEFT:
      return 3;
      break;

    case FORWARD:
      return 2;
      break;
    
    case RIGHT:
      return 1;
      break;
    
    case BACK:
      return 4;
      break;

    case COMPLETE:
      return 5;
      break;

    default:
      return -1;
  }
  Serial.println(dir);
  Serial.println("************************");
  
  return dir;
}

void RunMaze()
{
  while(!Serial.available());
  while(1)
  {
    int direction = pings();
    switch (direction)
    {
      case  1:
        Serial.print("state = ");
        Serial.println(direction);
        delay(100);
        TurnR();
        break;
      
      // Move Foward / Centering
      case 2:
        Serial.print("state = ");
        Serial.println(direction);
        delay(100);
        Center();
        break;
      
      // Turn Left
      case 3:
        Serial.print("state = ");
        Serial.println(direction);
        delay(100);
        TurnL();
        break;

      // Turn back
      case 4:
        Serial.print("state  = ");
        Serial.println(direction);
        delay(100);
        TurnB();
        break;
      
      // Maze Complete
      case 5:
        Serial.print("state  = ");
        Serial.println(direction);
        delay(1000);
        while(1);
        break;
    }
  }
}

void  loop()
{
  RunMaze();
}
