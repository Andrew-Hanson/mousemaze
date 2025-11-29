// Authors: Gonçalo Azinheira and João Castro, students at University of  the Algarve(Ualg);
// Project: Mazemaster3000;
// Subject: A finite state machine-powered  maze solver. The starting point of the maze must be in one of the left corners and  the ending must be in one of the right corners.
// Modified by Lawrence Scott, November 2025


//----------------------------------------------------------------------------
/* ##### MODIFIED #####
  Motor Pin Shield Information:
    Motor A:        Left
      Direction 12
      PWM       3
      Brake     9
    Motor B:        Right
      Direction 13
      PWM       11
      Brake     8
*/

int In1 = 8;   //right      // Modified - From A5 to 8    Brake - HIGH == on, LOW == off
int In2 = 13;  //right      // Modified - From A4 to 13   Direction - Low == reverse, HIGH == forward
int EnA = 11;  //right      // Modified - From 10 to 11   PWM

int In3 = 9;   //left       // Modified - From A3 to 9    Brake - HIGH == on, LOW == off     
int In4 = 12;  //left       // Modified - From A2 to 12   Direction - Low == forward, HIGH == reverse
int EnB = 3;   //left       //  Modified - From 11 to 3   PWM

static int state=-1;

// Variables for serial capture
String readString; //main captured String 
String motor; //data String
String dir;
String speed;
String duration;

int ind1; // , locations
int ind2;
int ind3;
int ind4;


void setup() {
  // HC-05 default baud rate
  Serial.begin(38400);

  // Motor config
  pinMode(In1,  OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnA,  OUTPUT);
  pinMode(EnB, OUTPUT);

  analogWrite(EnA,  0);
  analogWrite(EnB, 0);

  state = -1;

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
//Each  funcion uses different values in the analogWrite and delay. That's because the motors  aren't exactly the same and the battery will slightly lose power.

// Case 1 - Right
void motorA(int dir, int speed, int duration)
{
  Serial.print("dir = ");
  Serial.println(dir);
  Serial.print("speed = ");
  Serial.println(speed);
  Serial.print("duration = ");
  Serial.println(duration);
  analogWrite(EnA, speed);
  analogWrite(EnB, 0);

  // Motor A: right - Brake, Direction
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  if (dir == 0){ // reverse
    digitalWrite(In2, HIGH);  // Direction - Low == forward, HIGH == reverse
  } else if (dir == 1){ // forward
    digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  }
  
  // Motor B: left - Brake, Direction
  digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  digitalWrite(In4, LOW);   // Direction - Low == reverse, HIGH == forward

  // Duration
  delay(duration);

  // Turn off
  analogWrite(EnA, 0);
}

// Case 2 - Left
void motorB(int dir, int speed, int duration)
{
  analogWrite(EnA, 0);
  analogWrite(EnB, speed);

  // Motor A: right - Brake, Direction
  digitalWrite(In1, HIGH);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, HIGH);  // Direction - Low == forward, HIGH == reverse

  // Motor B: left - Brake, Direction
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  if (dir == 0){ // reverse
    digitalWrite(In4, LOW);  // Direction - Low == reverse, HIGH == forward
  } else if (dir == 1){ // forward
    digitalWrite(In4, HIGH);  // Direction - Low == reverse, HIGH == forward
  }

  // Duration
  delay(duration);

  // Turn off
  analogWrite(EnB, 0);
}

// Case 3 - Right
void TurnL(int duration) // straight, turn right
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 160); // Motor A
  analogWrite(EnB, 180);  // Motor B
  delay(300);

  // Turn right
  analogWrite(EnA, 140);
  analogWrite(EnB, 0);

  // Motor A: right - Brake, Direction
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  
  // Motor B: left - Brake, Direction
  digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  digitalWrite(In4, LOW);   // Direction - Low == reverse, HIGH == forward

  // Duration
  //delay(700); 600
  delay(duration);

  // Turn off
  analogWrite(EnA, 0);
}

// Case 4 - Left
void TurnR(int duration)  // straight, turn left
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 140);  // Motor A
  analogWrite(EnB, 170);  // Motor B
  delay(300);

  // Turn left
  analogWrite(EnA, 0);
  analogWrite(EnB, 145);

  // Motor A: right - Brake, Direction
  digitalWrite(In1, HIGH);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, HIGH);  // Direction - Low == forward, HIGH == reverse

  // Motor B: left - Brake, Direction
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In4, HIGH);  // Direction - Low == reverse, HIGH == forward

  // Duration
  //delay(900); 620
  delay(duration);

  // Turn off
  analogWrite(EnB, 0);
}

/* ### Does a pretty good 90 degree turn in place
// Case 5 - Turn Back
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
  delay(600);

  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}
*/

// Case 5 - Turn Back
void TurnB(int duration)  // straight, turn left
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
  delay(duration);

  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

void  MazeMaster_FSM(){
//The initial state is always -1 until  the button is pressed.
const int D = 20;  //This is the reference distance, in  cm, for the car to consider the obstacles. If the sensor detects something below  this distance, it will consider as an obstacle.
const int wtime = 500;  //Time  used on each action(right, straight, left or turn around).

  switch(state){

    case -1:
      break;

    case  1: // Motor A
      Serial.print("state = ");
      Serial.println(state);
      motorA(dir.toInt(), speed.toInt(), duration.toInt());
      state = -1;
      break;
    
    case 2: // Motor B
      Serial.print("state = ");
      Serial.println(state);
      motorB(dir.toInt(), speed.toInt(), duration.toInt());
      state = -1;
      break;

    case 3: // Straight, turn right
      Serial.print("state = ");
      Serial.println(state);
      TurnL(duration.toInt());
      state = -1;
      break;

    case 4: // Straight, turn left
      Serial.print("state = ");
      Serial.println(state);
      TurnR(duration.toInt());
      state = -1;
      break;

    case 5: // Turn back
      Serial.print("state = ");
      Serial.println(state);
      TurnB(duration.toInt());
      state = -1;
      break;
  }

}

//---------------------------

void  loop() {
  // Serial input example: motor, direction, speed, duration *(data end indicator)
  // 2,1,150,900*
  // Example by zoomkat - https://forum.arduino.cc/t/split-string-by-delimiters/373124/6
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == '*') {
      //do stuff
      
      Serial.println();
      Serial.print("captured String is : "); 
      Serial.println(readString); //prints string to serial port out
      
      ind1 = readString.indexOf(',');  //finds location of first ,
      motor = readString.substring(0, ind1);   //captures first data String
      ind2 = readString.indexOf(',', ind1+1 );   //finds location of second ,
      dir = readString.substring(ind1+1, ind2);   //captures second data String
      ind3 = readString.indexOf(',', ind2+1 );
      speed = readString.substring(ind2+1, ind3);
      ind4 = readString.indexOf(',', ind3+1);
      duration = readString.substring(ind3+1); //captures remain part of data after last ,

      Serial.print("motor = ");
      Serial.println(motor); 
      Serial.print("dir = ");
      Serial.println(dir);
      Serial.print("speed = ");
      Serial.println(speed);
      Serial.print("duration = ");
      Serial.println(duration);
      Serial.println();
      Serial.println();

      state = motor.toInt();

      MazeMaster_FSM();

      readString=""; //clears variable for new input
      motor="";
      dir="";
      speed="";
      duration="";
    }  
    else {     
      readString += c; //makes the string readString
    }
  }
}

  

