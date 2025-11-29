//Authors: Gonçalo Azinheira and João Castro, students at University of  the Algarve(Ualg);
//Project: Mazemaster3000;
//Subject: A finite state machine-powered  maze solver. The starting point of the maze must be in one of the left corners and  the ending must be in one of the right corners.

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

//#include "NavigationController.h"
#include "NavigationController.c"

const  int pingt = 7;   //the same trigger is used by all sensors
const int pinge2 = 6;   //front ultrassonic sensor
const int pinge1 = 8;   //right ultrassonic sensor
const  int pinge3 = 9;  //left ultrassonic sensor
int rec[50];
int a;

int In1 = 8;   //right      // Modified - From A5 to 8    Brake - HIGH == on, LOW == off
int In2 = 13;  //right      // Modified - From A4 to 13   Direction - Low == reverse, HIGH == forward
int EnA = 11;  //right      // Modified - From 10 to 11   PWM

int In3 = 9;   //left       // Modified - From A3 to 9    Brake - HIGH == on, LOW == off     
int In4 = 12;  //left       // Modified - From A2 to 12   Direction - Low == forward, HIGH == reverse
int EnB = 3;   //left       //  Modified - From 11 to 3   PWM
int dir;
int esqdist;
int dirdist;
int  fdist;
boolean button=0;
static int state=-1;
double rightdist = 0;
double leftdist = 0;
double frontdist = 0;

void B_ISR(){ //The button is activated by an  Interrupt System Routine
    button=1;
 }  

void setup() {
  // HC-05 default baud rate
  Serial.begin(38400);
  //button=0;
  pinMode(button,  INPUT);
  attachInterrupt(0,B_ISR,RISING);

  // Ultrasonic Sensors
  pinMode(pingt, OUTPUT);
  pinMode(pinge1, INPUT);
  pinMode(pinge2, INPUT);
  pinMode(pinge3, INPUT);

  a=0;

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

  state = -1;

  // Sensor Distances
  rightdist = 0;
  leftdist = 0;
  frontdist = 0;

  // Help to Serial
  Serial.println("************************");
  Serial.println("Accepted Values: ");
  Serial.println("Turn Right:  1");
  Serial.println("Go Straight: 2");
  Serial.println("Turn Left:   3");
  Serial.println("Turn Around: 4");
  Serial.println("************************");
}


//******************************************
//---------------------------------------------------
int  ping (int pinge){
  long duration, cm;
  int x,xmed;
  int dela=25;
  int MAX = 1500;
  int num = 0;
  int sum=0;

  for(int i=0;i<=5;i++){
    digitalWrite(pingt, LOW);
    delayMicroseconds(2);
    digitalWrite(pingt,  HIGH);
    delayMicroseconds(5);
    digitalWrite(pingt, LOW);
    duration =  pulseIn(pinge, HIGH);         //Refrence Doc:   https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/
    x = microsecondsToCentimeters(duration);
    if(x<MAX){
      num ++;
    }
    else{x=0;}
    sum+=x;
    
    delay(dela);
  }

  xmed=sum/num;
  return xmed;
} 

long microsecondsToCentimeters(long  microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
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

// Case 2
void  walk(int wtime) //Makes the cars go straight
{
  digitalWrite(In1, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);  // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);   // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, HIGH);   // Direction - Low == reverse, HIGH == forward
  analogWrite(EnA, 155);    //Top down: right
  analogWrite(EnB, 220);    //Top down: left
  delay(965);
  //digitalWrite(In1, HIGH);  // Brake - HIGH == on, LOW == off
  //digitalWrite(In3, HIGH);  // Brake - HIGH == on, LOW == off
  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}


// Case 1
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
  delay(620);
  //delay(duration);

  // Turn off
  analogWrite(EnB, 0);
}

// Case 3
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
  delay(600);
  //delay(duration);

  // Turn off
  analogWrite(EnA, 0);
}

// Case 4
void back(int wtime) //Makes the car turn around  180. This happens when the sensor detect obstacles in all directions.
{
  // Equalize speeds, move forward an inch
  digitalWrite(In1, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In3, LOW);   // Brake - HIGH == on, LOW == off
  digitalWrite(In2, LOW);  // Direction - Low == forward, HIGH == reverse
  digitalWrite(In4, LOW);   // Direction - Low == reverse, HIGH == forward
  delay(10);
  analogWrite(EnA, 140); // Motor A
  analogWrite(EnB, 190);  // Motor B
  delay(800);

  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

// Case 5
void TurnA(int wtime) //
{
  analogWrite(EnA, 192);
  analogWrite(EnB, 172);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(wtime-141);
  analogWrite(EnA, 170);
  analogWrite(EnB, 170);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(wtime*2+50);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(5);
}

// Case 6
void TurnB(int wtime) //
{
  analogWrite(EnA, 170);
  analogWrite(EnB, 170);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  delay(wtime-130);
  analogWrite(EnA, 170);
  analogWrite(EnB, 170);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(wtime*2+30);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(5);
}

//**********************************
//-----------------------------------------------

//This  is the implementation of the finit state machine shown in the picture. It starts  always in state -1 and stays there until the button is pressed. After that, it will  go to state 0
//and measures the distances using the ultrassonic sensors using  the pings() function. According to what it measured, the finite state machine will  go to the corresponding state using the
//dir value (the meaning of this variable  is explained below in the pings() function). The states 1 to 4 are responsible for  making the car turn right, go straight, turn left or turn around respectively.
//At  each of those states, the first thing that is done is record a value in the array  rec []. This value is equal to the the present state of the machine. For example,  if the car is going
//straight ahead, it is in state 2 and the value recored  in the array will be also 2. In other words, the array stores the actions done by  the car to be written in the function Recb().
//The cars reaches to the end if  it does 3 lefts in a row and to do so, it needs to go to state 3, state 8 and 5  consecutively. So state 8 represents the 2nd left and state 5 represents the
//3rd  left. After reaching to the end of the maze, it goes from state 5 to 6. By this  point, the finite state machine will be always moving from state 6 to 7 and vice-versa.
//State  6 turns on the arduino's builtin LED and state 7 turns it off. Regardless of if  it is in 6 or 7, it waits for the button to be pressed to print the log.

void  MazeMaster_FSM(){
//The initial state is always -1 until  the button is pressed.
const int D = 20;  //This is the reference distance, in  cm, for the car to consider the obstacles. If the sensor detects something below  this distance, it will consider as an obstacle.
const int wtime = 500;  //Time  used on each action(right, straight, left or turn around).

 

  switch(state){

    case -1:
      if(button==1) {
        button=0;
        state=0;
      }
      break;
    
    case 0:
      dir=pings();
      state=dir;
      break;

    case  1:
      Serial.print("state = ");
      Serial.println(state);
      rec[a]=1;
      a++;
      turnR(wtime);
      
      // DEBUG - restore once sensors are operational
      //dir=pings();
      //state=dir;
      break;
    
    case 2:
      Serial.print("state = ");
      Serial.println(state);
      rec[a]=2;
      a++;
      Serial.print("a2 = ");
      Serial.println(a);
      walk(wtime);
      
      // DEBUG - restore once sensors are operational
      //dir=pings();
      //state=dir;
      break;
    
    case 3:
      Serial.print("state = ");
      Serial.println(state);
      rec[a]=3;
      a++;
      turnL(wtime);
      
      // DEBUG - restore once sensors are operational
      //dir=pings();
      //if(dir==3)state=8;
      //else state=dir;
      break;

    case 4:
      Serial.print("state  = ");
      Serial.println(state);
      rec[a]=4;
      a++;
      Serial.print("a4  = ");
      Serial.println(a);
      back(wtime);

      // DEBUG - restore once sensors are operational
      //dir=pings();
      //state=dir;
      break;

    case 5:
    /*
      Serial.print("state = ");
      Serial.println(state);
      rec[a]=3;
      a++;
      turnL(wtime);
      Serial.println("I HAVE ARRIVED! ");
      rec[a]=5;
      state=6;
      break;
    */
      TurnA(wtime); // originally right
      break;
    
    case 6:
    /*
      digitalWrite(LED_BUILTIN,HIGH);
      if(button==1)
      {Recb();
      button==0;}
      delay(500);
      state=7;
      break;
    */
      TurnB(wtime); // originally left
      break;
     
    case 7:
      digitalWrite(LED_BUILTIN,LOW);
    
      if(button==1) {
        Recb();
        button==0;
      }
      delay(500);
      state=6;
      break;

    case 8:
      Serial.print("state = ");
      Serial.println(state);
      rec[a]=3;
      a++;
      turnL(wtime);

      dir=pings();
      if(dir==3)state=5;
      else state=dir;
      break;

  }

}
//------------------------------

void  Recb(){
  Serial.print("start -> ");
  for(int i=0;i<=a;i++)
  {Serial.println(rec[i]);}
  button=0;
}
    //---------------------------

//This is the function  that measures the distances using the ultrassonic sensors.
//As the ending of  the maze is always in the right corner, the function will give priority the right  . It will only start measuring the front distance
//if the right sensor detects  an obstacle (values below 20cm). The same goes for the left sensor, it will only  start measuring if the right and front sensors detect an obstacle.
//This type  of functioning prevents the car to waste time in measuring distances that aren't  be necessary.
//The function pings() returns a value, called dir, between 1 and  4. 1 - No obstacle on the right, 2 - Obstacle on the right, 3 - Obstacles in front  and right and 4 - Obstacles in all directions.

// Output state value which determines which action is taken
// 1 right, 2 walk, 3 left, 4 back
int pings(){
  const  int D = 20;
  
  rightdist = getDistance(int pinTrig, int pinEcho, double R, double offset);
  leftdist = getDistance(int pinTrig, int pinEcho, double R, double offset);
  frontdist = getDistance(int pinTrig, int pinEcho, double R, double offset);
  /*
  dirdist = ping(pinge1);
  Serial.print("distance  at right(->) = ");
  Serial.println(dirdist);

  if(dirdist>D){dir=1;}
  else{
    fdist = ping(pinge2);
    Serial.print("distance at front(^)  = ");
    Serial.println(fdist);

    if(fdist>D){dir=2;}
    else{ 
      esqdist = ping(pinge3);
      Serial.print("distance at left(<-) = ");
      Serial.println(esqdist);

      if(esqdist>D){dir=3;}
      else {dir=4;}
    }
  }
  */
  return dir;
}

//---------------------------

void  loop() {
  String sendString;
  String Q;
  // Computer to Arduino
  while (Serial.available()) {
    delay(1);
    if (Serial.available() > 0) {
      char c = Serial.read();  //gets one byte from serial buffer
      if (isControl(c)) {
        //'Serial.println("it's a control character");
        break;
      }
      sendString += c; //makes the string sendString
    }
  }
  Q = sendString;
  if(Q.charAt(0) == '1' || Q.charAt(0) == '2' || Q.charAt(0) == '3' || Q.charAt(0) == '4' || Q.charAt(0) == '5' || Q.charAt(0) == '6'){         
    state = Q.toInt();
    Serial.print("Sent value: ");
    Serial.println(Q.toInt());
    MazeMaster_FSM();
  }
}

  
