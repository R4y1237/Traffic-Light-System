#include <IRremote.h>

#include <Servo.h>


/**
 * Name: Ray Wang, Robin Li
 * Date: May 17, 2022
 * Teacher: Mr. Wong (TEJ3MP)
 * Purpose: This program runs a T-intersection traffic light system
 *       while constantly checking for the light level for a streetlight
 *      independently of the traffic light system.
 */

/**
 * Variables to declare the pins of the side (left/right) traffic lights.
 */
int sRed = 13;
int sYellow = 12;
int sGreen = 11;

/**
 * Variables to declare the pins of the middle traffic lights.
 */
int mRed = 10;
int mYellow = 9;
int mGreen = 8;

/**
 * Variables to declare the pins of the photoresistor and streetlight.
 */
int pr = A0;
int street = 7;

/**
 * Variables to declare the time durations of each light.
 */
int yDelay = 2000; //Duration for a yellow light.
int gDelay = 3000; //Gap between when all lights are red and one turns green.
int rDelay = 1000; //Duration for a red light (and the complementary green light).

int redMillisParallel = rDelay;
int yellowMillisParallel = yDelay;
int greenMillisParallel = gDelay;

int redMillisAcross = rDelay;
int yellowMillisAcross = yDelay;
int greenMillisAcross = gDelay;

/**
 * Variable to store the previous time to allow tracking of time passed.
 */
unsigned long prevMillis = 0;

/**
 * Variable to store which phase "p" of the traffic light system is currently running
 * so that the program uses the correct delay and has the correct lights turned on.
 */
int p = 1;

//variables for ultrasonic sensor
int trigPin = 6;
int echoPin = 5;
unsigned long duration;
double distance;

//variable for IR sensor
int irSensor = 4;
int irSensorValue = -1;
bool isRemoteOn = false;
bool isIrRunning = false;
unsigned long prevMillisServoIR = 0;
  
//variables for servo
unsigned long prevMillisServo = 0;
int degreeCount = 0;
int servoPin=3;
int ps = 0;
Servo gate;

//variables for pedestrian light
int pedLight = 2;
int pedLightValue;
int cycleCount = 0;
int lightWhenPressed;

//variables for button 
int button = A1;




/**
 * Method to initialize the LEDs and the LDR.
 */
void setup()
{
  Serial.begin(115200);
  pinMode(sRed, OUTPUT);
  pinMode(sYellow, OUTPUT);
  pinMode(sGreen, OUTPUT);
  
  pinMode(mRed, OUTPUT);
  pinMode(mYellow, OUTPUT);
  pinMode(mGreen, OUTPUT);
  
  pinMode(pr, INPUT);
  pinMode(street, OUTPUT);
  //Serial.begin(9600);
  
  writeStates(LOW, LOW, HIGH, LOW, HIGH, LOW, HIGH); //Sets initial light sequence
  
  //ultrasonic sensor setup
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  
  pinMode(irSensor,INPUT);
  IrReceiver.begin(irSensor);
  
  pinMode(servoPin,OUTPUT);
  gate.attach(servoPin);
  gate.write(0);
  pinMode(pedLight,OUTPUT);
  
  pinMode(button,INPUT);
  
}

/**
 * Method to run the traffic/streetlights constantly.
 */
void loop()
{
  readPedLight();
  setTimings();
  trafficLight();
  streetLight();
  readIR();
  
  if(!isIrRunning){
    ultrasonicSensor();
  	distanceCheck();
    servo();
  }
  if(isIrRunning) updateIR();
  
  gate.write(degreeCount);
  delay(10);
  
}

void readPedLight(){
  if(pedLightValue == LOW){
    pedLightValue = digitalRead(button);
  }
}

void setTimings(){
  if(pedLightValue == HIGH){
    if(cycleCount == 0){
      cycleCount = 1;
      if(p == 1) lightWhenPressed = 3;
      else if(p == 2) lightWhenPressed = 2;
      else lightWhenPressed = 1;
    }
    
    if(lightWhenPressed == 1){  
      if(cycleCount == 1){
		redMillisParallel = rDelay * 0.5;
        yellowMillisParallel = yDelay;
        greenMillisParallel = gDelay;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
      else if(cycleCount ==2){
        redMillisParallel = rDelay;
        yellowMillisParallel = yDelay*0.5;
        greenMillisParallel = gDelay*1.5;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
    }else if(lightWhenPressed == 2){
      if(cycleCount == 1){
		redMillisParallel = rDelay;
        yellowMillisParallel = yDelay * 0.5;
        greenMillisParallel = gDelay;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
      else if(cycleCount ==2){
        redMillisParallel = rDelay;
        yellowMillisParallel = yDelay;
        greenMillisParallel = gDelay*1.5;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
    }else if(lightWhenPressed == 3){
      if(cycleCount == 1){
		redMillisParallel = rDelay;
        yellowMillisParallel = yDelay;
        greenMillisParallel = gDelay;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
      else if(cycleCount ==2){
        redMillisParallel = rDelay * 0.5;
        yellowMillisParallel = yDelay * 0.5;
        greenMillisParallel = gDelay*1.5;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
      }
    }else if(pedLightValue == LOW){
        redMillisParallel = rDelay;
        yellowMillisParallel = yDelay;
        greenMillisParallel = gDelay;
        
        redMillisAcross = rDelay;
        yellowMillisAcross = yDelay;
        greenMillisAcross = gDelay;
    }
  }
  
}
/**
 * Method to run the traffic lights by checking if its
 * on the correct part of the phase "p" and if the time required
 * for the phase has passed.
 */
void trafficLight() {
  if (p == 1 && wait(greenMillisParallel))
    writeStates(LOW, LOW, HIGH, HIGH, LOW, LOW, HIGH);
  if (p == 2 && wait(yellowMillisParallel))
    writeStates(LOW, LOW, HIGH, LOW, LOW, HIGH, LOW);
  if (p == 3 && wait(redMillisParallel))
    writeStates(LOW, HIGH, LOW, LOW, LOW, HIGH, LOW);
  if (p == 4 && wait(greenMillisAcross))
    writeStates(HIGH, LOW, LOW, LOW, LOW, HIGH, LOW);  
  if (p == 5 && wait(yellowMillisAcross))
    writeStates(LOW, LOW, HIGH, LOW, LOW, HIGH, LOW);
  if (p == 6 && wait(redMillisAcross))
    writeStates(LOW, LOW, HIGH, LOW, HIGH, LOW, HIGH);
}

/**
 * Helper method to check if the time specified has passed, then
 * changes the light sequence to run the next phase and returns true.
 */
bool wait(int t) {
  unsigned long time = millis();
  if(time-prevMillis >= t) {
    prevMillis = time;
    
    if(p >= 6) {
      p = 1; 
    } else {
      p++;
    }
    
    return true;
  } else return false;
}

/**
 * Helper Method to turn on/off the LEDs.
 */
void writeStates(int sY, int sG, int sR, int mY, int mG, int mR, int ped) {
  digitalWrite(sYellow, sY);
  digitalWrite(sGreen, sG);
  digitalWrite(sRed, sR);
  digitalWrite(mYellow, mY);
  digitalWrite(mGreen, mG);
  digitalWrite(mRed, mR);
  digitalWrite(pedLight,ped);
}

/**
 * Method to check the light level and turn on/off the streetlight.
 * If the light level is below a threshold, then the streetlight will
 * turn on; otherwise it will remain off.
 */
void streetLight() {
  //Serial.println(analogRead(pr));
  if(analogRead(pr) < 150) {
    digitalWrite(street, HIGH); 
  } else {
    digitalWrite(street, LOW);
  }
}

void distanceCheck(){
  //Serial.println(distance);
  if(distance >= 150 && distance <= 200){ 
    if(ps == 0){
     ps = 1;
     prevMillisServo = millis();
    } 
    else if(ps == 2 || ps == 3){
      ps = 2;
    }
  }
  //Serial.println(ps);
}

void ultrasonicSensor(){
	digitalWrite(trigPin, LOW); 
  	digitalWrite(trigPin, HIGH);
  	digitalWrite(trigPin, LOW);
  
  	duration = pulseIn(echoPin, HIGH);
  	distance = duration * 0.034 / 2;
}
bool servoWait(int t){
  unsigned long servoTime = millis();
  //Serial.println(servoTime - prevMillisServo);
  //Serial.println(prevMillisServo);
  if(servoTime-prevMillisServo >= t) {
    if(ps == 1){
     ps++;
    }else if(ps == 2 && gate.read()==60){
     ps++; 
     isIrRunning = false;
     irSensorValue = -1;
    }else if (ps == 3){
      ps++;
    }else if (ps == 4 && gate.read() == 0){
      ps = 0;
      isIrRunning = false;
      irSensorValue = -1;
    }
    prevMillisServo = servoTime;
    return true;
  } else return false;
}

void servo(){ 
  if(ps == 1 && servoWait(1900)){}
  else if(ps == 2 && servoWait(100)){
    degreeCount+=2;
    if(degreeCount >= 60){
      degreeCount = 60;
    }
  }
  else if(ps == 3 && servoWait(1400)){}
  else if(ps == 4 && servoWait(100)){
    degreeCount-=2;
    if(degreeCount <= 0){
      degreeCount = 0;
    }
  }
}

void readIR(){
  if(IrReceiver.decode()){
    if(isRemoteOn &&  (IrReceiver.decodedIRData.command == 10 ||IrReceiver.decodedIRData.command == 8)){
      irSensorValue = IrReceiver.decodedIRData.command;
      ps=0;
      prevMillisServo = millis();
      isIrRunning = true;
    }
    else if(IrReceiver.decodedIRData.command == 0){
      isRemoteOn = !isRemoteOn;
      if(!isRemoteOn){
       irSensorValue = -1;
       isIrRunning = false;
      }
    }
    IrReceiver.resume();
  }
  
}

void updateIR(){
  if(irSensorValue == 10){
    degreeCount += 2;
    if (degreeCount >= 60){
      degreeCount = 60;
      isIrRunning = false;
    }
  }
  else if(irSensorValue == 8){
   	degreeCount -= 2;
    if (degreeCount <= 0) {
      degreeCount = 0;
      isIrRunning = false;
    }
  }
}


