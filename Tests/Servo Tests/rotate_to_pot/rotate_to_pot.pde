// see the adafruit motor control library for managing the DC motor
#include <AFMotor.h>
AF_DCMotor motor(1, MOTOR12_8KHZ);

//set up pins for the quadrature encoder
//At least one of them must support external interrups so it has to be 2 or 3 (interrupt 0 or 1 just for confusions sake)
//I picked 2 and 3 because I wanted to play with using full quad counting and therefore needed 2 interrupts
// make sure to check the adaruit controller FAQ to find the pins used by the motors in various configurations
#define encoder0PinA 2
#define encoder0PinB 3

//the encoder wheel position is set in a ISR so it has to be marked volatile
volatile int encoder0Pos = 0; 
//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
//PID controller constants
float KP = 2; //position multiplier (gain)
float KI = .05; // Intergral multiplier (gain)
float KD = 0; // derivative multiplier (gain)

//track the previous error for the derivitive term, and the sum of the errors for the integral term
int lastError = 0;
int sumError = 0;
//Integral term min/max (random value and not yet tested/verified)
int iMax = 100;
int iMin = 0;
//pick an analog input for the potentiometer
int potPin = 5;

void setup() {
  //set encoder pind to input
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  // attach interrupt service routine (ISR) to the encoder pins
  attachInterrupt(0, doEncoderA, CHANGE);
  //to use the full 4x encoding uncomment out the other ISR attachment
  //attachInterrupt(1, doEncoderB, CHANGE);  
}

void loop() {

  int val = analogRead(potPin); // read the potentiometerv alue (0 - 1023)
  int target = map(val,0,1023,0,3600);// set the target to seek to by mapping the potentiometer to the encoder max count
  int error = encoder0Pos - target; // find the error term of current position - target
  // generalized PID formula
  //correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
  int ms = KP * error + KD * (error - lastError) +KI * (sumError);// calculate a motor speed for the current conditions
  // set the last and sumerrors for next loop iteration
  lastError = error;
  sumError += error;
  //scale the sum for the integral term
  if(sumError > iMax){
    sumError = iMax;
  }
  else if(sumError < iMin){
    sumError = iMin;
  }

  int direction; //determine the direction to go in since the adafruit controller expects posotive values
  if(ms > 0){
    direction = BACKWARD;
  }
  if(ms < 0){
    direction = FORWARD;
    ms = -1 * ms;
  }

  // map the result to the max speed the controller will expect 
  //(not sure if this is a good idea)
  int motorSpeed = map(ms,0,7200,0,6500);
  // do it  
  motor.setSpeed(motorSpeed);
  motor.run(direction);
}
//ISR functions
void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // look for a high-to-low on channel A
  else                                        
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);          
  // use for debugging - remember to comment out

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }

} 
