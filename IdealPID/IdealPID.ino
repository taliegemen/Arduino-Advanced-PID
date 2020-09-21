// Encoder library version 1.4.1 by Paul Stoffregen is required
// PLX-DAQ is required if you want to hold the data 
#define ENCODER_OPTIMIZE_INTERRUPTS // To enable assembly optimization of Encoder library
#include <Encoder.h> // Encoder library defining
Encoder EncA(2, 3); //encoder inputs
int motor = 9; // 1st pin of the motor
int motorback = 10; // 2nd pin of the motor
float error = 0;
float wanted = 100; // This is the desired position
float ti = 99999999999; //ti is set to practically infinite to cancel integral gain
float td = 0; // td is set to 0 to cancel derivative gain
float Kc = 1; // main gain
float Ki = 0; // integral gain
float Kd = 0; // derivative gain
//local-ish variables start
float integral;
float derivative;
float Ku;
float lasterror;
float pos = 0; //last position 
float last = 0; //time at last iter
float tim2 = 0; //time for integral wind-up
float now = 0; //time atm
float timchang = 0; //difference in time
//local-ish variables end
void setup() {
  Serial.begin(38400); //set it to highest value as possible for accuracy
  //pin set
  pinMode(motor, OUTPUT);
  pinMode(motorback, OUTPUT);
  //pin set end
  // plx daq setup
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Real Time,Timer, Position");
  Serial.println("RESETTIMER");
  // plx daq setup end
}

void loop() {
  pos = EncA.read(); //get current position
  error = wanted - pos; //get current error
  now = millis(); //get current time in ms
  timchang = now - last; //get change in time since last iter
  //plx-daq code start
  Serial.print("DATA,TIME,");
  Serial.print(now);
  Serial.print(",");
  Serial.println(pos);
  //plx-daq code end
  //pid gain calculations
  integral += error * timchang *0.001;
  Ki = error + (integral / ti);
  derivative = ((error - lasterror) / (timchang)) * 0.001;
  Kd =(derivative * td);
  Ku = Kc * (Ki + Kd);
  //pid gain calculations end
  //we can get maximum of 255 when sending pwm
  if (Ku >= 255) {
    Ku = 255;
  }
  if (-255 >= Ku) {
    Ku = -255;
  }
  //if we get minus value, then send current to 2nd pin instead of first
  if (Ku >= 0) {
    analogWrite(motor, Ku);
    analogWrite(motorback, 0);
  }
  if (0 >= Ku) {
    Ku = -Ku;
    analogWrite(motor, 0);
    analogWrite(motorback, Ku);
  }
  // end of driving the motor
  last = now; //note the last iter time
  //these are for wind-up
  //tim2 += 5;
  //if (tim2 == 200) {
   //integral = 0;
  //}
   lasterror = error; // note the last iter error
}
