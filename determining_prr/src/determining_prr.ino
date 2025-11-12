#include <Arduino.h>

#include <InterruptEncoder.h>
InterruptEncoder encoder;
 
// Pins
const int M1 = 12;
const int M2 = 11;
const int SENSOR_PIN = A4;
const int Enc_A = 4;
const int Enc_B = 3;
const int dt = 1; //the time for the sampling
long last_pulses = 0;
int pulse_per_rotation = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  encoder.attach(Enc_A, Enc_B);
}
 
void loop() {
  long pulses = encoder.read(); //read is the cumulative total so deduct
  Serial.println(pulses);
}