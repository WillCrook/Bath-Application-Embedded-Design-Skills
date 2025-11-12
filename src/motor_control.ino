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
int pulse_per_rotation = 64;
int gear_ratio = 50;
 
void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  encoder.attach(Enc_A, Enc_B);
}
 
void loop() {
  int sensorValue = analogRead(SENSOR_PIN);

  if (sensorValue < 1900) {                
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    Serial.println("Forward");
  } else if (sensorValue < 2200) {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    Serial.println("Halt");
  } else {                                  
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    Serial.println("Backward");
  }
  
  
  long pulses = encoder.read(); //read is the cumulative total so deduct
  long delta_pulses = pulses - last_pulses;
  last_pulses = pulses;
 
  Serial.println(((delta_pulses/pulse_per_rotation)*(60/dt))/gear_ratio);
  delay(dt*1000);
}