#include <Arduino.h>
#include <InterruptEncoder.h>

InterruptEncoder encoder;
 
// Pins
const int M1 = 12; 
const int M2 = 11;
const int sensor_pin = A4;
const int Enc_A = 4;
const int Enc_B = 3;
const int button_F = 7;  // Forward
const int button_H = 8;  // Halt
const int button_B = 9;  // Backward

//config
const int dt = 1; //sampling time
long last_pulses = 0;
int pulse_per_rotation = 64;
int gear_ratio = 50;
int RPM = 0;

void setup() {
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(button_F, INPUT_PULLUP);
  pinMode(button_H, INPUT_PULLUP);
  pinMode(button_B, INPUT_PULLUP);
  Serial.begin(9600);
  encoder.attach(Enc_A, Enc_B);
}
 
void loop() {
  int timmer_value = analogRead(sensor_pin);
  int pwm_value = map(timmer_value, 0 , 4095, 0, 255);

  bool fPressed = digitalRead(button_F) == LOW;
  bool hPressed = digitalRead(button_H) == LOW;
  bool bPressed = digitalRead(button_B) == LOW;

  if (fPressed) {
  analogWrite(M1, pwm_value);
  digitalWrite(M2, LOW);
  Serial.println("Forward");
  }
  else if (bPressed) {
  analogWrite(M1, pwm_value);
  digitalWrite(M2, HIGH);
  Serial.println("Backward");
  }
  else if (hPressed) {
  analogWrite(M1, 0);
  digitalWrite(M2, LOW);
  Serial.println("Halt");
  }

  long pulses = encoder.read(); //read is the cumulative total so deduct
  long delta_pulses = pulses - last_pulses;
  last_pulses = pulses;
  RPM = ((delta_pulses/pulse_per_rotation)*(60/dt))/gear_ratio;
  
  //OUTPUT
  Serial.println("RPM: " + String(RPM));
  Serial.println("Trimmer Value: " + String(timmer_value));

  delay(dt*1000); //sampling delay
  
}