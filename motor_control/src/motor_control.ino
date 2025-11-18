#include <Arduino.h>

#include <InterruptEncoder.h>
InterruptEncoder encoder;
 
// Pins
const int M1 = 10; 
const int M2 = 11;
const int sensor_pin =  A7; //potentiometer pin
const int Enc_A = 12;
const int Enc_B = 9;
const int button_F = 6;  //forward
const int button_B = 5;  //backward

//config
const float dt = 0.01; //sampling time in seconds
long last_pulses = 0;
float pulse_per_rotation = 64;
float gear_ratio = 50;
float RPM = 0;
char mode = 'Dh'; //set mode for task C, D, E etc.

void setup() {

  switch (mode) {
    case 'C':
      //Task C Setup Code
      Serial.begin(9600);
      pinMode(M1, OUTPUT);
      pinMode(M2, OUTPUT);
      encoder.attach(Enc_A, Enc_B);
      break;
      
    case 'D':
      //Task D Setup Code
      pinMode(M1, OUTPUT);
      pinMode(M2, OUTPUT);
      pinMode(button_F, INPUT_PULLUP);
      pinMode(button_B, INPUT_PULLUP);
      Serial.begin(9600);
      encoder.attach(Enc_A, Enc_B);
      break;

    
    case 'E':
    //Task E Setup Code
      pinMode(M1, OUTPUT);
      pinMode(M2, OUTPUT);
      pinMode(button_F, INPUT_PULLUP);
      pinMode(button_B, INPUT_PULLUP);
      Serial.begin(9600);
      encoder.attach(Enc_A, Enc_B);

    break;

  }
  
}
 
void loop() {
  switch (mode) {
    case 'C': {
      //Task C  Code
      int sensorValue = analogRead(sensor_pin);
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
      RPM = ((delta_pulses/pulse_per_rotation)*(60/dt))/gear_ratio;
      
      //OUTPUT
      Serial.println("RPM: " + String(RPM));
      Serial.println("Trimmer Value: " + String(sensorValue));

      //sampling delay
      delay(dt * 1000);
      break;
    }

    case 'D': {
      //Task D Code
      int trimmer_value = analogRead(sensor_pin);
      int pwm_value = map(trimmer_value, 0, 4095, 0, 255);

      bool fPressed = digitalRead(button_F) == LOW;
      bool bPressed = digitalRead(button_B) == LOW;

      if (fPressed) {
      analogWrite(M1, pwm_value);
      digitalWrite(M2,  LOW);
      Serial.println("Forward");
      }
      else if (bPressed) {
      digitalWrite(M1, LOW);
      analogWrite(M2, pwm_value);
      Serial.println("Backward");
      }
      else {
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
      Serial.println("Trimmer Value: " + String(trimmer_value));

      delay(dt*1000); //sampling delay
      break;
    }

    
    case 'E': {
    //PID constants
    const float Kp = 2.0;
    const float Ki = 0.5;
    const float Kd = 0.1;

    static float integral = 0;
    static float previous_error = 0;

    //Read target from trimmer
    int trimmer_value = analogRead(sensor_pin);
    float target_angle = map(trimmer_value, 0, 4095, -180, 180);

    //Read actual position from encoder (convert pulses to degrees)
    long pulses = encoder.read();
    float actual_angle = (pulses / (pulse_per_rotation * gear_ratio)) * 360.0;
    long delta_pulses = pulses - last_pulses;
    last_pulses = pulses;
    RPM = (delta_pulses / (pulse_per_rotation * gear_ratio)) * (60.0 / dt);

    //PID calculations
    float error = target_angle - actual_angle;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    //Limit output to PWM range
    int pwm_value = constrain(abs(output), 0, 255);

    //Apply motor direction
    if (output > 0) {
        analogWrite(M1, pwm_value);
        digitalWrite(M2, LOW);
    } else if (output < 0) {
        digitalWrite(M1, LOW);
        analogWrite(M2, pwm_value);
    } else {
        analogWrite(M1, 0);
        analogWrite(M2, 0);
    }

    //Serial output
    Serial.print("Target: "); Serial.print(target_angle);
    Serial.print("  Actual: "); Serial.print(actual_angle);
    Serial.print("  RPM: "); Serial.println(RPM);

    delay(dt * 1000);
    break;
}

  }
}