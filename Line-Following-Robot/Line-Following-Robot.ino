#include <Arduino.h>
#include "driver/ledc.h"  // Required for ESP32 LEDC PWM

// Motor Driver Pins
#define ENA  5   // PWM para sa Left Motor speed
#define IN1  18  // Left Motor Forward
#define IN2  19  // Left Motor Backward

#define ENB  4   // PWM para sa Right Motor speed
#define IN3  21  // Right Motor Forward
#define IN4  22  // Right Motor Backward

// IR Sensor Pins
#define IR_LEFT  32   // Left IR sensor
#define IR_RIGHT 33   // Right IR sensor

// LED Indicator (pang-debug)
#define LED_PIN 2   // D2 LED sa ESP32

// PWM Configuration
#define PWM_FREQ 5000  
#define PWM_RESOLUTION 8  
#define SPEED 180   // Normal speed (0-255)
#define TURN_SPEED 100  // Lower speed for smooth turn

void setup() {
    Serial.begin(115200);  

    // I-setup ang motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // I-setup ang IR sensor pins
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    // I-initialize ang motors (off)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    // I-attach ang PWM channels sa motor speed control
    ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
}

void moveForward() {
    ledcWrite(ENA, SPEED);
    ledcWrite(ENB, SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnLeft() {
    ledcWrite(ENA, TURN_SPEED);  // Left motor slow
    ledcWrite(ENB, SPEED);       // Right motor normal speed
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(150);  // Short delay for controlled turns

void turnRight() {
    ledcWrite(ENA, SPEED);       // Left motor normal speed
    ledcWrite(ENB, TURN_SPEED);  // Right motor slow
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(150);  // Short delay for controlled turns
}

void stopRobot() {
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void loop() {
    int leftSensor = digitalRead(IR_LEFT);
    int rightSensor = digitalRead(IR_RIGHT);

    Serial.print("Left Sensor: ");
    Serial.print(leftSensor);
    Serial.print(" | Right Sensor: ");
    Serial.println(rightSensor);

    digitalWrite(LED_PIN, HIGH);
    delay(10);
    digitalWrite(LED_PIN, LOW);
    delay(10);

    // *LOGIC: Follow Black Line, Stop on White*
    if (leftSensor == 0 && rightSensor == 0) {  
        stopRobot();  // if both sensor detects white
    }
    else if (leftSensor == 1 && rightSensor == 0) {  
        turnLeft();  //if left sensor detects black
    }
    else if (leftSensor == 0 && rightSensor == 1) {  
        turnRight(); //if right sensor detects black
    }
    else {  
        moveForward();  // if both sensor detects black
    }

    delay(5);  // Reduce delay for faster response
}