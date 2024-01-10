#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Voltage.h>
#include <Wire.h>

#include <Arduino.h>
#include <Adafruit_MPU6050.h>  
#include <ESP32Servo.h>

#pragma once

extern int PWM;

extern byte X_value;
extern byte Y_value;

extern byte leftB;
extern byte rightB;

extern float PRate;
extern float IRate;
extern float DRate;

extern float PAngle;
extern float IAngle;
extern float DAngle;

void init_ESPNOW_Slave();
void sendingData();
void Print_PID_Value ();
void TimeCount();