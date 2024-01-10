#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>  
#include <ESP32Servo.h>

#pragma once

extern float PID_output_roll;
extern float PID_output_pitch;
extern float InputThrottle;
extern float MotorInput4, MotorInput1, MotorInput2, MotorInput3;
extern float InputRoll, InputThrottle, InputPitch, InputYaw;
extern float PIDOutput;
extern float DesiredRateRoll, DesiredRateYaw;
extern float RateRoll, RatePitch;
extern float RateCalibrationRoll, RateCalibrationPitch;

extern float KalmanAngleRoll, KalmanAnglePitch;

extern float AngleRoll;
extern float AnglePitch;
// extern uint32_t LoopTimer;
// setup functions//

void trans();
void calibration_measurement();
void init_ESC();
void pid_equation(float Error, float Rate, float Angle, float P , float I, float D, float PrevError, float PrevIterm, float PrevRate, float PrevPterm, char PIDmode, char PIDdir);

void system_setup();

// loop functions//
void corrected_values();
void kalman_1d_roll();
void kalman_1d_pitch();
void value_update();
float ReceiveThrottleInput();

void pid_equation_angleroll();
void pid_equation_anglepitch();
void pid_equation_rateroll();
void pid_equation_ratepitch();
void pid_equation_rateyaw();
void calibrate();
void checkInputController();
void gyro_compensate();
void SerialDataPrint();
void control_throttle();