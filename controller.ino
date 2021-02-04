#include "pid.h"

// Pin numbers are arbitrary - change it before flashing
#define thermocouple_pin A6
#define triac_in_pin A7
#define triac_out_pin A8


// Create pid object with params
double dt = 0.1;          // loop interval time
double max_out = 100;     // maximum allowable output from pid
double min_out = -100;    // minimum allowable output from pid
double Kp = 0.1;          // proportional gain
double Kd = 0.01;         // derivative gain
double Ki = 0.5;          // integral gain
PID pid = PID(dt, max_out, min_out, Kp, Kd, Ki);

double test_current_val = 20;
double test_setpoint = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Calculate this interval's control output
  double control_output = pid.calculate(test_setpoint, test_current_val);

  Serial.print("current_val: ");
  Serial.print(test_current_val);
  Serial.print(", control_output: ");
  Serial.print(control_output);
  Serial.println("");
  delay(300);

  // For test, assume the control_output is fully realized
  test_current_val += control_output;
  
}
