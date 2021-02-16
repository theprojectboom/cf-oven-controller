#include "pid.h"
#include "LiquidCrystal_I2C.h"


// Pin numbers are arbitrary - change it before flashing
#define thermocouple_pin A6
#define triac_in_pin A7
#define triac_out_pin A8
#define LCD_PRESENT true

// Set the LCD address to 0x27
LiquidCrystal_I2C lcd(0x27,20,4);

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
  Serial.print("Serial started.\n");

  if (LCD_PRESENT) {
    // Initialize LCD, wait for 5 sec
    Serial.print("Initializing LCD.\n");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("Hello, welcome!");
    Serial.print("LCD Initialized.\n You should see \"Hello, welcome!\" for 5 seconds.");
    delay(5000);
  }
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

  if (LCD_PRESENT) {
    // Refresh the LCD screen
    lcd.clear(); // Clear the screen
    lcd.setCursor(3,0);
    lcd.print("Current value: ");
    lcd.print(test_current_val);
    lcd.setCursor(3,1);
    lcd.print("Control output: ");
    lcd.print(control_output);
  }
}
