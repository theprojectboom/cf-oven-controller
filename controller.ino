#include "pid.h"
#include "thermocouple.h"
#include "LiquidCrystal_I2C.h"


// Pin numbers are arbitrary - change it before flashing
#define LCD_PRESENT true
#define LCD_ADDRESS 0x27
#define TEMP_CALIBRATION true

// Setup the LCD Matrix
LiquidCrystal_I2C lcd(LCD_ADDRESS,20,4);

// Setup the thermocouples, use farenheight
// For now, use one thermocouple for ease of setup and testing
Thermocouple t1(0, false, 0.5);
Thermocouple t2(0, false, 0.5);
Thermocouple t3(0, false, 0.5);
Thermocouple t4(0, false, 0.5);
Thermocouple t5(0, false, 0.5);

// PID object params
double dt = 0.1;          // loop interval time
double max_out = 1;     // maximum allowable output from pid
double min_out = -1;    // minimum allowable output from pid
double Kp = 0.01;          // proportional gain
double Kd = 0.01;         // derivative gain
double Ki = 0.5;          // integral gain

// Create pid object with params
PID pid = PID(dt, max_out, min_out, Kp, Kd, Ki);

// Variables for test/debug
double test_setpoint = 60;

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

  if (TEMP_CALIBRATION) {
    // Log the temperature to serial output
    // We'll save this to an output file on a companion computer
    Serial.print("T1, ");
    Serial.print(t1.read());
    Serial.println("");

    Serial.print("T2, ");
    Serial.print(t2.read());
    Serial.println("");

    Serial.print("T3, ");
    Serial.print(t3.read());
    Serial.println("");

    Serial.print("T4, ");
    Serial.print(t4.read());
    Serial.println("");

    Serial.print("T5, ");
    Serial.print(t5.read());
    Serial.println("");
  }

  // For now, we'll average all the temp readings in F
  double avg_temp = (t1.read() + t2.read() + t3.read() + t4.read() + t5.read()) / 5;

  // Calculate this interval's control output
  double control_output = pid.calculate(test_setpoint, avg_temp);

  if (LCD_PRESENT) {
    // Refresh the LCD screen
    lcd.clear(); // Clear the screen
    lcd.setCursor(3,0);
    lcd.print("Curr Temp (F): ");
    lcd.print(avg_temp);
    lcd.setCursor(3,1);
    lcd.print("Ctrl Ouput: ");
    lcd.print(control_output);
  }

  // Delay the loop for human readable debugging
  delay(300);
}
