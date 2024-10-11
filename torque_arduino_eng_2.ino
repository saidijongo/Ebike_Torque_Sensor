/******************************************************************
  Description : Arduino, e-bike Torque/PAS to throttle control
  Torque sensor : AXLETORQUEBB68MM (0.8-3.7V, 75Nm max, 44mV/Nm)
******************************************************************/

#include <RunningMedian.h>   // Library to handle median filtering

// --- Pin Definitions ---
const int PIN_TORK = A1;      // Analog pin for torque sensor input
const int PIN_PWM =  3;       // PWM output to throttle control
const int PIN_PAS_IN =  2;    // PAS (Pedal Assist System) input
const int PIN_PAS_OUT = 4;    // Optional PAS output (replication)
const int PIN_LED = 13;       // LED pin for status indication
const int PIN_DEBUG = 5;      // Debug pin for oscilloscope

// --- Constants for Calibration ---
const int NB_MAGNETS =  32;   // Number of magnets in PAS sensor

// Voltage reference (Arduino 5V)
const float VREF = 5.00; 

// Throttle signal settings (PWM)
const float FV_TR_SLEEP = 0.80;  // Throttle at rest (0.8V)
const float FV_TR_MIN = 1.00;    // Minimum throttle voltage
const float FV_TR_MAX = 3.60;    // Maximum throttle voltage

// Torque sensor settings
const float FV_TORK_MIN = 0.80;    // Minimum torque sensor voltage (no load)
const float FV_TORK_MAX = 3.70;    // Maximum torque sensor voltage (75 Nm)

// --- Computed Constants ---
const int TR_SLEEP = round(FV_TR_SLEEP / VREF * 256); 
const int TR_MIN = round(FV_TR_MIN / VREF * 256);
const int TR_MAX = round(FV_TR_MAX / VREF * 256);
const int TORK_MIN = round(FV_TORK_MIN / VREF * 1024);  
const int TORK_MAX = round(FV_TORK_MAX / VREF * 1024);

// Global Variables
unsigned int high_tork = 0;  // Current torque value (ADC)
unsigned int pwm_out = 0;    // PWM output value

// Running median filter
RunningMedian samples = RunningMedian(3);  // Use 3 samples for median filtering

void setup() {
  Serial.begin(115200);  // Start serial communication for debugging

  // Setup pin modes
  pinMode(PIN_TORK, INPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_PAS_IN, INPUT_PULLUP); 
  pinMode(PIN_PAS_OUT, OUTPUT);  
  pinMode(PIN_LED, OUTPUT); 
  pinMode(PIN_DEBUG, OUTPUT);
  
  // Interrupt for PAS sensor
  attachInterrupt(digitalPinToInterrupt(PIN_PAS_IN), isr_pas, CHANGE);
  
  // Initialize outputs
  digitalWrite(PIN_LED, LOW);  
  digitalWrite(PIN_PAS_OUT, LOW);  
  turnOff(); // Ensure throttle is off at startup
}

void loop() {
  digitalWrite(PIN_DEBUG, HIGH); // Debug signal for oscilloscope
  
  // Read the torque sensor value (average of 10 samples)
  unsigned int adc = readADC(PIN_TORK, 10);  

  // Process torque sensor data
  high_tork = constrain(adc, TORK_MIN, TORK_MAX);  // Limit torque within the range
  
  // Map the torque to a throttle signal (PWM)
  pwm_out = map(high_tork, TORK_MIN, TORK_MAX, TR_MIN, TR_MAX); 
  pwm_out = constrain(pwm_out, TR_SLEEP, TR_MAX);  // Constrain the PWM signal
  
  // Output the PWM signal
  analogWrite(PIN_PWM, pwm_out); 
  
  // LED indicates throttle state (ON/OFF)
  digitalWrite(PIN_LED, HIGH);

  digitalWrite(PIN_DEBUG, LOW);  // Debug signal end
}

// Interrupt Service Routine for PAS sensor
void isr_pas() {
  // PAS sensor interrupt logic (e.g., counting pulses for RPM)
  // This will handle pedal assist signal processing.
}

// Function to read the ADC with averaging
int readADC(int adcPin, int samples) {
  int reading = 0;
  for (int i = 0; i < samples; i++) {
    reading += analogRead(adcPin);
  }
  return round(reading / samples);
}

// Function to turn off the throttle
void turnOff() {
  analogWrite(PIN_PWM, TR_SLEEP);  // Set throttle to minimum
  digitalWrite(PIN_LED, LOW);      // Turn off LED
}
