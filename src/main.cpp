#include <Adafruit_MotorShield.h>
#include <AutoPID.h>
#include <Wire.h>
#include <LowPass.h>

// Initial direction for motors
int lastDirection = FORWARD;  // Assuming FORWARD is the initial direction

// Create Adafruit Motor Shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);  // Create Motor Shield object with I2C address 0x60
// Define Motors
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);  // Motor 1
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);  // Motor 2
Adafruit_DCMotor* motor3 = AFMS.getMotor(3);  // Motor 3
Adafruit_DCMotor* motor4 = AFMS.getMotor(4);  // Motor 4

// -------- RPM Sensor declarations -----------
const byte sensor1 = A0;
const byte sensor2 = A1;
const byte sensor3 = A2;
const byte sensor4 = A3;

volatile int countA, countB, countC, countD;  // Variables to count RPM pulses

volatile uint8_t
    oldPortVal;  // Variable to store previous port state for detecting changes

unsigned long lastMillis =
    0;  // Variable to store last millis for RPM calculation
// ----------------------------------------------

// ------------I2C variable declarations------------------
volatile uint8_t command = 0;  // Received command variable
volatile bool i2cDataReceived =
    false;  // Flag to indicate if I2C data has been received

// Struct for receiving Data:
struct CommandData {
  uint8_t registerAddr;  // throwaway variable to store register address (could be used later for different functions?)
  int16_t m1;            // desired RPM for each motor.
  int16_t m2;
  int16_t m3;
  int16_t m4;
} commandData;

// Filter instances for each Motor

LowPass<2> lp_filters[4] = {
    LowPass<2>(0.8, 1e3, true),  // Parameters for lp_Motor_A
    LowPass<2>(0.8, 1e3, true),  // Parameters for lp_Motor_B
    LowPass<2>(0.8, 1e3, true),  // Parameters for lp_Motor_C
    LowPass<2>(0.8, 1e3, true)   // Parameters for lp_Motor_D
};

// PID control parameters
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0.15
#define KI 1
#define KD 1

// Define input, setpoint, and output variables for PID controllers of each
// motor
double input1, input2, input3, input4, setpoint1, setpoint2, setpoint3, setpoint4, output1, output2, output3, output4;

// Create PID controllers for each motor
AutoPID motor1PID(&input1, &setpoint1, &output1, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor2PID(&input2, &setpoint2, &output2, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor3PID(&input3, &setpoint3, &output3, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor4PID(&input4, &setpoint4, &output4, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// int motorSpeeds[4] = {0, 0, 0, 0}; // Array to store motor speeds for each
// motor

void setup() {
  AFMS.begin();  // Initialize the Motor Shield
  Wire.begin(0x8);  // Initialize the I2C communication address 0x8
  Wire.onReceive(receiveEvent); // Call receiveEvent when data is received
  Serial.begin(9600);  // Initialize Serial comms

  // Set sensor pins to INPUT. INPUT_PULLUP caused weird behavior. this works I
  // guess.
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  // Pin Register stuff, enable Pin Change Interrupt on pin A0 - A3 basically.
  PCICR |= (1 << PCIE1);  // enable PCINT[14:8] (PCIE1)
  PCMSK1 |= (1 << PCINT3) | (1 << PCINT2) | (1 << PCINT1) | (1 << PCINT0);  // enable PCINT[3:0] (PCMSK1)
  // initialize the RPM counter
  oldPortVal = PINC & B00001111;

  // Set PID time step and initial setpoints
  motor1PID.setTimeStep(50);
  motor2PID.setTimeStep(50);
  motor3PID.setTimeStep(50);
  motor4PID.setTimeStep(50);
  setpoint1 = 0;
  setpoint2 = 0;
  setpoint3 = 0;
  setpoint4 = 0;

  Serial.println("Arduino initialized");
}

void loop() {

  // Set speed of all motors to the last speed value
  motor1->setSpeed((int)output1);  // Set speed for motor 1
  motor2->setSpeed((int)output2);  // Set speed for motor 2
  motor3->setSpeed((int)output3);  // Set speed for motor 3
  motor4->setSpeed((int)output4);  // Set speed for motor 4

  // Set direction of motors
  motor1->run(lastDirection);
  motor2->run(lastDirection);
  motor3->run(lastDirection);
  motor4->run(lastDirection);

  // Calculate RPM values
  float* rpmValues = calculateRPMs(countA, countB, countC, countD);

  // Update low-pass filters with new sensor readings
  float smoothedRPMs[4];
  for (int i = 0; i < 4; i++) {
    smoothedRPMs[i] = lp_filters[i].filt(
        rpmValues[i]);  // Applies lowpassfilter to the RPM array and writes it
                        // to a new array called smoothedRPMs
  }

  // Print setpoints and smoothed RPMs for debugging
  Serial.print("Setpoint1:");
  Serial.print(setpoint1);
  Serial.print(",");
  // Serial.print("Motor_1:");
  // Serial.print(rpmValues[0]);
  // Serial.print(",");
  Serial.print("Smoothed_1:");
  Serial.println(smoothedRPMs[0]);
  Serial.print("Setpoint2:");
  Serial.print(setpoint2);
  Serial.print(",");
  // Serial.print("Motor_2:");
  // Serial.print(rpmValues[1]);
  // Serial.print(",");
  Serial.print("Smoothed_2:");
  Serial.println(smoothedRPMs[1]);
  Serial.print("Setpoint3:");
  Serial.print(setpoint3);
  Serial.print(",");
  // Serial.print("Motor_3:");
  // Serial.print(rpmValues[2]);
  // Serial.print(",");
  Serial.print("Smoothed_3:");
  Serial.println(smoothedRPMs[2]);
  Serial.print("Setpoint4:");
  Serial.print(setpoint4);
  Serial.print(",");
  // Serial.print("Motor_4:");
  // Serial.print(rpmValues[3]);
  // Serial.print(",");
  Serial.print("Smoothed_4:");
  Serial.println(smoothedRPMs[3]);

  // Update PID inputs
  input1 = smoothedRPMs[0];
  input2 = smoothedRPMs[1];
  input3 = smoothedRPMs[2];
  input4 = smoothedRPMs[3];

  // Run PID controllers
  motor1PID.run();
  motor2PID.run();
  motor3PID.run();
  motor4PID.run();

  delay(100);
}

// Interrupt Service Routine for pin change black magic and RPM counting
ISR(PCINT1_vect) {
  // Detect which pins have changed since the last interrupt
  uint8_t changedPins = (PINC & B00001111) ^ oldPortVal;
  if (!changedPins) {
    return;  // If no pins have changed, exit ISR
  }

  // Increment counts based on which pins have changed
  if (changedPins & B00000001) {  // If pin A0 changed
    countA++;
  }
  if (changedPins & B00000010) {  // If pin A1 changed
    countB++;
  }
  if (changedPins & B00000100) {  // If pin A2 changed
    countC++;
  }
  if (changedPins & B00001000) {  // If pin A3 changed
    countD++;
  }

  oldPortVal =
      (PINC & B00001111);  // Update oldPortVal for the next ISR iteration
}

// Function to calculate RPMs from pulse counts
float* calculateRPMs(int cA, int cB, int cC, int cD) {
  static float rpms[4];                 // Static array to store RPM values
  int totalCounts = cA + cB + cC + cD;  // Total counts from all sensors
  float factor =
      60000.0 / (millis() - lastMillis);  // Factor to convert counts to RPM
  // Calculate RPM for each sensor and store in the rpms array
  rpms[0] = (cA / 40.0) * factor;  // Convert counts to RPM for sensor 1
  rpms[1] = (cB / 40.0) * factor;  // Convert counts to RPM for sensor 2
  rpms[2] = (cC / 40.0) * factor;  // Convert counts to RPM for sensor 3
  rpms[3] = (cD / 40.0) * factor;  // Convert counts to RPM for sensor 4
  lastMillis = millis();  // Update lastMillis for the next RPM calculation
  countA = 0;             // Reset counts for sensor 1
  countB = 0;             // Reset counts for sensor 2
  countC = 0;             // Reset counts for sensor 3
  countD = 0;             // Reset counts for sensor 4
  return rpms;            // Return the array of RPM values
}

void receiveEvent(int howMany) {
  if (howMany >= sizeof(commandData)) {
    uint8_t *bytes = (uint8_t*)&commandData;
    for (size_t i = 0; i < sizeof(commandData); i++) {
      bytes[i] = Wire.read();
    }
    i2cDataReceived = true;
  }
}
