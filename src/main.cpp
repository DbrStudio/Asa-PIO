#include <Adafruit_MotorShield.h>
#include <AutoPID.h>
#include <LowPass.h>
#include <SPI.h>
#include <Wire.h>
#include <TimeLib.h>
// #include <MPU6050_6Axis_MotionApps20.h>
// #include <I2Cdev.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Adafruit_MPU6050 mpu;
// MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>


// Initial direction for motors
int lastDirection[4] = {FORWARD, FORWARD, FORWARD, FORWARD};  // Assuming FORWARD is the initial direction

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

struct EncoderCounts
{
volatile int countA;
volatile int countB;
volatile int countC;
volatile int countD;  // Variables to count RPM pulses
};

EncoderCounts encoderCounts;


volatile uint8_t oldPortVal;  // Variable to store previous port state for detecting changes

unsigned long lastMillis = 0;  // Variable to store last millis for RPM calculation
// ----------------------------------------------

// ------------I2C variable declarations------------------
volatile uint8_t command = 0;           // Received command variable
volatile bool i2cDataReceived = false;  // Flag to indicate if I2C data has been received

// Struct for receiving Data:
struct CommandData {
  uint8_t registerAddr;  // throwaway variable to store register address (could be used later for different functions?)
  int8_t m1;             // desired RPM for each motor.
  int8_t m2;
  int8_t m3;
  int8_t m4;
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

// int motorCommands[4] = {0, 0, 0, 0}; // Array to store motor Commands for each
// motor

// Gyroscope object
// Adafruit_MPU6050 mpu;


float* calculateRPMs(int cA, int cB, int cC, int cD);
void receiveEvent(int howMany);
unsigned long lastTime = 0;


const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

int motorCommands[4];

bool timeSynced = false;


void parseData();
void recvWithStartEndMarkers();
void getDirectionAndSpeed();
String getISOTimestamp();
uint16_t calculateHash();
void readGyroData();
void sendSerialData(float* rpms);

void setup(){
  AFMS.begin();                  
  Serial.begin(115200);



  Serial.println("Serial connected. Waiting for TimeSync.");
  while (!timeSynced){
    if (Serial.available() >= 11) {  
      String input = Serial.readStringUntil('\n');
      if (input.length() == 11 && input.startsWith("T")) {
        setTime(input.substring(1).toInt());
        timeSynced = true;
        Serial.println("ACK");
      }
    }
  }

  // Set sensor pins to INPUT. INPUT_PULLUP caused weird behavior. this works I guess.
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  // Pin Register stuff, enable Pin Change Interrupt on pin A0 - A3 basically.
  PCICR |= (1 << PCIE1);                                                    // enable PCINT[14:8] (PCIE1)
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

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: +-8G");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: +- 500 deg/s");
  
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Serial.print("Filter bandwidth set to: 21 Hz");

  Serial.println("Arduino Initialized");
  Serial.print("The time is: ");
  Serial.println(getISOTimestamp());
  Serial.println("Enter data in this format (int8_t) <m1, m2, m3, m4>");
}

void loop() {
  recvWithStartEndMarkers();
    if (newData == true) {
        parseData();
        newData = false;
    }
    getDirectionAndSpeed();

  // Calculate RPM values
  float* rpmValues = calculateRPMs(encoderCounts.countA, encoderCounts.countB, encoderCounts.countC, encoderCounts.countD);

  // Update low-pass filters with new sensor readings
  float smoothedRPMs[4];
  for (int i = 0; i < 4; i++) {
    smoothedRPMs[i] = lp_filters[i].filt(rpmValues[i]);  // Applies lowpassfilter to the RPM array and writes it to a new array called smoothedRPMs
  }
  sendSerialData(smoothedRPMs);

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

  // Set speed of all motors to the last speed value
  motor1->setSpeed((int)output1);
  motor2->setSpeed((int)output2);
  motor3->setSpeed((int)output3);
  motor4->setSpeed((int)output4);

  // Set direction of motors
  motor1->run(lastDirection[0]);
  motor2->run(lastDirection[1]);
  motor3->run(lastDirection[2]);
  motor4->run(lastDirection[3]);

  delay(20);
}

// Interrupt Service Routine for pin change black magic and RPM counting
ISR(PCINT1_vect) {
  // Detect which pins have changed since the last interrupt
  uint8_t changedPins = (PINC & B00001111) ^ oldPortVal;
  if (!changedPins) {
    return;  // If no pins have changed, exit ISR
  }

  if (changedPins & B00000001) {
    encoderCounts.countA++;
  }
  if (changedPins & B00000010) {
    encoderCounts.countB++;
  }
  if (changedPins & B00000100) {
    encoderCounts.countC++;
  }
  if (changedPins & B00001000) {
    encoderCounts.countD++;
  }

  oldPortVal = (PINC & B00001111);  // Update oldPortVal for the next ISR iteration
}

// Function to calculate RPMs from pulse counts
float* calculateRPMs(int cA, int cB, int cC, int cD) {
  static float rpms[4];                              // Static array to store RPM values
// int totalCounts = cA + cB + cC + cD;               // Total counts from all sensors
  float factor = 60000.0 / (millis() - lastMillis);  // Factor to convert counts to RPM
  // Calculate RPM for each sensor and store in the rpms array
  rpms[0] = (cA / 40.0) * factor;  // Convert counts to RPMs
  rpms[1] = (cB / 40.0) * factor;
  rpms[2] = (cC / 40.0) * factor;
  rpms[3] = (cD / 40.0) * factor; 
  lastMillis = millis();           // Update lastMillis for the next RPM calculation
  encoderCounts.countA = 0;                      // Reset counts for sensors
  encoderCounts.countB = 0;                      
  encoderCounts.countC = 0;                      
  encoderCounts.countD = 0;                      
  return rpms;                     
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx;
    strtokIndx = strtok(receivedChars, ",");
    for (int i = 0; i < 4; i++) {
        motorCommands[i] = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ",");
    }
}

void getDirectionAndSpeed() {
  for (int i = 0; i < 4; i++) {
    if (motorCommands[i] > 0) {
      lastDirection[i] = FORWARD;
    } else if (motorCommands[i] < 0) {
      lastDirection[i] = BACKWARD;
    } else {
      lastDirection[i] = RELEASE;
    }
  
   // Set the setpoint to the absolute value of the received data
  switch (i) {
    case 0:
      setpoint1 = abs(motorCommands[i]);
      break;
    case 1:
      setpoint2 = abs(motorCommands[i]);
      break;
    case 2:
      setpoint3 = abs(motorCommands[i]);
      break;
    case 3:
      setpoint4 = abs(motorCommands[i]);
      break;
    }
  }
}

String getISOTimestamp(){
  char buf[30];
  sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", year(), month(), day(), hour(), minute(), second(), (int)millis() % 1000);
  return String(buf);
}

uint16_t calculateHash(String &data) {
  uint16_t hash = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    hash = ((hash << 5) + hash) + char(data[i]);
  }
  return hash;
}

void readGyroData(float &x, float &y, float &z) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  x = g.gyro.x;
  y = g.gyro.y;
  z = g.gyro.z;
}

void sendSerialData(float* rpms) {
  
  float gyroX, gyroY, gyroZ;
  readGyroData(gyroX, gyroY, gyroZ);
  
  String timestamp = getISOTimestamp();
  
  String dataString = "ODO," + timestamp + "," +
                      String(rpms[0]) + "," + String(rpms[1]) + "," +
                      String(rpms[2]) + "," + String(rpms[3]) + "," +
                      String(gyroX, 4) + "," + String(gyroY, 4) + "," +
                      String(gyroZ, 4);
  
  Serial.print(dataString);
  uint16_t hash = calculateHash(dataString);
  
  Serial.println("," + String(hash));
  // Serial.println(">GyroX: " + String(gyroX, 4));
  // Serial.println(">GyroY: " + String(gyroY, 4));
  // Serial.println(">GyroZ: " + String(gyroZ, 4));
}
