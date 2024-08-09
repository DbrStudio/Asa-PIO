#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <Wire.h>

#include <AutoPID.h>
#include <LowPass.h>

#include <TimeLib.h>


struct Vec3 {
  float x,y,z;
};
struct Odom {
  String timestamp;
  float rpms[4],temp;
  Vec3 acceleration, rotation;
};
Adafruit_MPU6050 mpu;

// Communication stuff
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
int motorCommands[4];


// Initial direction for motors
int lastDirection[4] = {FORWARD, FORWARD, FORWARD, FORWARD};

// Create Adafruit Motor Shield object with I2C address 0x60
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(2); 
Adafruit_DCMotor* motor3 = AFMS.getMotor(3); 
Adafruit_DCMotor* motor4 = AFMS.getMotor(4); 


// -------- RPM Sensor -----------
const byte sensor1 = A0;
const byte sensor2 = A1;
const byte sensor3 = A2;
const byte sensor4 = A3;

struct EncoderCounts // Variables to count RPM pulses
{
volatile int countA;
volatile int countB;
volatile int countC;
volatile int countD;  
};

EncoderCounts encoderCounts;

volatile uint8_t oldPortVal;  // previous port state for detecting changes

unsigned long lastMillis = 0;
// ----------------------------------------------

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

// Define input, setpoint, and output variables for PID controllers of each motor
double input1, input2, input3, input4, setpoint1, setpoint2, setpoint3, setpoint4, output1, output2, output3, output4;

// Create PID controllers for each motor
AutoPID motor1PID(&input1, &setpoint1, &output1, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor2PID(&input2, &setpoint2, &output2, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor3PID(&input3, &setpoint3, &output3, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID motor4PID(&input4, &setpoint4, &output4, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

bool timeSynced = false;


void readMPU(Vec3 &accel, Vec3 &gyro, float &temp){
  sensors_event_t a, g, temp_event;
  mpu.getEvent(&a, &g, &temp_event);

  accel.x = a.acceleration.x;
  accel.y = a.acceleration.y;
  accel.z = a.acceleration.z;

  gyro.x = g.gyro.x; 
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;

  temp = temp_event.temperature;
}
String getISOTimestamp(){
  char buf[30];
  unsigned long ms = millis() % 1000;
  sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02d.%03luZ", year(), month(), day(), hour(), minute(), second(), ms);
  return String(buf);
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

uint16_t calculateHash(String &data) {
  uint16_t hash = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    hash = ((hash << 5) + hash) + char(data[i]);
  }
  return hash;
}

void sendSerialData(float* rpms) {
  
  Odom odom;
  odom.timestamp = getISOTimestamp();
  readMPU(odom.acceleration, odom.rotation, odom.temp);
  for (unsigned int i = 0; i < 4; i++) {
    odom.rpms[i] = rpms[i];
  }
  
  
  
  String dataString = String(odom.timestamp) + ",m1_" + odom.rpms[0] + ",m2_" + odom.rpms[1] + ",m3_" + odom.rpms[2] + ",m4_" + odom.rpms[3] + 
                      ",acc_x_" + odom.acceleration.x + ",acc_y_" + odom.acceleration.y + ",acc_z_" + odom.acceleration.z + 
                      ",rot_x" + odom.rotation.x + ",rot_y" + odom.rotation.y + ",rot_z" + odom.rotation.z + 
                      ",temp_c_" + odom.temp;
  
  Serial.print(dataString);
  uint16_t hash = calculateHash(dataString);
  
  Serial.println("," + String(hash));
}

void setup(void){
  AFMS.begin();   
  Serial.begin(115200);
  Serial.println(F("ASA, wake up!"));

  Serial.println(F("Serial connected. Waiting for TimeSync."));
  while (!timeSynced){
    if (Serial.available() >= 11) {  
      String input = Serial.readStringUntil('\n');
      if (input.length() == 11 && input.startsWith("T")) {
        setTime(input.substring(1).toInt());
        timeSynced = true;
        Serial.println(F("ACK"));
        delay(200);
      }
    }
  }

  Serial.println(F("Preparing Arduino..."));
  Serial.println(F("Setting pin-mode for motor encoders."));
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  Serial.println(F("Pins set. Enabling Pin Change Interrupt on pin A0-A3."));
  PCICR |= (1 << PCIE1);                                                    // enable PCINT[14:8] (PCIE1)
  PCMSK1 |= (1 << PCINT3) | (1 << PCINT2) | (1 << PCINT1) | (1 << PCINT0);  // enable PCINT[3:0] (PCMSK1)
  // initialize the RPM counter
  oldPortVal = PINC & B00001111;
  Serial.println(F("PCIR enabled, RPM counter initialized."));
  Serial.println(F("Setting PID time step and initializing PID."));
  motor1PID.setTimeStep(50);
  motor2PID.setTimeStep(50);
  motor3PID.setTimeStep(50);
  motor4PID.setTimeStep(50);
  setpoint1 = 0;
  setpoint2 = 0;
  setpoint3 = 0;
  setpoint4 = 0;

  Serial.println(F("MPU6050 Startup."));
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip"));
    while (1) {delay(10);}
  }
  Serial.println(F("MPU6050 Found!"));

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println(F("Accelerometer range set to: +-8G"));
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println(F("Gyro range set to: +- 500 deg/s"));

  Serial.print(F("The time is: "));
  Serial.println(getISOTimestamp());
  Serial.println(F("Arduino Ready, continue in:"));
  for (int i = 3; i > 0; i--) {
    Serial.println(i);
    delay(1000);
  }
  Serial.println(F("Ready to receive data in this format (int8_t): <m1, m2, m3, m4>"));
  delay(100);
  Serial.println(F("Enjoy your ride little robot! <3"));
}

void loop() {

  recvWithStartEndMarkers();
  if (newData == true) {
      parseData();
      newData = false;
  }
  getDirectionAndSpeed();

  float* rpmValues = calculateRPMs(encoderCounts.countA, encoderCounts.countB, encoderCounts.countC, encoderCounts.countD);

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


  delay(50);

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
