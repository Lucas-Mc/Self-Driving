/*
 *  Code to read the Ultrasonic Sensors and control the motor capabilities through various controllers
 */
 
// #include <PID_v1.h>
#include "Wire.h" // This library allows you to communicate with I2C devices.

// Defines the pin numbers
#define TRIG_PIN_C A0
#define ECHO_PIN_C A1

// Motor driver pins for H-bridge voltage outputs
#define EN1 2
#define EN2 3
#define EN3 4
#define EN4 5

// Motor driver pins for PWM
#define ENA 10
#define ENB 11

// For the acquisition of data
#define TIME_STEP 10
#define DELAY_TIME 3
#define ACQ_SPEED 100

// Set shortcuts for specific movements
#define STOP 0
#define FORWARD 1
#define BACKWARD 2

// Define the PWM values for the motors (what percent [x/255] of max velocity will they go?)
#define PWM_A 55  // 255
#define PWM_B 55  // 255

// Ultrasonic sensor range and follow specs
#define RANGE 400
#define FOLLOW_DISTANCE 6 

// Should we calculate and display the values from the accelerometers and motors?
bool show_accel = true;
bool show_motor = false;

String controller_setting = "PID"; // "bang_bang" -- "P" -- "PI" -- "PID"

// PID constants
// P
double kp1 = 14;
double ki1 = 0;
double kd1 = 0;
// PI
double kp2 = 14;
double ki2 = 0.1;
double kd2 = 0;
// PID
double kp3 = 14;
double ki3 = 0.1;
double kd3 = 0.4;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double output;
double cumError, rateError;

int dir;
int distance_mat_C[TIME_STEP] = {};
long duration_C, distance_C;
boolean frontwall;
double avg_distance_C; // float

// Accelerometer
const int MPU_ADDR = 0x68;                                  // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z;  // Variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                             // Variables for gyro raw data
int16_t temperature;                                        // Variables for temperature data
char tmp_str[7];                                            // Temporary variable used in convert function
char* convert_int16_to_str(int16_t i) {                     // Converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// Motor min and max PWM values
const int OutMin = -PWM_A;
const int OutMax = PWM_A; 
double lastInput;

// This is the main part of the code
void setup() 
{
  Serial.begin(9600);           // Starts the serial communication
  Serial.print("fish");
  if (show_motor)
  {
    pinMode(TRIG_PIN_C, OUTPUT);  // Sets the trigPin as an Output
    pinMode(ECHO_PIN_C, INPUT);   // Sets the echoPin as an Input

    duration_C = 0;
    distance_C = 0;
  }
  Serial.print("fish");
  Serial.print(show_accel);
  // For the accelerometers
  if (show_accel)
  {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B);                 // PWR_MGMT_1 register
    Wire.write(0);                    // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }

  // Get the next TIME_STEP integers at DELAY_TIME intervals and return as an array to be processed
  // Could this part be put into the void loop() function??
  while (true)
  {

    if (show_motor)
    {
      Serial.println("");
      Serial.print("Center\n");
      
      analogWrite(ENA , PWM_A);
      analogWrite(ENB , PWM_B);

      get_distance_C(TIME_STEP,DELAY_TIME);
      Serial.print("Avg_Distance = ");Serial.println(avg_distance_C);
    }

    // For the accelerometers
    if (show_accel)
    {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);                       // Starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
      Wire.endTransmission(false);            // The parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
      Wire.requestFrom(MPU_ADDR, 7*2, true);  // Request a total of 7*2=14 registers
    
      // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
      accelerometer_x = Wire.read()<<8 | Wire.read(); // Reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
      accelerometer_y = Wire.read()<<8 | Wire.read(); // Reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
      accelerometer_z = Wire.read()<<8 | Wire.read(); // Reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
      temperature = Wire.read()<<8 | Wire.read();     // Reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
      gyro_x = Wire.read()<<8 | Wire.read();          // Reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
      gyro_y = Wire.read()<<8 | Wire.read();          // Reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
      gyro_z = Wire.read()<<8 | Wire.read();          // Reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    
      // Print out the data
      Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
      Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
      Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
      // The following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
      Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
      Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
      Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
      Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
      Serial.println();
    }

    if (show_motor)
    {
      // This is where the controller code should go
      if (controller_setting == "bang_bang")
      {
        
        if (avg_distance_C > FOLLOW_DISTANCE && avg_distance_C < RANGE)       // Range is 400 cm, otherwise bogus values
        {
          setDirection(FORWARD);
          Serial.print("FORWARD");
        }
        else if (avg_distance_C == FOLLOW_DISTANCE)
        {
          setDirection(STOP);
          Serial.print("STOP");
        }                    
        else if (avg_distance_C < FOLLOW_DISTANCE || avg_distance_C > RANGE)
        {
          setDirection(BACKWARD);
          Serial.print("BACKWARD");
        }
        
      }
      else if (controller_setting == "P") 
      {

        output = computePID(avg_distance_C, kp1, ki1, kd1);
        Serial.print("Output = ");Serial.println(output);

        drive(output);

      }
      else if (controller_setting == "PI") 
      {

        output = computePID(avg_distance_C, kp2, ki2, kd2);
        Serial.print("Output = ");Serial.println(output);

        drive(output);

      }
      else if (controller_setting == "PID") 
      {

        output = computePID(avg_distance_C, kp3, ki3, kd3);
        Serial.print("Output = ");Serial.println(output);

        drive(output);
        
      }
    }

    delay(ACQ_SPEED);  // How often should the sensor be checked?

  }

}

// Required function for Arduino to work, not need for us though refactoring might use this 
void loop()
{
}

// Get the input distances from the Ultrasonic sensors
void get_distance_C(float time_step, float delay_time)
{ 

  for (int i = 0; i < time_step; i++)
  {
    // Clears the trigPin
    digitalWrite(TRIG_PIN_C, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro-seconds
    digitalWrite(TRIG_PIN_C, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_C, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in micro-seconds
    duration_C = pulseIn(ECHO_PIN_C, HIGH);
    // Calculating the distance
    distance_C = duration_C*0.034/2;
    distance_mat_C[i] = distance_C;
    Serial.print(distance_C);
    Serial.print(" ");
    delay(delay_time);
  }

  int sum = 0;
  int temp = 0;

  for (int i = 0; i < time_step; i++) 
  {
    temp = distance_mat_C[i];
    sum += temp;
  }

  avg_distance_C = sum / time_step; 

}

// Change the mode the motors are in
void setDirection(int dir) 
{

  if (dir == FORWARD) 
  {
    digitalWrite(EN1, LOW);   // Left wheel forward
    digitalWrite(EN2, HIGH);
    digitalWrite(EN3, LOW);   // Right wheel forward
    digitalWrite(EN4, HIGH);
  }
  else if (dir == BACKWARD) 
  {
    digitalWrite(EN1, HIGH);  // Left wheel forward
    digitalWrite(EN2, LOW);
    digitalWrite(EN3, HIGH);  // Right wheel forward
    digitalWrite(EN4, LOW);
  }
  else if (dir == STOP) 
  {
    digitalWrite(EN1, HIGH);  // Left wheel forward
    digitalWrite(EN2, HIGH);
    digitalWrite(EN3, HIGH);  // Right wheel forward
    digitalWrite(EN4, HIGH);
  }

}

// Drive the motors
void drive(int speed)
{
  if (speed > 0)
  {
    Fwd((byte)(abs(speed)));
    Fwd((byte)(abs(speed)));
  }
  else
  {
    Rev((byte)(abs(speed)));
    Rev((byte)(abs(speed)));
  }
}

// Set motor to go forwards at the specified speed
void Fwd(byte spd)
{
  digitalWrite(EN1, LOW);   // Left wheel forward
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, LOW);   // Right wheel forward
  digitalWrite(EN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

// Set motor to go backwards at the specified speed
void Rev(byte spd)
{
  digitalWrite(EN1, HIGH);  // Left wheel forward
  digitalWrite(EN2, LOW);
  digitalWrite(EN3, HIGH);  // Right wheel forward
  digitalWrite(EN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

// Determine the PID output
double computePID(double inp, double kp, double ki, double kd)
{     

  double Input = map(inp, 0, 500, 0, 255);                // Map the input distance to PID-friendly values (8-bit numbers)
  currentTime = millis();                                 // Get current time
  elapsedTime = (double)(currentTime - previousTime);     // Compute time elapsed from previous computation

  error = -FOLLOW_DISTANCE + Input;                       // Determine error
  Serial.print("Input = ");Serial.println(Input);
  Serial.print("Error = ");Serial.println(error);

  cumError += (ki*error);                                 // Integral term
  rateError = Input - lastInput;                          // Derivative term
  double out = (kp*error) + (cumError) - (kd*rateError);  // PID output               

  if (out > OutMax) 
  {
    out = OutMax;
  }
  else if (out < OutMin) 
  {
    out = OutMin;
  }
  if (cumError > OutMax) 
  {
    cumError = OutMax;
  }
  else if (cumError < OutMin) 
  {
    cumError = OutMin;
  }

  lastInput = Input;                                      // Remember current input
  previousTime = currentTime;                             // Remember current time

  return out;                                             // Have function return the PID output

}
