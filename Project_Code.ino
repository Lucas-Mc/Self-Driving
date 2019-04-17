/*
 *  Code to read the Ultrasonic Sensors and control the motor capabilities through various controllers
 */
 
// #include <PID_v1.h>

// Defines the pin numbers
#define TRIG_PIN_C A0
#define ECHO_PIN_C A1

// Motor driver pins for H-bridge voltage outputs
#define EN1 = 2;
#define EN2 = 3;
#define EN3 = 4;
#define EN4 = 5;

// Motor driver pins for PWM
#define ENA = 10;
#define ENB = 11;

// For the acquisition of 
#define TIME_STEP 10
#define DELAY_TIME 3

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

// Define the PWM values for the motors (what percent [x/255] of max velocity will they go?)
#define PWM_A 255
#define PWM_B 255

#define RANGE = 400;
#define FOLLOW_DISTANCE = 4; 

//Define Variables we'll be connecting to
//double Setpoint, Input, Output;
 
//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

char controller_setting[] = "bang_bang"; // "bang_bang" -- "P" -- "PI" -- "PID"

int dir;
int distance_mat_C[TIME_STEP] = {};
long duration_C, distance_C;

boolean frontwall;

float avg_distance_C;

// This is the main part of the code
void setup() 
{

  pinMode(TRIG_PIN_C, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHO_PIN_C, INPUT);   // Sets the echoPin as an Input
  Serial.begin(9600);           // Starts the serial communication

  duration_C = 0;
  distance_C = 0;
  
  // Get the next TIME_STEP integers at DELAY_TIME intervals and return as an array to be processed
  // Could this part be put into the void loop() function??
  while (true)
  {

    // Always try to go forward, the controller will handle the rest
    setDirection(FORWARD);

    Serial.print("Center\n");

    for (int i = 0; i < TIME_STEP; i++)
    {
      Serial.print(distance_mat_C[i]);
      Serial.print(" ");
    }

    Serial.println("");
    Serial.println(avg_distance_C);

    analogWrite(ENA , PWM_A);
    analogWrite(ENB , PWM_B);

    // This is where the controller code should go
    if (controller_setting == "bang_bang")
    {

      get_distance_C(TIME_STEP,DELAY_TIME);
      
      if (avg_distance_C > FOLLOW_DISTANCE && avg_distance_C < RANGE) // Range is 400 cm, otherwise bogus values
      {
        setDirection(FORWARD);
        Serial.print("FORWARD");
      }
          
      if (avg_distance_C < FOLLOW_DISTANCE || avg_distance_C > RANGE)
      {
        setDirection(BACKWARD);
        Serial.print("BACKWARD");
      }

      delay(10);  // How often should the sensor be checked?
       
    }
    // else if (controller_setting == "P") {}
    // else if (controller_setting == "PI") {}
    // else if (controller_setting == "PID") { 
    //  turn the PID on
    //  myPID.SetMode(AUTOMATIC);
    //  myPID.Compute();
    // }

  }

}

// Required function for Arduino to work, not need for us though refactoring might use this 
void loop()
{
}

// Get the input distances from the Ultrasonic sensors
void get_distance_C(float time_step, float delay_time)
{ 

  for (int i=0; i<time_step; i++)
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
