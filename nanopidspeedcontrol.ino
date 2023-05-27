// Set the loop interval in microsecond


// Set the proportional, integral, and derivative constants for the PID controller

float kp = 0.8;
float ki = 0.0001; // really low integral is the integral part of this correct?
float kd = 0.001;

const unsigned long debugInterval = 100000;


const unsigned long loopInterval = 1000;
const int maxRPM =60;
// Define the pin numbers for the left motor
const int LeftMotor_Pin1 = 9;
const int LeftMotor_Pin2 = 10;
const int LeftMotor_PWM = 11;
const int LeftMotor_EncA = 2;
const int LeftMotor_EncB = 4;

// Define the pin numbers for the right motor
const int RightMotor_Pin1 = 7;
const int RightMotor_Pin2 = 8;
const int RightMotor_PWM = 6;
const int RightMotor_EncA = 3;
const int RightMotor_EncB = 5;


// Define the encoder resolution, gear ratio, and counts per revolution
const float encoderResolution = 64.0;
const float gearRatio = 131.3/1;
const float encoder_counts_per_revolution = encoderResolution * gearRatio;

// Initialize the motor speeds and encoder positions
float requiredSpeedLeft = 0.0;
float requiredSpeedRight = 0.0;
int encoderPositionLeft =0;
int encoderPositionRight=0;

float integralLeft=0;
float integralRight=0;

int prev_encoder_count_left  = 0;
int prev_encoder_count_right = 0;

float prev_error_left=0;
float prev_error_right=0;


unsigned long previousMillis=0;



void encoderInterrupt(int pin, int &encoderPosition) 
{
    int encB = digitalRead(pin + 1);
    int encDirection = -1;
    if (encB == LOW) 
    {
      encDirection = 1;
    }
    encoderPosition += encDirection;  
}

void setup() 
{
  Serial.begin(115200);
  // Set motor Pins to low to stop bot from moving
  setPWM (LeftMotor_Pin1, LeftMotor_Pin2, LeftMotor_PWM,0);

  
  // Enable the encoder interrupt
  attachInterrupt(digitalPinToInterrupt(Left_encA), [](){encoderInterrupt(Left_encA, encoderPositionLeft);}, RISING);

  pinMode(Right_encA, INPUT);
  pinMode(Right_encB, INPUT);
  // Enable the encoder interrupt
  attachInterrupt(digitalPinToInterrupt(Right_encA), [](){encoderInterrupt(Right_encA, encoderPositionRight);}, RISING);
  
  previousMillis = micros();
}

void setPWM (int motorPin1, int motorPin2, int pwmPin, int pwmValue)
{
  if (pwmValue > 0)
  {  
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

  }
  else if(pwmValue == 0)
  {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);

  }
  else
  {    
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  analogWrite(pwmPin,abs(pwmValue));
}
float currentSpeedLeft = 0;
float currentSpeedRight = 0;

void pidLoop(unsigned long interval)
{


  
  int pwmLeft  = 0;
  if(requiredSpeedLeft !=0)
  {
      // Calculate the error between the desired speed and the actual speed
      currentSpeedLeft = findSpeed(interval,prev_encoder_count_left,encoderPositionLeft);
      
      float errorLeft  = requiredSpeedLeft - currentSpeedLeft;
      prev_encoder_count_left = encoderPositionLeft;
     
      // Calculate the derivative of the error
      float derivativeLeft = (errorLeft - prev_error_left) / interval;
      prev_error_left = errorLeft;
      // Calculate the integral of the error
      integralLeft += errorLeft * interval;
          
      // Calculate the PID output
      pwmLeft = kp * errorLeft + ki * integralLeft + kd * derivativeLeft;
      
      if (pwmLeft>255)
      {
        pwmLeft=255;
      }      
    
  }
  else
  {
    currentSpeedLeft =0;
  }
  
  int pwmRight = 0;
  if(requiredSpeedRight !=0)
  {
      // Calculate the error between the desired speed and the actual speed
      currentSpeedRight = findSpeed(interval,prev_encoder_count_right,encoderPositionRight); 
      
      float errorRight  = requiredSpeedRight - currentSpeedRight;
      prev_encoder_count_right = encoderPositionRight;
    
    // Calculate the derivative of the error
    float derivativeRight = (errorRight - prev_error_right) / interval;
    prev_error_right = errorRight;
    // Calculate the integral of the error
    integralRight += errorRight * interval;
        
    // Calculate the PID output
    pwmRight = kp * errorRight + ki * integralRight + kd * derivativeRight;
    
    if (pwmRight>255)
    {
      pwmRight=255;
    }
    
      
  }
  else
  {
    currentSpeedRight =0;
  }
  
  setPWM(LeftMotor_Pin1,LeftMotor_Pin2,LeftMotor_PWM, pwmLeft);
  setPWM(RightMotor_Pin1,RightMotor_Pin2,RightMotor_PWM,pwmRight);
  
}

float findSpeed(unsigned long time_diff, int prev_encoder_count, int current_encoder_count)
{
    float encoder_count = current_encoder_count - prev_encoder_count;
//    float rpm = (encoder_count / encoder_counts_per_revolution) / (time_diff / (1000000.0 * 600.0));
    float rpm = (encoder_count / encoder_counts_per_revolution) / (time_diff / (1000000.0 * 60.0));

    return rpm;
}





void stop()
{
  currentSpeedLeft=0;
  currentSpeedRight=0;
  // resetting these variables seems to be a bad idea 
  //prev_error_right=0;
  //prev_error_left=0;
  //integralLeft=0;
  //integralRight=0;
  //encoderPositionLeft = 0;
  //encoderPositionRight = 0;

  setPWM (LeftMotor_Pin1, LeftMotor_Pin2, LeftMotor_PWM,0);
  setPWM (RightMotor_Pin1, RightMotor_Pin2, RightMotor_PWM,0);
}




void loop() {
  // Check if there is data available on the serial port
  if (Serial.available() > 0) 
  {
    
    String input = Serial.readStringUntil('\n');  // Read the input from the serial port until a newline character is received
    int commaIndex = input.indexOf(',');  // Find the index of the comma in the input string
    if (commaIndex > -1) {  // Check if a comma was found
      String firstIntString = input.substring(0, commaIndex);  // Extract the first integer as a string
      String secondIntString = input.substring(commaIndex + 1, input.length());  // Extract the second integer as a string
      requiredSpeedLeft = firstIntString.toFloat();  // Convert the first integer string to an integer
      requiredSpeedRight = secondIntString.toFloat();  // Convert the second integer string to an integer

      requiredSpeedLeft = constrain(requiredSpeedLeft, -maxRPM, maxRPM);
      requiredSpeedRight = constrain(requiredSpeedRight, -maxRPM, maxRPM);

      prev_error_right=0;
      prev_error_left=0;
      integralLeft=0;
      integralRight=0;

    }
  }

  unsigned long currentTime = micros();
  unsigned long interval = currentTime - previousMillis;
  if (interval >= loopInterval) 
  {
    if (requiredSpeedLeft != 0 || requiredSpeedRight !=0)
    {
      pidLoop(interval);
      Serial.print("currentSpeedLeft:");
      Serial.print(currentSpeedLeft);
      Serial.print(",currentSpeedRight:");
      Serial.println(currentSpeedRight);
      
    }
    else
    {
      stop();
    }
    previousMillis = currentTime; 
  }
}
