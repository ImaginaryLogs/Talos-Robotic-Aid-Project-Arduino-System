//Pin signature: Data, Clock01, Latch, Clock02, Decrease, Increase, ChangeMode, Save
const int DataPin = 2;
const int Clock01Pin = 4;
const int LatchPin = 3;
const int Clock02Pin = 5;
const int DecreasePin = 6;
const int IncreasePin = 7;
const int ChangeModePin = 8;
const int SaveModePin = 9;
const int LoadModePin = A3;

// InputState Data
int DecState = 0;
int IncState = 0;
int CMState = 0;
int SaveState = 0;
int LoadState = 0;
int oldDecState = 0;
int oldIncState = 0;
int oldCMState = 0;
int oldSaveState = 0;
int oldLoadState = 0;
int distance = 0;

// OutputState Data
int Output[] = {0,0,0,0};
int OutputProcessing01 = 0;
int OutputProcessing02 = 0;
int OutputProcessing03 = 0;
int OutputProcessing04 = 0;

// Limb States
// Measured in seconds
int Interaction_State = 0;
int Motors[4][2] = {{0,0}, {0,0}, {0,0}, {0,0}};


int range = 90;
int oldrange = range;
int timer = 0;

// Motor definitions
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Stepper.h>

const int SERVO_FREQ = 50;
const int C_SERVOMIN = 275;
const int C_SERVOMAX = 442;
const int C_SERVOSTOP = 359;

const int stepsPerRevolution = 200;

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver();
Stepper myStepper(stepsPerRevolution, 10, 11, 12, 13);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ServoDriver.begin();
  ServoDriver.setPWMFreq(60);
  myStepper.setSpeed(60);
  for (int n = 2; n < 6; n += 1){
    pinMode(n, OUTPUT);
  }
  for (int n = 6; n < 10; n += 1){
    pinMode(n, INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  DecState = digitalRead(DecreasePin);
  IncState = digitalRead(IncreasePin);
  CMState = digitalRead(ChangeModePin);
  SaveState = digitalRead(SaveModePin);
  LoadState = analogRead(LoadModePin);
  

  if (DecState != oldDecState){
    if (DecState == HIGH & range > 0 ){
      Serial.println("Pressed Decrease");
      Motors[Interaction_State][0] = Motors[Interaction_State][0] - 1;
      if (Interaction_State != 2){
        ServoDriver.wakeup();
        ServoDriver.setPWM(Interaction_State, 0, C_SERVOMAX);
        delay(2000);
    
        ServoDriver.setPWM(Interaction_State, 0, C_SERVOSTOP);
        ServoDriver.sleep();
      } else {
        myStepper.step(-stepsPerRevolution*2);
      }

      delay(50);
    } else {
      Serial.println("Return of Key: Decrease");
    }
    
  }
  oldDecState = DecState;

  if (IncState != oldIncState){
    if (IncState == HIGH & range < 180 ){
      Serial.println("Pressed Increase");
      Motors[Interaction_State][0] = Motors[Interaction_State][0] + 1;
      if (Interaction_State != 2){
        ServoDriver.wakeup();
        ServoDriver.setPWM(Interaction_State, 0, C_SERVOMIN);
        delay(2000);
        ServoDriver.setPWM(Interaction_State, 0, C_SERVOSTOP);
        ServoDriver.sleep();
      } else {
        myStepper.step(stepsPerRevolution*2);
      }
      delay(50);
    } else {
      Serial.println("Return of Key: Increase");
    }
    
  }
  oldIncState = IncState;

  if (CMState != oldCMState){
    if (CMState == HIGH){
      Serial.println("Pressed Change Mode");
      Interaction_State = (Interaction_State + 1) % 3; 

      //Print: LNB Number
      for (int n = 0; n < 100; n++){
      screenWrite(0, 19);
      screenWrite(1, 20);
      screenWrite(2, 11);
      screenWrite(3, Interaction_State);
      }

      delay (50);
    } else {
      Serial.println("Return of Key: Change Mode");
    }
  }
  oldCMState = CMState;

  if (SaveState != oldSaveState){
    if (CMState == HIGH){
      Serial.println("Pressed Save Mode");
      Motors[Interaction_State][1] = Motors[Interaction_State][0];

      // Save Limb X
      for (int n = 0; n < 100; n++){
        screenWrite(0, 17);
        screenWrite(1, 10);
        screenWrite(2, 18);
        screenWrite(3, 14);
      }
      for (int n = 0; n < 100; n++){
        screenWrite(0, 19);
        screenWrite(1, 20);
        screenWrite(2, 11);
        screenWrite(3, Interaction_State);
      }

      delay(50);
    } else {
      Serial.println("Return of Key: Save Mode");
    }
  }
  oldSaveState = SaveState;
  
  if (LoadState > 500){
    for (int n = 0; n < 4; n++){
      distance = Motors[n][1] - Motors[n][0];
      if (distance > 0){
        if (n != 2){
          ServoDriver.setPWM(n, 0, C_SERVOMAX);
          delay(2000*distance);
          ServoDriver.setPWM(n, 0, C_SERVOSTOP);
        } else {
          myStepper.step(stepsPerRevolution*2*distance);
        }
      } else if (distance < 0) {
        if (n != 2){
          ServoDriver.setPWM(n, 0, C_SERVOMIN);
          delay(2000*distance*-1);
          ServoDriver.setPWM(n, 0, C_SERVOSTOP);
        } else {
          myStepper.step(stepsPerRevolution*2*distance);
        }
      }
    }
    for (int n = 0; n < 4; n++){
      distance = Motors[n][1] - Motors[n][0];
      if (distance > 0){
        if (n != 2){
          ServoDriver.setPWM(n, 0, C_SERVOMIN);
          delay(2000*distance*-1);
          ServoDriver.setPWM(n, 0, C_SERVOSTOP);
        } else {
          myStepper.step(stepsPerRevolution*2*distance);
        }
      } else if (distance < 0) {
        if (n != 2){
          ServoDriver.setPWM(Interaction_State, 0, C_SERVOMAX);
          delay(2000*distance);
          ServoDriver.setPWM(Interaction_State, 0, C_SERVOSTOP);
        } else {
          myStepper.step(stepsPerRevolution*2*distance);
        }
      }
    }
  }

  int_to_individualDigits(Motors[Interaction_State][0]);
  
  for (int n = 0; n < 25; n++){
    screenWrite(0, Output[3]);
    screenWrite(1, Output[2]);
    screenWrite(2, Output[1]);
    screenWrite(3, Output[0]);
  }

  delay(50);
}

void shiftWrite(int ChipNumber, int Data){
  digitalWrite(LatchPin, LOW);
  shiftOut(DataPin, ChipNumber, MSBFIRST, Data);
  digitalWrite(LatchPin, HIGH);
} 

void clearDigit(int LatchNumber){                                               
  for(int n = 0; n < 8; n++){
    shiftWrite(LatchNumber, 0b00000000);
  }
}

void int_to_individualDigits(int inputInteger){
  int negativeFlag = 0;
  if (inputInteger < 0){
      inputInteger *= -1;
      negativeFlag = 1;
  }
  for (int n = 0; n <4; n++){
    OutputProcessing01 = (inputInteger % (int)(pow(10,n+1)));
    OutputProcessing02 = (inputInteger % (int)(pow(10,n)));
    OutputProcessing03 = (int)((OutputProcessing01 - OutputProcessing02)/(int)(pow(10,n)));


    Output[n] = OutputProcessing03;
    
    
    // B = angle >> n + 1;
    // C = B << n + 1;
    // D = angle - C;
    // D = D >> n;
    //Output[n] = D;
  }
  if (negativeFlag == 1){
    Output[3] = 16;
  } else{
    Output[3] = 0;
  }
}

int decimal_to_7segment(int x){
  int SegmentData = 0;
  switch(x){
    // numbers 0 - 9
    case 0  : SegmentData = 0b00111111; break;
    case 1  : SegmentData = 0b00000110; break;
    case 2  : SegmentData = 0b01011011; break;
    case 3  : SegmentData = 0b01001111; break;
    case 4  : SegmentData = 0b01100110; break;
    case 5  : SegmentData = 0b01101101; break;
    case 6  : SegmentData = 0b01111101; break;
    case 7  : SegmentData = 0b00000111; break;
    case 8  : SegmentData = 0b01111111; break;
    case 9  : SegmentData = 0b01100111; break;
    // alphabets: A - E
    case 10 : SegmentData = 0b01110111; break; 
    case 11 : SegmentData = 0b01111100; break;
    case 12 : SegmentData = 0b00111001; break;
    case 13 : SegmentData = 0b01011110; break;
    case 14 : SegmentData = 0b01111001; break;
    case 15 : SegmentData = 0b01110001; break;
    // negative sign
    case 16 : SegmentData = 0b01000000; break;
    // alphabets S, V, L, N
    case 17 : SegmentData = 0b01101101; break;
    case 18 : SegmentData = 0b00111110; break;
    case 19 : SegmentData = 0b00111000; break;
    case 20 : SegmentData = 0b00110111; break;
    

    default : SegmentData = 0b01001001; break;
  }
  return SegmentData;
}

void screenWrite(int Digit, int NUM){
  int digitAddress = 0;
  switch(Digit){
    case 0 : digitAddress = 0b00001110; break;
    case 1 : digitAddress = 0b00001011; break;
    case 2 : digitAddress = 0b00001101; break;
    case 3 : digitAddress = 0b00000111; break;
    case 4 : digitAddress = 0b01100110; break;
    case 5 : digitAddress = 0b01101101; break;
    case 6 : digitAddress = 0b01111101; break;
    case 7 : digitAddress = 0b00000110; break;
    case 8 : digitAddress = 0b01111111; break;
    case 9 : digitAddress = 0b01100111; break;
  }
  shiftWrite(Clock02Pin, digitAddress);
  shiftWrite(Clock01Pin, decimal_to_7segment(NUM));
  shiftWrite(Clock01Pin, 0b00000000);
  delay(1);
}