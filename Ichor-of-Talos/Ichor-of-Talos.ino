//Pin signature: Data, Clock01, Latch, Clock02, Decrease, Increase, ChangeMode, Save
const int DataPin = 2;
const int Clock01Pin = 4;
const int LatchPin = 3;
const int Clock02Pin = 5;
const int DecreasePin = 6;
const int IncreasePin = 7;
const int ChangeModePin = 8;

// InputState Data
int DecState = 0;
int IncState = 0;
int CMState = 0;
int SaveState = 0;
int oldDecState = 0;
int oldIncState = 0;
int oldCMState = 0;


// OutputState Data
int Output[] = {0,0,0,0};
int OutputProcessing01 = 0;
int OutputProcessing02 = 0;
int OutputProcessing03 = 0;
int OutputProcessing04 = 0;

// Limb States
// Measured in seconds
int Limb_Alpha = 0;
int Limb_Beta = 0;
int Limb_Gamma = 0;
int Interaction_State = 0;
int Motors[] = {Limb_Alpha, Limb_Beta, Limb_Gamma};


int range = 90;
int oldrange = range;
int timer = 0;

// servo definitions
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver();
#define SERVO_FREQ  50
#define C_SERVOMIN  275
#define C_SERVOMAX  442


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ServoDriver.begin();
  ServoDriver.setPWMFreq(60);
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
  

  if (DecState != oldDecState){
    if (DecState == HIGH & range > 0 ){
      Serial.println("Pressed Decrease");
      range = range - 1;
      
      delay(50);
    } else {
      Serial.println("Return of Key: Decrease");
    }
    
  }
  oldDecState = DecState;

  if (IncState != oldIncState){
    if (IncState == HIGH & range < 180 ){
      Serial.println("Pressed Increase");
      range = range + 1;
      delay(50);
    } else {
      Serial.println("Return of Key: Increase");
    }
    
  }
  oldIncState = IncState;

  if (CMState != oldCMState){
    if (CMState == HIGH){
      Serial.println("Action");
      timer = oldrange - range;
      if (timer > 0){
        timer = timer * 100;
        ServoDriver.setPWM(0,0,C_SERVOMAX);
        ServoDriver.writeMicroseconds(0, timer);
        delay(timer);
        ServoDriver.setPWM(0,0,359);
      } else if (timer < 0) {
        timer = timer * -1 * 100;
        ServoDriver.setPWM(0,0,C_SERVOMIN);
        ServoDriver.writeMicroseconds(0, timer);
        delay(timer);
        ServoDriver.setPWM(0,0,359);
      } 
      timer = 0;
      oldrange = range;
      delay (50);
    }
  }
  oldCMState = CMState;

  Serial.print("Angle: ");
  Serial.print(range);
  Serial.print("\n");

  for (int n = 0; n <4; n++){
    OutputProcessing01 = (range % (int)(pow(10,n+1)));
    OutputProcessing02 = (range % (int)(pow(10,n)));
    OutputProcessing03 = (int)((OutputProcessing01 - OutputProcessing02)/(int)(pow(10,n)));
    Output[n] = OutputProcessing03;
    // B = angle >> n + 1;
    // C = B << n + 1;
    // D = angle - C;
    // D = D >> n;
    //Output[n] = D;
  }
  
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
    // alphabets S and V
    case 17 : SegmentData = 0b01110001; break;
    case 18 : SegmentData = 0b01110001; break;
    

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