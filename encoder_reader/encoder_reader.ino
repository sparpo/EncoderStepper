#include <SevSeg.h>
SevSeg sevseg; //Initiate a seven segment controller object



int inputPins[6] = {A0,A1,A2,A3,A4};
int incrementalPin = A5;
volatile int counter = 0;

int stepPin = 5;
int directionPin = 4;

int dir = 1;

int inversegrayCode(int n) {
    int inv = 0;
    // Taking xor until n becomes zero
    for (; n; n = n >> 1)
        inv ^= n;
 
    return inv;
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  int i = 0;
  for(i=0;i<6;i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }
  pinMode(incrementalPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(incrementalPin), count, RISING);

  //segment display!
    byte numDigits = 4;  
    byte digitPins[] = {2, 3, 4, 5};
    byte segmentPins[] = {6, 7, 8, 9, 10, 11, 12, 13};
    bool resistorsOnSegments = 0; 
    // variable above indicates that 4 resistors were placed on the digit pins.
    // set variable to 1 if you want to use 8 resistors on the segment pins.
    sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins, resistorsOnSegments);
    sevseg.setBrightness(90);
  
}

void loop() {
  int i = 0;
  //String out = "";
  int outputGray = 0;

  //first 5 bits (A,B,C,D,E) are absolute encoder
  for(i=0;i<5;i++) {
    int val = digitalRead(inputPins[i]);
    outputGray = ((outputGray<<1) | val); //combine digital readings into one gray code
  }
  
  
  if (counter > 70) {
    dir = 0;
  }
  if (counter < 10) {
    dir = 1;
  }
    

  float outputDecimal = inversegrayCode(outputGray)* 11.25; //5bits = 32 segments. 360/32 = 11.25 degrees

  //Serial.println(out);
  sevseg.setNumber(outputDecimal*10, 1);
  sevseg.refreshDisplay(); // Must run repeatedly
}

void count() {
  if (dir == 1) {
    counter++;
  } else {
    counter--;
  }
}
