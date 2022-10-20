#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define avgSize 10

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int wave = 0;
//pins
int absolutePins[5] = {9,10,11,12,13};
int incrementalPin = 8;
int stepPin = 5; //pwm pin with 1khz default
int directionPin = 4;

//global variables
int dir = 1;

int outputGray = 0;
long counterFreq = 0;

//rolling average 
float rollingRPM[avgSize] = {0};
float avgRPM;

//all variables used in interrupt must be declared volatile
volatile int counter = 0;
volatile long currentTime = 0;
volatile long previousTime = 0;
volatile long timeDif = 0;

int prevCounter = 0;

void setup() {
  Serial.begin(19200);
  // put your setup code here, to run once:
  int i = 0;
  for(i=0;i<5;i++) {
    pinMode(absolutePins[i], INPUT_PULLUP);
  }

  //attach interupt to incremental counter
  pinMode(incrementalPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(incrementalPin), count, RISING);

  //set up output pins
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
   TCCR0B = 0b00000010; // x8
  TCCR0A = 0b00000001; // phase correct
  analogWrite(stepPin, 127);
  digitalWrite(directionPin, dir);

  
  currentTime = millis();

  setupOLED();
  
}

void loop() {
  //Serial.println(counter, avgRPM);
  int i = 0;
  //String out = "";
  digitalWrite(directionPin, dir);
  outputGray = 0;
  //first 5 bits (A,B,C,D,E) are absolute encoder
  for(i=0;i<5;i++) {
    int val = digitalRead(absolutePins[i]);
    outputGray = ((outputGray<<1) | val); //combine digital readings into one gray code
  }
  float outputDecimal = inversegrayCode(outputGray)* 11.25; //5bits = 32 segments. 360/32 = 11.25 degrees
  
  //incremental counter
  if (counter >= 60) {
    dir = 0;
  }
  if (counter <= 10) {
    dir = 1;
  }
  
  if(digitalRead(incrementalPin) == 1 && wave == 0) {
    wave = 1;
    if (dir == 1) {
      counter++;
    } else {
      counter--;
    }
    
    /*previousTime = currentTime;
    
    timeDif = currentTime - previousTime;
    currentTime = millis();*/
  }
  if(digitalRead(incrementalPin) == 0 && wave == 1) {
    
   wave = 0;
  }


  String timing = String(counter) + "," + timeDif;
  
  //if we know freqnecy of counters
  counterFreq = 1.0/timeDif;
  //One counter corresponds to 6 degrees
  //Angles per second = 6 * counters * counterfreq => 6*1*freq
  long omega = 6.0 * counterFreq; 
  //(angles per s)/360 = cycles per second, frequency
  long freq = omega/360.0;
  // 1hz = 60 rpm
  float rpm = freq * 60.0;
/*
  //fill out rolling average array
  for(i=1; i < avgSize; i++) {
    rollingRPM[i] = rollingRPM[i - 1]; //move array items back one
    avgRPM += rollingRPM[i];
  }
  rollingRPM[0] = rpm; //insert newest value into array

  //find average
  for(i=0; i < avgSize; i++) {
    avgRPM += rollingRPM[i]; //get total
  }
  avgRPM = avgRPM/avgSize; //divide by array size
  */
  //Serial.println(counter);
  //Serial.println(rpm);
  Serial.println(counter);
  Serial.println(rpm);
  updateOLED(outputDecimal, counter, rpm);
}

int inversegrayCode(int n) {
    int inv = 0;
    // Taking xor until n becomes zero
    for (; n; n = n >> 1)
        inv ^= n;
 
    return inv;
}
void count() {
  Serial.println("hello");
  //counter++;
  //previousTime = currentTime;
  //currentTime = millis();
  //timeDif = currentTime - previousTime;
  
  //if (dir == 1) {
    
  //} else {
  //  counter--;
  //}
}

void updateOLED(float angle, int counter, float rpm){
  oled.setTextSize(2);
  oled.clearDisplay(); // clear display
  oled.setCursor(0, 0);
  oled.println(String(angle) + " deg"); // text to display
  oled.setCursor(20, 30);        // position to display
  oled.println(String(0) + " rpm"); // text to display
  
  oled.setTextSize(1);
  oled.setCursor(0, 50);        // position to display
  oled.println(String(counter) + " counts"); // text to display

  oled.display();               // show on OLED
}
void setupOLED() {
  //setup OLED
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  delay(1000);         // wait for initializing
  oled.clearDisplay(); // clear display
  oled.setTextColor(WHITE);     // text color
  oled.setTextSize(2);          // text size
  oled.setCursor(0, 10);        // position to display
  //oled.println("Hello World!"); // text to display
  oled.display();               // show on OLED
}
