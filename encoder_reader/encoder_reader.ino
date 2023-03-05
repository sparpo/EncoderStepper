#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define measureTime 100.0
//#define testInterrupt
#define testTime
#define debug

int testLED = 0;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//pins
int absolutePins[5] = {12,11,10,9,8};//first 5 bits (A,B,C,D,E) are absolute encoder
int incrementalPin = 2; //incremental encoder bit. Attach to pin 2 or 3, as they have hardware interupt (Nano only, Every should have an interrupt on every pin)

//motor pins
int outputPin = 5; //pwm pin with 1khz default.
int directionPin = 4; //motor direction

//motor control variables
int dir = 0;
int output = 0;

//absolute position
int absolutePos = 0;
int setPos = 0;

//control variables
int k = 0;

//Counter variables
volatile int counter = 0;//all variables used in interrupt must be declared volatile
int prevCounter = 0;
int counterDif = 0;

//timing variables
float currentTime = 0;
float previousTime = 0;
float timeScaled  = 8;

float rpm = 0.0;

void setup() {
  Serial.begin(19200);
  // put your setup code here, to run once:

  setupEncoder();
  pinMode(13, OUTPUT);
  //set up output pins
  pinMode(outputPin, OUTPUT);
  pinMode(directionPin, OUTPUT);


  
  analogWrite(outputPin, 127); // 50% duty. Even square wave
  digitalWrite(directionPin, dir);

  
  currentTime = millis();

  setupOLED();
  
}

void loop() {
  digitalWrite(13, testLED);
  float lastDecimal = absolutePos;//record last angle  
  absolutePos = absolutePosition();

  int error = calculateError(setPos, absolutePos, 32);
  
  if(error > 0){
    dir = 1;
  } else {
    dir = 0;
  }
  output = abs(error)*k;
  if(output > 255) {
    output = 255;
  }
  if(output < 0) {
    output = 0;
  }
  
  analogWrite(outputPin, output);
  digitalWrite(directionPin, dir);
  
  
  
  float angle = float(absolutePos)*11.25;
  rpm = calculateRPM();
  updateOLED(absolutePos, counter, rpm);
}



int calculateError(int a1, int a2, int maxVal){ //angular error 
    int e = a1 - a2;
    
    if(abs(e)>16){ //if mag is more than 16, we know it should be the other side
        if(e>0) {
            e = -(32-abs(e)); //change it to reflex angle with direction
        } else{
            e = 32-abs(e);
        }
    }
    return e;
}
int inversegrayCode(int n) {
    int inv = 0;
    // Taking xor until n becomes zero
    for (; n; n = n >> 1)
        inv ^= n;
 
    return inv;
}
int normalizePos(int a, int maxVal) {
  a = a% maxVal;
  if (a < 0) 
      a += maxVal;
  return a;
}

void count() {
  counter +=1;

  //testing interupt
  #ifdef testInterrupt
    testLED = !testLED;
    
  #endif
}

void setupEncoder() {
  int i = 0;
  for(i=0;i<5;i++) {
    pinMode(absolutePins[i], INPUT_PULLUP);
  }

  //attach interupt to incremental counter
  pinMode(incrementalPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(incrementalPin), count, FALLING);
}

int absolutePosition() {
  //first 5 bits (A,B,C,D,E) are absolute encoder
  int outputGray = 0;
  int i = 0;
  for(i=0;i<5;i++) {
    int val = digitalRead(absolutePins[i]);
    outputGray = ((outputGray<<1) | val); //combine digital readings into one gray code
  }
  return inversegrayCode(outputGray); //5bits = 32 segments. 360/32 = 11.25 degrees
}

int calculateRPM() {
  float rotations = 0;
  currentTime = millis();
  if((previousTime + measureTime) <= currentTime) {
    
    counterDif = counter - prevCounter;
    previousTime = currentTime;
    prevCounter = counter;
  
    rotations = counterDif/60.0; //60 counts per rotation
   //debug
  #ifdef testTime
    testLED = !testLED;
  #endif
  
  return rotations / (float(measureTime)/(1000.0*60.0)); //rotations per minute 
  }
  return rpm;
}

String binaryString(int decimal, int bits) {
  String val = "";
    for(int i = 0; i < bits; i++) {
    val+=bitRead(decimal, i);
  }
  return val;
}

void updateOLED(float angle, int counter, float rpm){
  oled.setTextSize(2);
  oled.clearDisplay(); // clear display
  oled.setCursor(0, 0);
  oled.println(String(rpm) + " rpm"); // RPM
  oled.setCursor(0, 35);        // position to display
  
  oled.println(String(float(angle)*11.25) + " deg"); // Angle
  
  oled.setTextSize(1);
  oled.setCursor(0, 54);        // position to display

  
  oled.println("ABCDE: " + binaryString(angle, 5)); // text to display
  oled.setCursor(0, 16);        // position to display
  oled.println("F: "+ String(counter) + " Counts"); // text to display

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
