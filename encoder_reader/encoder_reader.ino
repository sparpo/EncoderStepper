
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define measureTime 1000.0
//#define testInterrupt
#define testTime


//pins
//#define

int testLED = 0;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int wave = 0;
//pins
int absolutePins[5] = {12,11,10,9,8};//first 5 bits (A,B,C,D,E) are absolute encoder
int incrementalPin = 2; //incremental encoder bit. Attach to pin 2 or 3, as they have hardware interupt

//with normal 1.8 deg steppers, we would choose a frequency around 0hz to 50hz
//The stepper drive in this stepup requires a much higher frequency because it is a microstepping drive.

int stepPin = 5; //pwm pin with 1khz default. Later set to 4kHz
int directionPin = 4; //motor direction

//global variables
int dir = 1;
int dirMeasured = 1;

//graycode output
int outputGray = 0;
String binaryString = "";
float outputDecimal = 0;

//Counter variables
volatile int counter = 0;//all variables used in interrupt must be declared volatile
int prevCounter = 0;
int counterDif = 0;

//timing variables
float currentTime = 0;
float previousTime = 0;
float timeScaled  = 8;

float rpm = 0.0;

//position
int maxPos = 180;
int minPos = 10;

void setup() {
  Serial.begin(19200);
  // put your setup code here, to run once:
  int i = 0;
  for(i=0;i<5;i++) {
    pinMode(absolutePins[i], INPUT_PULLUP);
  }

  //attach interupt to incremental counter
  pinMode(incrementalPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(incrementalPin), count, FALLING);

  pinMode(13, OUTPUT);
  //set up output pins
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  //change PWM frequency
  setSpeed_nanoEvery(3); //for nano every
  //setSpeed() //for uno/nano
  
  analogWrite(stepPin, 127); // 50% duty. Even square wave
  digitalWrite(directionPin, dir);

 //steps per rev = 8000
 //frequency = 7.8k
  
  currentTime = millis() * timeScaled;

  setupOLED();
  
}

void loop() {
  digitalWrite(13, testLED);
  
  float lastDecimal = outputDecimal;//record last angle
  
  int i = 0;
  //String out = "";
  digitalWrite(directionPin, dir);
  outputGray = 0;
  binaryString = "";
  
  //first 5 bits (A,B,C,D,E) are absolute encoder
  for(i=0;i<5;i++) {
    int val = digitalRead(absolutePins[i]);
    outputGray = ((outputGray<<1) | val); //combine digital readings into one gray code
    binaryString = binaryString + String(val);
  }

  outputDecimal = inversegrayCode(outputGray)* 11.25; //5bits = 32 segments. 360/32 = 11.25 degrees
  if(lastDecimal > outputDecimal) { //ld = 0, od = 360
    dirMeasured = 1;
    if(lastDecimal == 348.75) {
      dirMeasured = -1;
    }
  } else if(lastDecimal < outputDecimal) {
    dirMeasured = -1;
    if(lastDecimal == 0){
      dirMeasured = 1;
    }
  }
  
  //incremental counter
  if (counter >= maxPos) {
    dir = 0;
  }
  if (counter <= minPos) {
    dir = 1;
  }
  if(counter > (maxPos*0.9) | counter < (minPos*1.1)){
    setSpeed_nanoEvery(1);
  }else if(counter > (maxPos*0.8) | counter < (minPos*1.2)){
    setSpeed_nanoEvery(3);
  } else {
    setSpeed_nanoEvery(5);
  }
  
  float rotations = 0;
  currentTime = millis();
  float timeLimit = float(measureTime) * float(timeScaled); //needs to be scaled up as faster clock frequency is used
  if((previousTime + timeLimit) <= currentTime) {
    
    counterDif = counter - prevCounter;
    previousTime = currentTime;
    prevCounter = counter;
  
    rotations = counterDif/60.0; //60 counts per rotation

    rpm = rotations / (float(measureTime)/(1000.0*60.0)); //rotations per minute 
    
   //testing time
  #ifdef testTime
    testLED = !testLED;
    
  #endif
  }
  //Serial.println(counterDif);
  //Serial.println(rotations);
  
  
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
  counter +=dirMeasured;

  //testing interupt
  #ifdef testInterrupt
    testLED = !testLED;
    
  #endif
}

void updateOLED(float angle, int counter, float rpm){
  oled.setTextSize(2);
  oled.clearDisplay(); // clear display
  oled.setCursor(0, 0);
  oled.println(String(rpm) + " rpm"); // RPM
  oled.setCursor(0, 35);        // position to display
  
  oled.println(String(angle) + " deg"); // Angle
  
  oled.setTextSize(1);
  oled.setCursor(0, 54);        // position to display
  oled.println("ABCDE: " + binaryString); // text to display
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
/*
void setSpeed() {
  //For Atmega328 Nano/Uno
  TCCR0B = 0b00000010; // 10 = x8, 101 = x1024
  TCCR0A = 0b00000011; // 01 = phase correct(4khz), 11 = fast pwm(7.8khz)  
}*/
void setSpeed_nanoEvery(int spd) { 
  //For Atmega4809 Nano Every
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
                        // Turn off timer while we change parameters.
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_CLKSEL_gm;
                          // Clear all CLKSEL bits.
  switch(spd) {
    case 8:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1_gc;
      timeScaled = 64;
    break;
    case 7:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV2_gc;
      timeScaled = 32;
    break;
    case 6:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV4_gc;
      timeScaled = 16;
    break;
    case 5:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV8_gc;
      timeScaled = 8;
    break;
    case 4:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV16_gc;
      timeScaled = 4;
    break;
    default:
    case 3:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV64_gc;
      timeScaled = 1;
    break;
      
    case 2:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV256_gc;
      timeScaled = 0.25;
    break;

    case 1:
      TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1024_gc; //976hz / 16 
      timeScaled = 0.0625;
    break;
  }
  
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
                          // Re-enable timer. Pins 5 and 9 now run
                        
  
}
