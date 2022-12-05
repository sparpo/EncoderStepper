#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define avgSize 10
#define measureTime 5000000
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
float outputDecimal = 0;

//Counter variables
volatile int counter = 0;//all variables used in interrupt must be declared volatile
int prevCounter = 0;
int counterDif = 0;

//timing variables
double currentTime = 0;
double previousTime = 0;

float rpm = 0.0;

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
  TCCR0B = 0b00000010; // 10 = x8, 101 = x1024
  TCCR0A = 0b00000011; // 01 = phase correct(4khz), 11 = fast pwm(7.8khz)
  analogWrite(stepPin, 127); // 50% duty. Even square wave
  digitalWrite(directionPin, dir);

 //steps per rev = 8000
 //frequency = 7.8k
  
  currentTime = micros();

  setupOLED();
  
}

void loop() {
  float lastDecimal = outputDecimal;//record last angle
  
  digitalWrite(13, testLED);
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
  outputDecimal = inversegrayCode(outputGray)* 11.25; //5bits = 32 segments. 360/32 = 11.25 degrees
  
  if(lastDecimal > outputDecimal) {
    dirMeasured = -1;
  } else if(lastDecimal < outputDecimal) {
    dirMeasured = 1;
  }
  
  //incremental counter
  if (counter >= 160) {
    dir = 0;
  }
  if (counter <= 10) {
    dir = 1;
  }
  
  float rotations = 0;
  currentTime = micros();
  if((previousTime + measureTime) <= currentTime) {
    
    counterDif = counter - prevCounter;
    previousTime = currentTime;
    prevCounter = counter;
  
    rotations = counterDif/60.0; //60 counts per rotation

    rpm = rotations / (float(measureTime)/(10000000.0*60.0)); //rotations per minute 
    
   //testing time
  #ifdef testTime
    testLED = !testLED;
    
  #endif
  }
  Serial.println(counterDif);
  Serial.println(rotations);
  
  
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
  oled.println(String(angle) + " deg"); // text to display
  oled.setCursor(5, 30);        // position to display
  oled.println(String(rpm) + " rpm"); // text to display
  
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
