#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//pins
int absolutePins[5] = {9,10,11,12,13};
int incrementalPin = 8;
int stepPin = 5; //pwm pin with 1khz default
int directionPin = 4;

//global variables
int dir = 1;
float outputDecimal = 0;
int outputGray = 0;
volatile int counter = 0;
long currentTime;
long previousTime;
long timeDif;

long counterFreq;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  int i = 0;
  for(i=0;i<6;i++) {
    pinMode(absolutePins[i], INPUT_PULLUP);
  }

  //attach interupt to incremental counter
  pinMode(incrementalPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(incrementalPin), count, RISING);

  //
  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  analogWrite(stepPin, 127);
  digitalWrite(directionPin, dir);
  
  
  currentTime = micros();
  //setup OLED
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  delay(1000);         // wait for initializing
  oled.clearDisplay(); // clear display
  oled.setTextColor(WHITE);     // text color
  oled.setTextSize(1);          // text size
  oled.setCursor(0, 10);        // position to display
  oled.println("Hello World!"); // text to display
  oled.display();               // show on OLED
  
  
}

void loop() {
  int i = 0;
  //String out = "";
  digitalWrite(directionPin, dir);
  
  //first 5 bits (A,B,C,D,E) are absolute encoder
  for(i=0;i<5;i++) {
    int val = digitalRead(absolutePins[i]);
    outputGray = ((outputGray<<1) | val); //combine digital readings into one gray code
  }
  outputDecimal = inversegrayCode(outputGray)* 11.25; //5bits = 32 segments. 360/32 = 11.25 degrees
  
  //incremental counter
  if (counter > 70) {
    dir = 0;
  }
  if (counter < 10) {
    dir = 1;
  }
  String angle = String(outputDecimal) + "°";
  
  oled.clearDisplay(); // clear display
  oled.setCursor(0, 10);        // position to display
  oled.println(angle); // text to display
  oled.setCursor(0, 20);        // position to display
  oled.println(counter); // text to display
  oled.display();               // show on OLED

  String timing = String(counter) + "," + timeDif;
  
  //if we know freqnecy of counters
  //One counter corresponds to 6 degrees
  //Angles per second = 6 * counters * counterfreq => 6*1*freq
  long omega = 6 * counterFreq; //   (angles per s)/360 = cycles per second
  long freq = omega/360;
  // 1hz = 60 rpm
  float rpm = freq * 60;
  Serial.println(counter);
  
}

int inversegrayCode(int n) {
    int inv = 0;
    // Taking xor until n becomes zero
    for (; n; n = n >> 1)
        inv ^= n;
 
    return inv;
}
void count() {
  previousTime = currentTime;
  currentTime = millis();
  timeDif = currentTime - previousTime;
  counterFreq = 1/timeDif;
  if (dir == 1) {
    counter++;
  } else {
    counter--;
  }
}
