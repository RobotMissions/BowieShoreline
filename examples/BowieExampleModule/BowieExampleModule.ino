/*
 * Custom Bowie Module: Simple LEDs
 * ----------------------------------
 * 
 * ** REMEMBER TO HAVE A LOGIC LEVEL CONVERTOR BETWEEN
 *    YOUR ARDUINO AND BOWIE'S BRAIN BOARD! **
 *    
 * This is a simple example of a module added on
 * to Bowie. It can respond to the Operator Interface,
 * by having the Operator Interface messages sent to 
 * the Bowie Brain, then forwarded to the our connected
 * Arduino.
 * 
 * Erin RobotGrrl for RobotMissions
 * March 20th, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// neopixel
#define PIN            6
#define NUMPIXELS      16
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

long current_time = 0;
long last_blink = 0;
bool blink_on = true;

bool update_pixels = false;
int pixel_ind = 7;
int bright_val = 0;

char c = ' ';
uint32_t pixel_col = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pixels.begin();

  for(int i=0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i, pixels.Color(0,150,0));
    pixels.show();
    delay(50);
  }
  for(int i=NUMPIXELS-1; i>=0; i--) {
    pixels.setPixelColor(i, pixels.Color(0,0,0));
    pixels.show();
    delay(50);
  }
  
}

void loop() {
  current_time = millis();
  
  // sending API command to Bowie to blink its leds
  // this is an example of sending data from your module,
  // back to the brain of the robot
  if(current_time-last_blink >= 1000) {
    if(blink_on) {
      Serial.println("#Q0,200,Q1,200!");
      digitalWrite(13, HIGH);
    } else {
      Serial.println("#Q0,0,Q1,0!");
      digitalWrite(13, LOW);
    }
    blink_on = !blink_on;
    last_blink = current_time;
  }

  // depending on how fast your arduino is, this could
  // either be a while or an if. if your arduino is slow,
  // keep it as an if. in simple message forwarding mode
  // you'll only be receiving single chars anyway, so you
  // won't need the while to block the code while the buffer
  // fills.
  //
  // check out forwarding_simple_mapping.txt in the github
  // repository to see what command means what...
  if(Serial.available() > 0) {
    c = Serial.read();
    Serial.print(c);
  } else {
    c = ' ';
  }

  switch(c) {
    case 'P':
      Serial.println("YUP!");
      pixel_col = pixels.Color(200+bright_val,0,0);
    break;
    case 'Y':
      pixel_col = pixels.Color(180+bright_val,100+bright_val,0);
    break;
    case 'G':
      pixel_col = pixels.Color(150+bright_val,150+bright_val,0);
    break;
    case 'Q':
      pixel_col = pixels.Color(0,200+bright_val,0);
    break;
    case 'B':
      pixel_col = pixels.Color(0,10,200+bright_val);
    break;
    case 'N':
      pixel_col = pixels.Color(150+bright_val,0,150+bright_val);
    break;
    case 'W':
      // control pixel up
      pixel_ind++;
      if(pixel_ind > 16) pixel_ind = 15;
    break;
    case 'S':
      // control pixel down
      pixel_ind--;
      if(pixel_ind < 0) pixel_ind = 0;
    break;
    case 'A':
      // control brightness (less)
      bright_val -= 20;
      if(bright_val < -60) bright_val = -60;
    break;
    case 'D':
      // control brightness (more)
      bright_val += 20;
      if(bright_val > 60) bright_val = 60;
    break;
  }

  for(int i=0; i<NUMPIXELS; i++) {
    if(i == pixel_ind) {
      pixels.setPixelColor(pixel_ind, pixel_col);
    } else {
      pixels.setPixelColor(i, 0);
    }
  }
  pixels.show();

}


