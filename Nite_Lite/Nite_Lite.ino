// Created by Zen Maker Lab Inc.
// Website: zenmakerlab.com

#include "FastLED.h"
#include <inttypes.h>

#define NUM_LEDS 6
#define DATA_PIN 8
#define CLOCK_PIN 9

#define SIGNAL_A 2
#define SIGNAL_B 3
#define PUSH_BUTTON 4

#define NUM_STATES 3

#define RED_INTENSITY 0
#define GREEN_INTENSITY 1
#define BLUE_INTENSITY 2

CRGB leds[NUM_LEDS];

//CLK FREQ = 16MHz
volatile bool A_set = false;
volatile bool B_set = false;
volatile int encoderPos = 0;
int encoderPosition = 0;
int buttonRead = 0;
int buttonState = 0;

int state = 0;

uint8_t red = 0; 
uint8_t green = 0; 
uint8_t blue = 0; 

// color show parameters
int dir = 1; 
int stepTime = 50; 
int showLED = 0; 
int start = 0; 
int finish = 0; 

void setup() {
  noInterrupts();
  pinMode(SIGNAL_A, INPUT_PULLUP);
  pinMode(SIGNAL_B, INPUT_PULLUP);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), turnA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), turnB, CHANGE);
  Serial.begin(9600);
  
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical
  for (int i=0;i<NUM_LEDS-1;i++){
    leds[i] = CRGB (0,0,0);
    }
  
  FastLED.show();

  interrupts();
}

void loop() {

  //Statemachine.
  switch(state){
      case RED_INTENSITY:
          encoderPosition = encoderPos; 
          red = (int8_t) map(encoderPosition, 0, 24, 0, 255);
          break;
      case GREEN_INTENSITY:
          encoderPosition = encoderPos;
          green = (int8_t) map(encoderPosition, 0, 24, 0, 255);
          break;
      case BLUE_INTENSITY:
          encoderPosition = encoderPos;
          blue = (int8_t) map(encoderPosition, 0, 24, 0, 255);
          break;
      default:
        break;
    }

  colorStrip(red,green,blue,NUM_LEDS);
  // If button pressed, update state
  if (updateState()){state++;}
  if (state >= NUM_STATES){state = 0;}
  Serial.print(red);
  Serial.print(',');
  Serial.print(green);
  Serial.print(',');
  Serial.println(blue);
}

bool updateState(){
  buttonRead = 0x0;
  buttonRead |= !digitalRead(PUSH_BUTTON) << 0;

  buttonState |= buttonRead;
  
  if (!buttonRead){
    if(buttonState & 0x1){
      buttonState &= ~0x1;
      return true;
    }
  }
  return false;
}

void colorStrip(int8_t r, int8_t g, int8_t b, int pos){
  for (int i=0;i<pos;i++){
    leds[i] = CRGB (r,g,b);
  }
  FastLED.show();
}

void turnA(){
  if( digitalRead(SIGNAL_A) != A_set ) {  // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set ) {
      encoderPos += 1;
      if (encoderPos > 24) {
          encoderPos = 24; 
      }
    }
  }
}

void turnB(){
   if( digitalRead(SIGNAL_B) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) {
      encoderPos -= 1;
      if (encoderPos < 0) { 
          encoderPos = 0; 
      }
    }
  }
}
