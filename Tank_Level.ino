#include <TM1637Display.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <TM1637Display.h>

// USER SET PARAMETERS
float calibration = .8; // set this to calibrate: actual flow = measured flow * calibration
float tank_capacity = 36.0; // capacity of the water tank in gallons
int button_brightness = 64; // how bright the button LED is, 0-255
int displayBrightness = 7; // how bright the display is, 0-7

// timer variables:
long hack;
long hack_button;
long hack_display;
long hack_debug;
long hack_flow;
long hack_full;
long hack_stored; 

// liquid sensor
int liquid_pin = 5;  // green wire; red is +, blue is -
int liquidState = 0;
int liquid_led = 7;  // the pin of the indicator LED; turns on when it's full

// button variables
int buttonPin = 12;
int buttonValue;
int display_type = 1; // 0 = blank, 1 = gallons remaining, 2 = flow rate
int LED = 9; // for the light in the button
int brightness;
bool display_on;
bool button_pressed;

/// *** from adafruit's basic script for flowmeter, each pulse is ~ 2.25 milliliters *** ///
#define FLOWSENSORPIN   13     // paddlewheel pin
volatile uint16_t pulses = 0;
volatile uint8_t lastflowpinstate;
volatile uint32_t lastflowratetimer = 0;
volatile float flowrate;
float pulse_old = 0;
float pulse_new = 0;
float flow;
float gallons_remaining = tank_capacity;
int gallons_remaining_p = 100; 
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return;
  }
  if (x == HIGH) {
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;
  lastflowratetimer = 0;
}
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
  }
}
/// *** end adafruit's flow sensor script *** ///

// 7 segment display variables:
#define CLK 2
#define DIO 3
TM1637Display display(CLK, DIO);
uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
uint8_t d = SEG_B | SEG_C | SEG_D | SEG_E | SEG_G;
uint8_t F = SEG_A | SEG_E | SEG_F | SEG_G;
uint8_t G = SEG_A | SEG_C | SEG_D | SEG_E | SEG_F;
uint8_t P = SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
uint8_t r = SEG_E | SEG_G;
int one;
int two;
int three;
uint8_t four;

// eePROM variables:
int eeAddress = 0;
boolean stored = true;
boolean firstrun = true;

void setup() {

   // Set up Serial for debugging purposes
   Serial.begin(9600);

   // read from the eeProm
   EEPROM.get(eeAddress, gallons_remaining);
   gallons_remaining_p = ((gallons_remaining / tank_capacity) * 100) + .5;
   Serial.println("********** Initial read from eeProm ************** ");
   Serial.print("gallons_remaining: "); Serial.println(gallons_remaining);
   Serial.print("gallons_remaining_p: "); Serial.println(gallons_remaining_p);

   // for the display
   display.clear();
   display.setBrightness(displayBrightness);
   display_on = true;
   
   // for the flow sensor
   pinMode(FLOWSENSORPIN, INPUT);
   digitalWrite(FLOWSENSORPIN, HIGH);
   lastflowpinstate = digitalRead(FLOWSENSORPIN);
   useInterrupt(true);

   // for the button
   pinMode(buttonPin, INPUT_PULLUP);
   pinMode(LED, OUTPUT);

   // for the liquid level sensor
   pinMode(liquid_led, OUTPUT);
   pinMode(liquid_pin, INPUT);
}

void loop()
{ 

  // establish the current hack (in milliseconds from script initiation)
  hack = millis();

  // ************************ LIQUID LEVEL SENSOR ************************ //
  liquidState = digitalRead(liquid_pin);

  // if it's HIGH, it's dry
  if (liquidState == LOW) {
    digitalWrite(liquid_led, HIGH);
  }
  else {
    digitalWrite(liquid_led, LOW);
  }
  // ************************ END LIQUID LEVEL SENSOR ************************ //

  // ************************ FLOW SENSOR ************************ //
  pulse_new = pulses * calibration;

  // if the new pulses are different than the previous, and it's been more than a second since the last update
  if (pulse_new > pulse_old && hack - hack_flow > 1000) {
    hack_flow = hack;
    hack_button = hack;
    display_on = true;
    stored = false;

    // how much has flowed since the last run (1 pulse = 2.25 ml)
    flow = (pulse_new - pulse_old) * .000594 ; // flow = how many gallons have flowed since last run (in gal/1 sec) pulse
    gallons_remaining = gallons_remaining - flow;
    if (gallons_remaining < 0) {
      gallons_remaining = 0;
    }
    gallons_remaining_p = ((gallons_remaining / tank_capacity) * 100) + .5;
    pulse_old = pulse_new;
  }
  // ************************ END FLOW SENSOR ************************ //
  
  // ************************ BUTTON ************************ //
  buttonValue = digitalRead(buttonPin);

  // if it's low, button is pressed
  if (buttonValue == LOW){

    // if the button wasn't pressed before, but is now:
    if (button_pressed == false) {
      
      hack_button = hack;
      button_pressed = true;

      // if the display is already on, simply cycle the display type
      if (display_on == true) {
        display_type += 1;
        if (display_type == 4) {
          display_type = 0;
        }
        hack_display -= 1000; //this simply triggers an immediate update in the display portion below
      }
      // if the display is off and the button is pressed, display the last data and turn it on
      else {
        display_on = true;
        hack_display -= 1000; // this simply triggers an immediate update in the display portion below
      }
    }

    // if the button has been held for 2.5 seconds, set tank full and change display to show gallons remaining
    if (hack - hack_button > 2500) {
      gallons_remaining = tank_capacity;
      gallons_remaining_p = 100;
      display_type = 1;
      display_on = true;
      hack_display -= 1000;
      stored = false;
      Serial.println("*************** Tank full! ************************");
    }
   }
   // if the button is not pressed
   else {
    button_pressed = false;
   }
  // ************************ END BUTTON ************************ //

  // ************************ DISPLAY ************************ //
  // gallons remaining
  if (display_type == 1) {
    one = gallons_remaining/10;
    two = gallons_remaining - (one * 10); 
    three = (gallons_remaining * 10) - (one * 100) - (two * 10);
    four = SEG_A | SEG_C | SEG_D | SEG_E | SEG_F; // G
    data[0] = display.encodeDigit(one);
    data[1] = display.encodeDigit(two);
    data[2] = display.encodeDigit(three);
    data[3] = four;
    if (gallons_remaining < 10) {
      one = 0x00;
      two = gallons_remaining;
      three = (gallons_remaining * 10) - (two * 10);
      data[0] = one;
      data[1] = display.encodeDigit(two);
      data[2] = display.encodeDigit(three);
    }
    brightness = button_brightness;
  }

  // percent water remaining
  if (display_type == 2) {
    brightness = button_brightness;
    if (gallons_remaining_p == 100) {
      one = 1;
      two = 0;
      three = 0;
      four = P;
      data[0] = display.encodeDigit(one);
      data[1] = display.encodeDigit(two);
      data[2] = display.encodeDigit(three);
      data[3] = four;
    }
    else {
      one = 0x00;
      two = (gallons_remaining_p / 10);
      three = gallons_remaining_p - (two * 10);
      four = P;
      data[0] = one;
      data[1] = display.encodeDigit(two);
      data[2] = display.encodeDigit(three);
      data[3] = four;
    }
  }
  
  // current flow
  if (display_type == 3) {
    one = 0x00;
    two = flow * 60;
    three = ((flow * 60)  * 10) - (two * 10);
    four = SEG_A | SEG_E | SEG_F | SEG_G; // F
    data[0] = one;
    data[1] = display.encodeDigit(two);
    data[2] = display.encodeDigit(three);
    data[3] = four;
    brightness = button_brightness;
  }

  // update the display every second
  if (hack - hack_display >= 1000) {
    
    hack_display = hack;

    // if the display is supposed to be on:
    if (display_on == true) {
      // if display type is 0, turn it off; otherwise display the data
      if (display_type == 0) {
        display.clear();
        analogWrite(LED, 0);
      }
      else {
        display.setSegments(data);
        analogWrite(LED, button_brightness);
      }
    }
  }
  
  // if it's been more than 15 seconds from when the button was pushed, turn the display and arcade button off
  if (hack - hack_button > 15000) {
    display.clear();
    analogWrite(LED, 0);
    display_on = false;
  }
  // ************************ end display ************************ //

  // ************************ EEPROM ************************ //
  // if stored = false (from flow or button push) and it's been more than 15 seconds since last flow
  if (hack - hack_flow > 15000 && stored == false){ 
    EEPROM.put(eeAddress, gallons_remaining);
    stored = true;
    flow = 0;
    Serial.println("********** Saved to EEPROM! **********");
  }
  // ************************ END EEPROM ************************ //
  
}
