#include <Arduino.h>
#include <limits.h>
#include <Wire.h>

//set enums, arduino pro mini:

int digitalx_vmax = 255;  //max voltaje
int digitalx_pwm_width = 2000;  //pwm vawe length
//number of digital pins, configure mapings for your arduino
//arduino pro mini is currently set
int digitalx_pins;


int * digitalx_t_left; //time left to update pin state
//positive (high) time delay
//
int * digitalx_td;
bool * digitalx_high; //current state of pin;

int * digitalx_mapping; //from internal pin index into arduino pins
int * digitalx_mapping_in;  //from arduino pin into internal index
bool * digitalx_mode; //if pin is to be handled
int digitalx_pinsmax; //max pins index to check from outside


/*
setups pins variables,
general, pwm settings
mappings (in, out)
data arrays
*/
void digital_analog_setup(void) {
  //start mappings
  digitalx_pins = 7;
  digitalx_mapping = new int [7];

  /*
  generate mappings
    mapping_in converts from real pin onto internal index
    holds -1 if it doesnt exists

    mapping holds from internal index into real index
  */
  //for arduino pro mini
  digitalx_mapping[2] = 4;
  digitalx_mapping[3] = 7;
  digitalx_mapping[4] = 8;
  digitalx_mapping[5] = 12;
  digitalx_mapping[6] = 13;

  //digitalx_mapping_in = int[ digitalx_mapping[digitalx_pins] ];
  digitalx_mapping_in = new int[13];
  for(int i = 0; i < digitalx_mapping[digitalx_pins]; i++) {
    digitalx_mapping_in[i] = -1;

    for(int j=0; j < digitalx_pins; j++)
      if(digitalx_mapping[j] == i)
        digitalx_mapping_in[i] = j;
  }

  digitalx_pinsmax = digitalx_mapping[digitalx_pins];

  digitalx_t_left = new int[7];
  digitalx_td = new int[7];
  digitalx_high = new bool[7];

  digitalx_mode = new bool[7];
}


/*
executed by "thread" emulator, a small delay betwen executions gives
better results
*/
void analogXUpdate(int dt) {
  int t_left;
  int td;
  int pin_to;
  int high;

  for(int pin = 0; pin < digitalx_pins; pin++)
    //for each activated pin
    if(digitalx_mode[pin]) {
      pin_to = digitalx_mapping[pin];
      td = digitalx_td[pin];

      //voltaje is on lower threshold (the time it holds on is 0), shutdown
      if(td == 0) {
        if(digitalx_high[pin]) {
          digitalWrite(  pin_to, LOW);
          digitalx_high[pin] = false;
        }
      }
      //voltaje is maximum
      else if(td == digitalx_pwm_width) {
        if(!digitalx_high[pin]) {
          digitalWrite( pin_to, HIGH);
          digitalx_high[pin] = true;
        }
      }
      //update
      else {
        t_left = digitalx_t_left[pin] -= dt;
        if(t_left < 0) {
          high = digitalx_high[pin];

          //set new left time, if high, then time portion left from
          //pmw width - pin time delay, and just use pin time delay otherwise
          //subtracting t_left module in both cases
          digitalx_t_left[pin] = (high)?
            (digitalx_pwm_width - td)  +  t_left :
            td + t_left;

          digitalWrite(pin, (high)? LOW:HIGH);
          digitalx_high[pin] = !high;
        }
        else
          digitalx_t_left[pin] =  t_left;
      }

    }
}
/*
enable, disable pin
*/
void pinXMode(uint8_t pin, bool mode) {

  if(pin < 0 || pin > digitalx_pinsmax)
    return;

  digitalx_mode[ digitalx_mapping_in[pin] ] = mode;
}
/*
from a voltaje, with max_voltaje specified in setup
*/
void analogXWrite(uint8_t pin, uint8_t v) {
  if(pin < 0 || pin > digitalx_pinsmax ||
    v < 0 || v > digitalx_vmax)
    return;

  pin = digitalx_mapping_in[pin];

  digitalx_t_left[pin] = digitalx_td[pin] = (int)
    ( ((float)v/(float)digitalx_vmax)*(float)digitalx_pwm_width );

}
