#include <Arduino.h>
#include <limits.h>
/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746

  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

//pines PWM analogicos: 3 5 6 9 10 11
//delayMicroseconds()

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

int pin_a = 3;

void setup(void) {
  pinMode(2, OUTPUT);

  pinMode(3, OUTPUT);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
    /**
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);

    pinMode(A6, OUTPUT);
    pinMode(A7, OUTPUT);
    */

    Serial.begin(9600);
    Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

    /* Initialise the sensor */
    if (!mag.begin()) {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while (1);
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();
}

bool mots = false;

void loop_frame(void) {
  // Get a new sensor event
  sensors_event_t event;
  Serial.print("H");
  mag.getEvent(&event);
  Serial.print("H");

  // Display the results (magnetic vector values are in micro-Tesla (uT))
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle',
  //which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment ouut these two lines, your
  //compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.u
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.usb
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  for (int i = 0; i < 25; i += 1) {
    /*
    analogWrite(3, i * 10);

    analogWrite(5, i * 10);
    analogWrite(6, i * 10);

    analogWrite(9, i * 10);
    analogWrite(10, i * 10);
    analogWrite(11, i * 10);
    */

    /**
    analogWrite(A0, i * 10);
    analogWrite(A1, i * 10);
    analogWrite(A2, i * 10);
    analogWrite(A3, i * 10);

    analogWrite(A6, i * 10);
    analogWrite(A7, i * 10);

    Serial.println("#");
    */
    delay(50);
  }
}


void (*f)(void) = &loop_frame;

unsigned long t_last = micros();

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

bool high = false;
int t_left = 60;

void loop(void) {

  loop_frame();

  /*
  unsigned long t_current = micros();
  unsigned long dt = (int) (t_last > t_current)?
            ULONG_MAX + t_current : t_current - t_last;

  t_left -= dt;
  if(t_left < 0) {
    t_left = 60+t_left;
    digitalWrite(2,  (high)? HIGH: LOW);
    high = !high;
  }

  t_last = t_current;
  */
}

void loop_(void) {
  unsigned long t_current = micros();
  unsigned long dt = (int) (t_last > t_current)?
            ULONG_MAX + t_current : t_current - t_last;

  t_last = t_current;
}
