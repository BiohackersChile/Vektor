#include <math.h>
#include <Arduino.h>
#include <limits.h>
#include <Wire.h>

//if you use arduino ide, take those into your libraries folder
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//local sketch extra code
#include "digitalx.h"
#include "threader.h"
#include "vektor_mat.h"
// #include "vektor_motors.h"
/**
  For adafruit magsensor:
  http://www.adafruit.com/products/1746
*/
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/** gets the current manetometer direction */
void mag_dir(float vo[3]) {
  sensors_event_t event;
  mag.getEvent(&event);

  vo[0] = event.magnetic.x;
  vo[1] = event.magnetic.y;
  vo[2] = 0; //event.magnetic.z;
  /*
  Serial.println("mag dir  x "+String(event.magnetic.x)+
    " y: "+String(event.magnetic.y) );
    */
}

//////////////////////////////////////////////////////////////////////
////////////Engines configurations,  datastructures, functions
//////////////////////////////////////////////////////////////////////

/**
//For using the engines, make sure you setup the pin mappings, using
//engines_setup:
engines_setup({3,5,6,9})
//connects pin 3 to engine 0, pin 5 to engines 1, etc.

//then, add this function to your main loop:
engines_update_direction()
//it will update the engines voltaje config, and then apply them to the pins
*/

//////////////////////////////////////////////////////////////////////
////////////Those are your specific Vektor configurations
////////////after checking this, and using the functions highlited above
////////////just run the program ;)
//those three vector represent magnetometter orientation in space
float mag_vx[3] = {cos(M_PI/4), cos(M_PI/4), 0};
float mag_vy[3] = {-cos(M_PI/4), cos(M_PI/4), 0};
float mag_vz[3] = {0,0, 1};
//number of engines, equally distributed in circle
int engines_num;
//angle of the first engine relative to global coordinates, or just
//your geolocal phase relative to north pole
float engines_ang_offset=-M_PI/2;
//width of vibration wave, it will be linearly interpolated later
float engines_ang_width = M_PI*0.35f;
//////////////////////////////////////////////////////////////////////

//engines step, calculated angle space between engines
float engines_ang_step;
int * engines;  //voltaje from 0 to 255 for each engine
int * engines_map;  //a mapping: engines_map[engine] = pin (see engines_apply)
float engines_transform[9];  //coordinates transformation matrix

//those temporary vector and matrix data, let you optimize calculation
//avoid re-creating them, its better to rehuse the memory space
//(currently those are the only objects used)
float tmp_mag_dir[3];
float tmp_v0[3];
float tmp_v1[3];

/**
@desc setups engines data structures and variables
@param  engines_mapping the mapping from engines to pins:
    {engine_0_pin, engine_1_pin[, ..]}
*/
void engines_setup(int* engines_mapping, int eng_num) {
	engines_num = eng_num;
	engines_map = engines_mapping;

	engines = new int[eng_num];
	for(int i = 0;i<eng_num;i++)
		engines[i] = 0;

	create_transform(engines_transform, mag_vx, mag_vy, mag_vz);
	engines_ang_step = 2*M_PI/engines_num;
}

/**
@desc applies engines config to pin voltaje states, currently only
    supports native pwm enabled pins
*/
void engines_apply() {
  for(int i =0; i < engines_num; i++)
    analogXWrite( engines_map[i], engines[i] );
}

/**
This gets the magnetometter direction, transforms it to local cordinates
using the engines_transformacion matrix created on engines_setup, and
adjust each engines intensity acording to the configurations in
engines_ang*
*/
void engines_update_direction() {
  //get mag direction
  mag_dir(tmp_mag_dir);
  //transform coordinates
  mat_mult_vec(engines_transform, tmp_mag_dir, tmp_v0);

  //local magetometer angle
  float ang_mag = M_PI+ atan2(tmp_v0[1], tmp_v0[0]);//get flatten angle only
  //Serial.print("current angle ");
  //Serial.println(ang_mag);

  float ang_eng;    //engine angle
  float ang_diff;
  //relation between angle distance calculated from north and engine intensity
  float ang_coef = 255.0f / engines_ang_width;

  for(int i=0; i < engines_num;i++) { //for each engine
    ang_eng = engines_ang_step*i + engines_ang_offset;  //get local engine angle

    //get engine distance from local magnetometer angle
    ang_diff = min(	min(fabs(ang_eng - ang_mag - M_PI*2),
						fabs(ang_eng - ang_mag) ),
		      		fabs(ang_eng - ang_mag + M_PI*2));


    if(ang_diff > engines_ang_width)  //if engine angle scapes vibration width
      engines[i] = 0; //turn off
    else
      //adjust using linear interpolation
      engines[i] = int( (engines_ang_width-ang_diff)*ang_coef );
  }

  engines_apply();
}


//////////////////////////////////////////////////////////////////////
////////////Main program logic
//////////////////////////////////////////////////////////////////////

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

void engines_print() {
	int motor;
	for(motor=0; motor<engines_num; motor++) {
		Serial.print(String(motor)+":"+String(engines_map[motor])+", "+
			String(engines[motor])+",   ");
	}
	Serial.println();
}
void mag_print() {
	sensors_event_t event;
	mag.getEvent(&event);

	Serial.println("Mag direction x, y, z");
	Serial.print(
		"x: "+String(event.magnetic.x)+
		", y: "+String(event.magnetic.y)+
		", z: "+String(0));
	Serial.println();
}

void thread_update_digitalx(unsigned long dt) {
	analogXUpdate(dt, false);
}
void thread_update_digitalx_info(unsigned long dt) {
	analogXUpdate(dt, true);
}

void thread_update_engines(unsigned long dt) {
	engines_update_direction();
}

void thread_update_info(unsigned long dt) {
	// engines_print();
	// mag_print();
	analogXPrint(false, false);
}

void setup(void) {

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


	engines_setup( new int[8]{2, 3, 4, 5, 6, 7, 8, 9}, 8);
	Serial.println("Sensor and engines setup complete");
	Serial.println("---------------------------------");
	Serial.println("Engines mapping:");
	engines_print();

	/**
	DigitalX pin config, for using PWM in analog pins,
	currently on development
	*/
	digital_analog_setup();
	digital_analog_useall();
	analogXPrint(false, false);

	register_thread(thread_update_engines, (unsigned long)10*(unsigned long)1000);
	register_thread(thread_update_digitalx, 10);
	// register_thread(thread_update_digitalx_info, (unsigned long)1000*(unsigned long)1000);
	// register_thread(thread_update_info, (unsigned long)1000*(unsigned long)1000);

}


void loop(void) {
  thread_manager();
  delayMicroseconds(0);
}
