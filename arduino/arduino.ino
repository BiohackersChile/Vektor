#include <Arduino.h>
#include <limits.h>
#include <Wire.h>
#include <DigitalAnalog.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <math.h>

/**
  For adafruit magsensor:
  http://www.adafruit.com/products/1746
*/
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//////////////////////////////////////////////////////////////////////
////////////Mat functions
//////////////////////////////////////////////////////////////////////
/** creates a new empty vector */
float * create_vec() {
  float vec[3] = {0.0f ,0.0f ,0.0f};
  return vec;
}

/** vi = vi/mod(vi) */
void normalize(float * vi) {
  float l =  sqrt( double(vi[0]*vi[0] + vi[1]*vi[1] + vi[2]*vi[2]) );
  vi[0] /= l;
  vi[1] /= l;
  vi[2] /= l;
}

/** makes  mat = (vx, vy, vz)
  wich represents a tranformation from coordinates relative to vx,y,z to
  the ones on wich they are defined
*/
void create_transform(float* mat,float* vx, float* vy, float* vz) {
  mat[0] = vx[0];
  mat[1] = vx[1];
  mat[2] = vx[2];

  mat[3] = vy[0];
  mat[4] = vy[1];
  mat[5] = vy[2];

  mat[6] = vz[0];
  mat[7] = vz[1];
  mat[8] = vz[2];
}

/** makes vo = mat*vi */
void mat_mult_vec(float* mat, float* vi, float* vo) {
  vo[0] = vi[0]*mat[0] + vi[1]*mat[3] + vi[2]*mat[6];
  vo[1] = vi[0]*mat[1] + vi[1]*mat[4] + vi[2]*mat[7];
  vo[2] = vi[0]*mat[2] + vi[1]*mat[5] + vi[2]*mat[8];
}

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
int engines_num=4;
//angle of the first engine relative to global coordinates
float engines_ang_offset=0;
//width of vibration wave, it will be linearly interpolated later
float engines_ang_width = (M_PI*0.75f)/2.0f;
//////////////////////////////////////////////////////////////////////

//engines step, calculated angle space between engines
float engines_ang_step;
int * engines;  //voltaje from 0 to 255 for each engine
int engines_map[4];  //a mapping: engines_map[engine] = pin (see engines_apply)
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
void engines_setup(int engines_mapping[4]) {
  engines = new int[engines_num];
  engines_map[0] = engines_mapping[0];
  engines_map[1] = engines_mapping[1];
  engines_map[2] = engines_mapping[2];
  engines_map[3] = engines_mapping[3];

  //Serial.println( "Starting!:"+engines_mapping[0] );

  create_transform(engines_transform, mag_vx, mag_vy, mag_vz);
  engines_ang_step = 2*M_PI/engines_num;
}

/**
@desc applies engines config to pin voltaje states, currently only
    supports native pwm enabled pins
*/
void engines_apply() {
  int i, pin =0;


  for(i=0;i < engines_num; i++) {
    pin = engines_map[i];
    analogWrite( pin, engines[i] );
  }
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

  //loop variables:
  int i;
  //magetometer angle
  float ang_mag = M_PI+ atan2(tmp_mag_dir[1], tmp_mag_dir[0]);//get flatten angle only
  float ang_eng;    //engine angle
  float ang_diff;
  //relation between angle distance calculated from north and engine intensity
  float ang_coef = 255.0f / engines_ang_width;

  for(i=0; i < engines_num;i++) { //for each engine
    ang_eng = engines_ang_step*i + engines_ang_offset;  //get engine angle
    ang_diff = fabs(ang_eng - ang_mag);
    ang_diff = min(min(fabs(ang_eng - ang_mag - M_PI*2), fabs(ang_eng - ang_mag) ),
      fabs(ang_eng - ang_mag + M_PI*2)) ;


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

void setup(void) {
  int mapping[4] = {3,9,6,5};
  engines_setup(mapping);


  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);

  Serial.begin(9600);

  Serial.println("mapping "+String(engines_map[0]));
  Serial.println("mapping "+String(engines_map[1]));
  Serial.println("mapping "+String(engines_map[2]));
  Serial.println("mapping "+String(engines_map[3]));

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


void loop(void) {
  engines_update_direction();

  delay(10);
}
