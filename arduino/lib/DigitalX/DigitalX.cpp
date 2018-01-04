#include <Arduino.h>
#include <limits.h>
#include <Wire.h>

//set enums, arduino pro mini:

int digitalx_vmax = 255;  //max voltaje
unsigned long digitalx_pwm_width = 2000;  //pwm vawe length

//number of digital pins, configure mapings for your arduino
//arduino pro mini is currently set

unsigned long * digitalx_t_left; //time left to update pin state
unsigned long * digitalx_td; //positive (high) time delay

bool * digitalx_high; //current state of pin;

int * digitalx_mapping_in;  //from arduino pin into internal index
int * digitalx_mapping; //from internal pin index into arduino pins

int digitalx_pinsmax; //max pins index to check from outside
int digitalx_pins;
bool * digitalx_mode; //if pin is to be handled


int mapping_arduino_mini_pro[7] = {1,2,4,7,8,12,13};
int mapping_arduino_uno[6] = {2,4,7,8,12,13};

/*
setups pins variables,
general, pwm settings
mappings (in, out)
data arrays
*/
void digital_analog_setup(int pins_length, int * pins_mapping ) {
	/*
	generate mappings
	mapping_in converts from real pin onto internal index
	holds -1 if it doesnt exists

	mapping holds from internal index into real index
	*/
	digitalx_pins = pins_length;
	digitalx_mapping = pins_mapping;


	//digitalx_mapping_in = int[ digitalx_mapping[digitalx_pins] ];
	//get max pin index value, store it in max
	int i,l, max=0;
	for(i=0, l=digitalx_pins; i<l; i++)
		if(digitalx_mapping[i] > max) max = digitalx_mapping[i];
	max++;


	digitalx_mapping_in = new int[max];
	for(i = 0, l = max; i < l; i++) {
		digitalx_mapping_in[i] = -1;

		for(int j=0; j < digitalx_pins; j++)
			if(digitalx_mapping[j] == i)
				digitalx_mapping_in[i] = j;
	}

	digitalx_pinsmax = max;

	digitalx_t_left = new unsigned long[digitalx_pins];
	digitalx_td = new unsigned long[digitalx_pins];
	digitalx_high = new bool[digitalx_pins];

	digitalx_mode = new bool[digitalx_pins];

	//fill empty data
	for(i=0, l=digitalx_pins; i<l;i++) {
		digitalx_mode[i] = false;
		digitalx_high[i] = false;
		digitalx_td[i] = digitalx_t_left[i] = 0;
	}

}

void digital_analog_setup() {
  digital_analog_setup(6, mapping_arduino_uno);
}

void analogXPrint(bool config=false, bool mapings=true) {
	if(config) {
		Serial.println("digitalx_pins:"+String( digitalx_pins));
		Serial.println("digitalx_pinsmax:"+String( digitalx_pinsmax));
	}

	if(mapings) {
		Serial.println("digitalx_mapping:");
		for(int i = 0; i<digitalx_pins; i++) {
			Serial.print(String(i)+": "+String(digitalx_mapping[i])+", ");
		}
		Serial.println();

		Serial.println("digitalx_mapping_in:");
		for(int i = 0; i<digitalx_pinsmax; i++) {
			Serial.print(String(i)+": "+String(digitalx_mapping_in[i])+", ");
		}
		Serial.println();
	}

	Serial.print("tleft:");
	for(int i=0; i<digitalx_pins;i++) {
		Serial.print(String(i)+":"+String(digitalx_t_left[i])+", ");
	}
	Serial.println();

	Serial.print("td:");
	for(int i=0; i<digitalx_pins;i++) {
		Serial.print(String(i)+":"+String(digitalx_td[i])+", ");
	}
	Serial.println();

	Serial.print("pin_high:");
	for(int i=0; i<digitalx_pins;i++) {
		Serial.print(String(i)+":"+String(digitalx_high[i])+", ");
	}
	Serial.println();

	Serial.print("pinmode:");
	for(int i=0; i<digitalx_pins;i++) {
		Serial.print(String(i)+":"+String(digitalx_mode[i])+", ");
	}
	Serial.println();

}

unsigned long flips = 0;
/*
executed by "thread" emulator, smaller delays betwen executions gives
better results
*/
void analogXUpdate(unsigned long dt, bool print) {
  unsigned long t_left;
  unsigned long td;
  int pin_to;
  int high;
  int pin;

  if(print) Serial.println(String(flips));

  //for each activated pin
  for(pin = 0; pin < digitalx_pins; pin++) {
    if(digitalx_mode[pin]) {
      pin_to = digitalx_mapping[pin];
      td = digitalx_td[pin];

      if(print)
        Serial.println("pin_to "+String(pin_to)+
          " td "+String(td));

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
        if(print)
          Serial.println(" tl "+String(t_left));

        if(t_left < 0 || t_left > digitalx_pwm_width*2) {
          flips++;
          high = digitalx_high[pin];

          if(t_left > digitalx_pwm_width*2)
            t_left = ULONG_MAX - t_left;

          //set new left time, if high, then time portion left from
          //pmw width - pin time delay, and just use pin time delay otherwise
          //subtracting t_left module in both cases
          digitalx_t_left[pin] = (high)?
            digitalx_pwm_width - (td + t_left):
            td - t_left;

          digitalWrite(pin_to, (high)? LOW:HIGH);
          digitalx_high[pin] = !high;
        }
        else {
          if(print)
            Serial.println("pre pin_tl "+String(digitalx_t_left[pin])+
              " tl "+String(t_left) );

          digitalx_t_left[pin] =  t_left;

          if(print)
          Serial.println("Post pin_tl: "+String(digitalx_t_left[pin])+
            " state "+String(digitalx_high[pin]));
        }
      }

    }
  }

}
unsigned long update_t_last = 1;

void analogXUpdate(void) {
  if(update_t_last == 1)
    update_t_last = micros();

  unsigned long t_current = micros();
  analogXUpdate(t_current - update_t_last, false);
}

/*
enable, disable pin
*/
void pinXMode(uint8_t pin, bool mode) {
	if(pin <= 0) return;

	pinMode(pin, OUTPUT);

	if(pin > digitalx_pinsmax) return;

	pin = digitalx_mapping_in[pin];
	if(pin != -1) digitalx_mode[ pin ] = mode;
}

void digital_analog_useall() {
	for(int i=0; i<digitalx_pins; i++) {
		pinXMode((uint8_t)digitalx_mapping[i], true);
	}
}

/*
from a voltaje, with max_voltaje specified in setup
*/
void analogXWrite(uint8_t pin, uint8_t v) {
	int pin_in;
	if(pin <= 0 || v < 0 || v > digitalx_vmax)	return;

	// Serial.println("analogXWrite("+String(pin)+", "+String(v)+") ");
	if(pin > digitalx_pinsmax) {
		analogWrite(pin, v);
		return;
	}
	pin_in = digitalx_mapping_in[pin];

	if(pin_in == -1) {
		analogWrite(pin, v);
		return;
	}

  digitalx_t_left[pin_in] = digitalx_td[pin_in] = (unsigned long)
    ( ((float)v/(float)digitalx_vmax)*(float)digitalx_pwm_width );

  // Serial.println("Configuring analog "+String(pin_in)+
  //   " tl "+String(digitalx_t_left[pin_in]));
}
