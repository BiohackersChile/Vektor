#ifndef VEKTOR_THREADER
#define VEKTOR_THREADER

#include <Wire.h>

/**
add a thread to the thread_manager
*/
void register_thread( void (*exec) (unsigned long micros), unsigned long delay_micros );

/**
Executes the threads., call it on every loop cycle
*/
void thread_manager();

#endif //VEKTOR_THREADER
