#ifndef VEKTOR_THREADER
#define VEKTOR_THREADER

#include <Wire.h>

/*
holds delay data
*/
struct Thread_desc {
  bool executing;
  void  (*exec) (unsigned long micros);  //function to execute
  unsigned long delay_micros;  //time between repetitions
  unsigned long t_next;

  Thread_desc( void (*exec)(unsigned long micros), unsigned long delay_micros)
    : executing(false), exec(exec),
      delay_micros(delay_micros), t_next(1) {}

  Thread_desc() {}
};

void register_thread( void (*exec) (unsigned long micros), unsigned long delay_micros );

void thread_manager();

#endif //VEKTOR_THREADER
