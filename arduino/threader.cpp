#include <Arduino.h>
#include "threader.h"

Thread_desc * threads = new Thread_desc[10];
int threads_len = 0;
int threads_len_max = 10;

void register_thread( void (*exec) (unsigned long micros), unsigned long delay_micros ) {
  if(threads_len == threads_len_max) return;

  Thread_desc thread(exec, delay_micros);
  threads[threads_len] = thread;
  threads_len++;
}


unsigned long t_last=1, t_current;

void thread_manager() {
  //Serial.println("thread manager");
  if(t_last == 1)  t_last = micros();
  t_current = micros();

  bool rollover = false;
  if(t_last > t_current) {
    rollover = true;
    t_last = 0;
  }

  Thread_desc * thread;
  //Serial.println("t_last " + String(t_last));
  //Serial.println("t_current " + String(t_current));

  for(int i=0; i < threads_len; i++) {
    //Serial.println("thread " + String(i)+ " of "+ String(threads_len) );
    thread = &threads[i];

    //Serial.println( "t.next "+String(thread->t_next) );
    //Serial.println( "t.delay "+String(thread->delay_micros) );

    if(
      (thread->t_next < t_current && thread->t_next != 1 && !thread->executing )
      || thread->t_next == 1 || rollover ) {

      thread->t_next = t_current + thread->delay_micros;
      //Serial.println("elapsed ");
      thread->executing = true;
      thread->exec(t_current - t_last);
      thread->executing = false;
    }

  }

  t_last = t_current;
}
