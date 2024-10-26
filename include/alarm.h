#ifndef _ALARM_H_
#define _ALARM_H_

#define FALSE 0
#define TRUE 1

extern int alarm_enabled;
extern int alarm_count;

// Alarm function handler
void alarm_handler(int signal);

#endif // _ALARM_H_
