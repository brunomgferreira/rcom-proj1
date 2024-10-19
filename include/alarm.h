// Alarm example
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#define FALSE 0
#define TRUE 1

extern int alarm_enabled;
extern int alarm_count;

// Alarm function handler
void alarm_handler(int signal);
