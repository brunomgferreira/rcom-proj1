// Alarm example
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include "alarm.h"

int alarm_enabled = FALSE;
int alarm_count = 0;

// Alarm function handler
void alarm_handler(int signal)
{
    alarm_enabled = FALSE;
    alarm_count++;

    // printf("Alarm #%d\n", alarm_count);
}